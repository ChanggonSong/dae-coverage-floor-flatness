# dae_coverage_floor_flatness/mission_execution/mission_executor.py

import os
import sys
import yaml
import json
import csv
import time
import math
import traceback

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# 경량 유틸리티 모듈
try:
    from utils.map_utils import get_map_bounds
    from utils.ros_utils import create_pose_stamped, teleport_gazebo_entity
    from utils.nav2_utils import apply_nav2_monkey_patches
    from utils.visualizer import visualize_paths
except ImportError:
    from mission_execution.utils.map_utils import get_map_bounds
    from mission_execution.utils.ros_utils import create_pose_stamped, teleport_gazebo_entity
    from mission_execution.utils.nav2_utils import apply_nav2_monkey_patches
    from mission_execution.utils.visualizer import visualize_paths


class MissionExecutor(Node):
    """
    Nav2 기반 미션 실행을 담당하는 ROS 2 노드.

    is_sim 여부는 더 이상 input()으로 받지 않고, ROS 2 파라미터(launch argument)로
    주입받는다. 예: ros2 launch dae_coverage_floor_flatness mission_execution.launch.py is_sim:=true

    주행 방식: final_path.json 전체를 방향(heading)이 바뀌는 지점 기준으로만
    재분할한다(_split_into_straight_subsegments). coverage/transit 구분 없이
    모든 직선 sub-segment에서 동일하게 처리한다 (_execute_capture_subsegment):
      제자리 회전(Spin, 절대각 차이 기반) -> 1초 정착 대기 -> surface_profiler에
      캡처 시작 신호 -> 구간 끝점까지 직선 주행(NavigateToPose)하며 계속 캡처 ->
      도착 시 자동 정지 -> 캡처 종료 신호.
    회전/정착 중에는 캡처하지 않고 오직 등속 직선 구간에서만 캡처한다(회전 시
    AMCL 오차가 가장 크기 때문). 실측 결과 coverage뿐 아니라 transit 구간도
    연속으로 측정해야 라이다 blind zone이 충분히 메꿔지는 것으로 확인되어,
    coverage/transit을 다르게 처리하던 이전 방식(FollowWaypoints 전용 transit)을
    폐기하고 이 통합 방식으로 전환하였다.
    각 sub-segment는 별도 goal로 순차 전송되므로, 미션 전체의 성공/실패는
    self.mission_succeeded 플래그로 별도 추적한다(navigator.getResult()는
    마지막으로 전송된 goal 하나의 결과만 반영하기 때문).
    """

    def __init__(self):
        super().__init__('mission_executor_node')

        print("\n=======================================================")
        print("[*] Nav2 Mission Executor (Jetson / Sim Controller)")
        print("=======================================================\n")

        # self.spin_executor
        # self(MissionExecutor) 전용 SingleThreadedExecutor.
        # navigator(BasicNavigator)는 nav2_simple_commander 라이브러리 내부에서
        # 자체적으로 rclpy.spin_once(navigator, ...)를 호출하므로, 이중 spin 충돌을
        # 피하기 위해 별도 executor에 등록하지 않고 기존 방식 그대로 독립 관리한다.
        self.spin_executor = SingleThreadedExecutor()
        self.spin_executor.add_node(self)

        # 상태 변수 초기화
        self.config = None
        self.global_cfg = None
        self.env_cfg = None
        self.mission_exec_cfg = None
        self.workspace_root = None
        self.map_bounds = None
        self.final_path = None
        self.navigator = None

        # FollowWaypoints는 coverage 지점마다 여러 세그먼트(goal)로 나뉘어 전송되므로,
        # 미션 전체의 성공 여부는 navigator.getResult()(마지막 세그먼트만 반영)가 아닌
        # 이 플래그로 별도 추적한다.
        self.mission_succeeded = False

        # 주행 모니터링 관련 상태
        self.current_amcl_x = None
        self.current_amcl_y = None
        self.last_valid_amcl_pose = None
        self.max_allowed_jump = 0.60

        # 연속 점프 감지용 상태. 단발성 AMCL 보정(긴 직선 주행 후 누적된
        # dead-reckoning 오차가 한 번에 정상화되는 경우 등)까지 비상 상황으로
        # 취급해 미션을 중단시키면 너무 과민하므로, 짧은 시간 안에 여러 번
        # 반복될 때만 진짜 비상(로컬라이제이션 붕괴/텔레포트)으로 간주한다.
        self.amcl_jump_timestamps = []
        self.amcl_jump_window_sec = 5.0     # 이 시간 안에
        self.amcl_jump_count_threshold = 3  # 이만큼 반복되면 비상으로 격상
        self.path_history = []
        self._last_record_time = 0.0
        self.sub_amcl_check = None

        # AMCL 초기화 검증 관련 상태
        self.verified_amcl_x = None
        self.verified_amcl_y = None
        self.initial_pose = None
        self.sub_verify = None

        # surface_profiling(노트북) 측 PCD 수집 종료를 알리는 서비스 클라이언트.
        # 정상 종료와 비정상(실패/타임아웃) 종료를 서로 다른 서비스로 분리하여 호출한다.
        self.stop_collection_success_client = self.create_client(
            Trigger, '/surface_profiling/stop_collection_success'
        )
        self.stop_collection_abort_client = self.create_client(
            Trigger, '/surface_profiling/stop_collection_abort'
        )

        # surface_profiling(노트북) 측 지점별(Waypoint) 캡처 시작/종료 서비스 클라이언트.
        # coverage 웨이포인트에 정지할 때마다 한 쌍(start -> stop)씩 호출한다.
        self.start_capture_client = self.create_client(
            Trigger, '/surface_profiling/start_waypoint_capture'
        )
        self.stop_capture_client = self.create_client(
            Trigger, '/surface_profiling/stop_waypoint_capture'
        )

        # 1. ROS 2 파라미터로 시뮬레이션 모드 여부 확보 (launch argument로 주입됨)
        self._resolve_run_mode()

        # 2. params.yaml 설정 파일 파싱
        self._load_config()

        # 3. 글로벌 맵 바운즈 확보
        self._load_map_bounds()

        # 4. 이전 파이프라인에서 생성된, 보간/샘플링이 끝난 JSON 원본 경로 파일 로드
        self._load_final_path()

    # ------------------------------------------------------------------
    # 초기화 단계 헬퍼
    # ------------------------------------------------------------------

    def _resolve_run_mode(self):
        """
        'is_sim' ROS 2 파라미터를 선언하고 읽는다. launch 파일이 주입하지 않으면
        기본값 False(Real-world)로 동작한다.

        use_sim_time은 launch 파일이 is_sim과 함께 주입하는 것이 표준이지만,
        혹시 누락되더라도 안전하게 동작하도록 is_sim 값으로부터 use_sim_time을
        다시 추론하여 자기 자신에게 강제 적용한다.
        """
        self.declare_parameter('is_sim', False)
        self.is_sim = self.get_parameter('is_sim').get_parameter_value().bool_value

        if self.is_sim:
            self.get_logger().info("Running Simulation(Gazebo) Mode.")
        else:
            self.get_logger().info("Running Real-world Mode.")

        # use_sim_time 강제 동기화 (launch 파일이 누락했을 경우의 안전장치)
        use_sim_time_param = self.get_parameter_or(
            'use_sim_time', Parameter('use_sim_time', Parameter.Type.BOOL, self.is_sim)
        )
        if use_sim_time_param.value != self.is_sim:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, self.is_sim)])

    def _load_config(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share_dir = get_package_share_directory('dae_coverage_floor_flatness')
            config_path = os.path.join(package_share_dir, 'config', 'params.yaml')
        except Exception:
            # mission_execution 내에서 실행 시 프로젝트 루트의 config로 fallback 탐색
            base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
            config_path = os.path.join(base_dir, "config", "params.yaml")

        print(f"[*] Resolving parameters from: {config_path}")
        if not os.path.exists(config_path):
            print(f"[!] Critical Error: params.yaml file not found at {config_path}")
            sys.exit(1)

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.global_cfg = self.config.get('global', {})
        self.workspace_root = os.path.expanduser(
            self.global_cfg.get('workspace_root', '~/dae_floor_maps')
        )
        self.env_cfg = self.config.get('environment_modeling', {})
        self.mission_exec_cfg = self.config.get('mission_execution', {})

    def _load_map_bounds(self):
        grid_dir = os.path.join(self.workspace_root, self.env_cfg.get('output_grid_dir', 'maps/grid'))
        yaml_path = os.path.normpath(os.path.join(grid_dir, "map_from_dae.yaml"))

        if not os.path.exists(yaml_path):
            print(f"[!] CRITICAL ERROR: Map bounds data '{yaml_path}' not found!")
            sys.exit(1)

        self.map_bounds = get_map_bounds(yaml_path)

    def _load_final_path(self):
        metric_dir = os.path.join(self.workspace_root, self.mission_exec_cfg.get('input_metric_dir', 'analytics/metrics'))
        cache_file = os.path.normpath(os.path.join(metric_dir, "final_path.json"))

        if not os.path.exists(cache_file):
            print(f"[!] CRITICAL ERROR: Mission route '{cache_file}' not found!")
            print("[-] Please run 'run_generation_pipeline.py' on your workstation first.")
            sys.exit(1)

        print(f"[*] Found pre-generated path at {cache_file}. Loading...")
        with open(cache_file, 'r') as f:
            self.final_path = json.load(f)
        print(f"[*] Path loaded. Total raw waypoints: {len(self.final_path)}")

    # ------------------------------------------------------------------
    # ROS 환경 셋업
    # ------------------------------------------------------------------

    def _setup_ros_environment(self):
        """
        BasicNavigator 인스턴스 생성, Gazebo 환경일 경우 초기 텔레포트 인터페이스 수행.

        rclpy.init()은 더 이상 이 메서드에서 호출하지 않음. 이 노드(self) 자신을
        포함한 전체 ROS 2 컨텍스트는 main()에서 이미 단일하게 초기화되어 있다고 가정함.
        """
        print("[*] Connecting to Nav2 Server...")

        self.navigator = BasicNavigator()

        # BasicNavigator 레벨 몽키 패치
        # navigator 인자는 호출 시점 일관성을 위해 받지만, 패치는 클래스 자체에 적용되므로
        # 인스턴스 유무와 무관하게 한 번만 적용되면 모든 BasicNavigator 인스턴스에 영향을 줌.
        apply_nav2_monkey_patches(self.navigator)

        if self.is_sim:
            # AMCL의 initial_pose 힌트(_initialize_localization에서 final_path[0]의
            # orientation으로 설정됨)와 실제 Gazebo 로봇의 물리적 방향이 어긋나면,
            # AMCL이 '틀린 방향을 정답'이라고 믿은 채로 위치(x,y)만 짧은 시간에
            # 수렴해버릴 수 있다(position error는 작아도 orientation error는 크게
            # 남는 상태). 이후의 모든 제자리 회전(_rotate_in_place_to)이 이 틀린
            # 기준 위에서 계산되어 실제 방과 어긋난 방향으로 주행하게 되므로,
            # 텔레포트 시점부터 반드시 orientation까지 final_path[0]과 동일하게
            # 맞춰야 한다. create_pose_stamped를 재사용해 AMCL 힌트(아래
            # _initialize_localization)와 완전히 동일한 값이 되도록 보장한다.
            # (create_pose_stamped는 타임스탬프 용도로만 navigator.get_clock()을
            # 쓰므로, 이 호출 전에 navigator가 먼저 생성되어 있어야 한다.)
            temp_pose = create_pose_stamped(self.navigator, self.final_path[0], self.map_bounds)
            temp_pose.pose.position.z = 0.05

            teleport_gazebo_entity(temp_pose)
            print("[+] Gazebo Teleportation Successful (position + orientation matched to first waypoint).")
        else:
            print("[*] Real-world Mode: Skipping Gazebo Robot location Initializing.")

        if self.is_sim:
            print("[*] Waiting for Gazebo /clock synchronization...")
            while self.navigator.get_clock().now().nanoseconds == 0:
                rclpy.spin_once(self.navigator, timeout_sec=0.01)
            print("[+] Gazebo clock synchronization done.")

        print("[*] Waiting for Nav2 nodes to become fully Active...")
        self.navigator.waitUntilNav2Active()
        print("[+] Nav2 is now fully Active. Securing subscription margin...")
        time.sleep(4.0)

        # coverage run 시작 시 '제자리 회전'을 위해 현재 로봇의 실시간 헤딩(map->base_link)이
        # 필요하다. /amcl_pose 토픽(저주파, AMCL 보정 시점에만 갱신)이 아니라 TF buffer를
        # 직접 조회하는 이유는, TF는 odom 기반으로 계속 보간되어 훨씬 더 실시간에 가까운
        # 값을 주기 때문이다(회전 판단 시점의 정확도가 중요).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # ------------------------------------------------------------------
    # AMCL 초기 위치 수렴
    # ------------------------------------------------------------------

    def _target_verification_callback(self, msg):
        self.verified_amcl_x = msg.pose.pose.position.x
        self.verified_amcl_y = msg.pose.pose.position.y

    def _initialize_localization(self):
        """
        AMCL 초기 위치 수렴 스테이지 제어.
        수렴 실패/타임아웃 시 안전하게 시스템을 다운시키고 에러 코드로 종료한다.

        '/amcl_pose' Subscription은 self(MissionExecutor) 노드에 생성한다.
        따라서 이 단계의 spin은 self를 기준으로 수행한다.
        """
        print("[*] Entering Robust AMCL Initialization Stage...")

        self.sub_verify = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._target_verification_callback, 10
        )
        self.initial_pose = create_pose_stamped(self.navigator, self.final_path[0], self.map_bounds)

        start_verify_time = time.time()
        is_localization_safe = False
        publish_interval = 0.5
        last_publish_time = 0.0

        print("[*] Dynamically injecting Initial Pose until AMCL responds...")
        while time.time() - start_verify_time < 20.0:
            self.spin_executor.spin_once(timeout_sec=0.05)
            current_time = time.time()

            if self.verified_amcl_x == 0.0 or self.verified_amcl_x is None:
                if current_time - last_publish_time >= publish_interval:
                    self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                    self.initial_pose.pose.position.z = 0.0
                    print(f"  └─> [Pulse] Sending Initial Pose. Stamp Sec: {self.initial_pose.header.stamp.sec}")
                    self.navigator.setInitialPose(self.initial_pose)
                    last_publish_time = current_time
            else:
                dx = self.verified_amcl_x - self.initial_pose.pose.position.x
                dy = self.verified_amcl_y - self.initial_pose.pose.position.y
                error_dist = (dx ** 2 + dy ** 2) ** 0.5

                if error_dist > 0.5:
                    print(f"\n[!!! CRITICAL INITIALIZATION BLOCKED !!!] AMCL initialized in the WRONG ROOM!")
                    print(f"[-] Target: ({self.initial_pose.pose.position.x:.2f}, {self.initial_pose.pose.position.y:.2f})")
                    print(f"[-] AMCL Refused and went to: ({self.verified_amcl_x:.2f}, {self.verified_amcl_y:.2f})")
                    self.navigator.cancelTask()
                    self._notify_surface_profiling_stop(success=False, message="AMCL initialized in the wrong room.")
                    self.destroy_subscription(self.sub_verify)
                    self.spin_executor.remove_node(self)
                    self.destroy_node()
                    self.navigator.destroy_node()
                    rclpy.shutdown()
                    sys.exit(1)
                else:
                    print(f"\n[+] AMCL Successfully aligned within safe zone (Error: {error_dist:.3f}m).")
                    is_localization_safe = True
                    break
            time.sleep(0.05)

        if not is_localization_safe:
            print("[-] localization verification Failed.")
            self._notify_surface_profiling_stop(success=False, message="Localization verification timed out.")
            self.destroy_subscription(self.sub_verify)
            self.spin_executor.remove_node(self)
            self.destroy_node()
            self.navigator.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

        wait_sec = self.mission_exec_cfg.get('post_localization_wait_sec', 3.0)
        if wait_sec > 0:
            print(f"[*] Holding position for {wait_sec:.1f}s to let AMCL settle...")
            wait_start = time.time()
            while time.time() - wait_start < wait_sec:
                self.spin_executor.spin_once(timeout_sec=0.05)
                time.sleep(0.05)

        if self.is_sim:
            print("[*] Clearing costmaps explicitly after simulation teleport & AMCL convergence...")
            self.navigator.clearAllCostmaps()

        self.destroy_subscription(self.sub_verify)
        print("[+] Initialization Stage Cleared. Moving to Path Sampling...")

    # ------------------------------------------------------------------
    # 경로 샘플링 (클램핑 전용 — 보간/샘플링은 mission_planner.py에서 완료됨)
    # ------------------------------------------------------------------

    def _prepare_goal_poses(self):
        target_margin = self.env_cfg.get('robot_width', 0.28) * 5

        goal_poses = []
        out_of_bounds_count = 0

        safe_bounds = {
            'min_x': self.map_bounds['min_x'] + target_margin,
            'max_x': self.map_bounds['max_x'] - target_margin,
            'min_y': self.map_bounds['min_y'] + target_margin,
            'max_y': self.map_bounds['max_y'] - target_margin
        }

        for wp_dict in self.final_path:
            pose_stamped = create_pose_stamped(self.navigator, wp_dict, safe_bounds)
            goal_poses.append(pose_stamped)

            if (abs(pose_stamped.pose.position.x - wp_dict['pose']['position']['x']) > 0.01 or
                    abs(pose_stamped.pose.position.y - wp_dict['pose']['position']['y']) > 0.01):
                out_of_bounds_count += 1

        if out_of_bounds_count > 0:
            print(f"[!] Warning: {out_of_bounds_count} waypoints were nudged into the safe map zone.")

        return goal_poses

    # ------------------------------------------------------------------
    # 주행 모니터링 콜백
    # ------------------------------------------------------------------

    def _amcl_monitor_callback(self, msg):
        self.current_amcl_x = msg.pose.pose.position.x
        self.current_amcl_y = msg.pose.pose.position.y
        curr_time = time.time()

        # 5Hz 샘플링 (약 0.5초 간격으로 기록)
        if curr_time - self._last_record_time >= 0.5:
            self.path_history.append([curr_time, self.current_amcl_x, self.current_amcl_y])
            self._last_record_time = curr_time

        self.path_history.append([time.time(), self.current_amcl_x, self.current_amcl_y])

    # ------------------------------------------------------------------
    # 미션 실행 메인 루프
    # ------------------------------------------------------------------

    def execute_mission(self):
        """
        '/amcl_pose' 모니터링 Subscription은 self(MissionExecutor) 노드에 생성하며,
        self.spin_executor(SingleThreadedExecutor)로 spin한다. navigator.isTaskComplete()/
        getFeedback() 등 BasicNavigator 자체의 내부 동작은 nav2_simple_commander
        라이브러리 구현상 navigator 자신을 별도로 spin해야 하므로, 모니터링 루프
        안에서는 self.executor와 navigator를 각각 독립적으로 spin한다.

        방향전환 기반 통합 상태기계:
        coverage(F2C 스와스)와 transit(A* 커넥터) 모두 바닥을 연속으로 측정해야
        blind zone(라이다 최소 측정거리 사각지대)이 충분히 메꿔진다는 실측
        결과에 따라, 더 이상 coverage/transit을 다른 방식으로 다루지 않는다.
        전체 final_path를 "방향(heading)이 바뀌는 지점"만 기준으로 재분할해서,
        모든 직선 구간에서 동일한 패턴을 반복한다:

            제자리 회전(Spin, 직전 구간과의 절대각 차이) -> 1초 정착 대기 ->
            캡처 시작 -> 구간 끝점까지 직선 주행하며 계속 캡처 -> 도착 시
            자동 정지 -> 캡처 종료(그동안 모은 포인트 취합).

        회전/정착 중에는 절대 캡처하지 않고 오직 등속 직선 구간에서만 캡처한다
        (회전 시 AMCL 오차가 가장 크기 때문). 구간 경계는 각 웨이포인트에 이미
        기록된 orientation(translator.py가 진행방향 기준으로 계산해둔 값)을
        연속 비교해서 찾는다 — direction_change_threshold_deg를 넘는 지점마다
        새 구간 시작. 원래 coverage 세그먼트 하나(F2C 스와스)는 태생적으로
        직선이라 항상 구간 하나 그대로 유지되고, 예전에는 통으로 FollowWaypoints
        처리하던 transit(특히 A* 경로로 꺾임이 있는 구간)는 이제 꺾이는 지점마다
        자동으로 잘게 쪼개져서 각각 정지-측정된다. 좁은 방/복도가 단일 점으로
        축약되는 경우도 이 분할 로직에서 자연스럽게 길이 1짜리 구간으로 처리되어
        별도의 특수 케이스 코드가 필요 없다.

        각 구간은 별도 goal로 순차 전송되므로, 미션 전체의 성공/실패는
        self.mission_succeeded 플래그로 별도 추적한다(navigator.getResult()는
        마지막으로 전송된 goal 하나의 결과만 반영하기 때문).
        """
        env_text = "in Gazebo" if self.is_sim else "to Real-world Robot"
        print(f"[*] Executing Mission {env_text} (Direction-Change-Based Continuous Capture)...")

        goal_poses = self._prepare_goal_poses()
        sub_segments = self._split_into_straight_subsegments(goal_poses)
        print(f"[*] Total waypoints: {len(goal_poses)}, split into {len(sub_segments)} "
              f"straight sub-segments (coverage + transit measured continuously).")

        self.last_valid_amcl_pose = (self.initial_pose.pose.position.x, self.initial_pose.pose.position.y)
        self.amcl_jump_timestamps = []
        self.sub_amcl_check = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_monitor_callback, 10
        )

        self.mission_succeeded = True

        for seg_idx, (s, e) in enumerate(sub_segments):
            seg_poses = goal_poses[s:e]
            seg_header = self.final_path[s]['header']
            seg_type_prefix = seg_header['task_type'].split('_')[0]
            record_pcd = seg_header.get('record_pcd', True)

            print(f"\n[>>>] Sub-segment {seg_idx + 1}/{len(sub_segments)}: "
                f"origin_type='{seg_type_prefix}', record_pcd={record_pcd}, "
                f"points={len(seg_poses)} (global idx {s}~{e - 1})")

            is_genuine_single = self.final_path[s]['header']['task_type'].endswith('_single')
            ok = self._execute_capture_subsegment(seg_poses, record_pcd=record_pcd, is_genuine_single=is_genuine_single)

            if not ok:
                print(f"[-] Sub-segment {seg_idx + 1} failed. Aborting mission.")
                self.mission_succeeded = False
                break

        if self.sub_amcl_check is not None:
            self.destroy_subscription(self.sub_amcl_check)
            self.sub_amcl_check = None

        if not self.mission_succeeded and not self.navigator.isTaskComplete():
            self.navigator.cancelTask()

    def _split_into_straight_subsegments(self, goal_poses):
        """
        전체 경로(goal_poses)를 방향(heading)이 바뀌는 지점마다 분할한다.
        각 점에 이미 기록된 orientation(진행방향 기준)을 연속 비교해서, yaw
        변화량이 direction_change_threshold_deg(기본 15도)를 넘으면 그 지점을
        새 구간의 시작으로 삼는다. 반환값은 [(start_idx, end_idx_exclusive), ...].

        note: min_rotation_deg(_rotate_in_place_to에서 사용, 기본 3도)보다 이
        threshold를 더 크게 잡는 이유는, 미세한 리샘플링/부동소수점 오차로
        생기는 각도 잡음까지 매번 별도 구간(=매번 정지)으로 나눠버리면 불필요한
        정지가 과도하게 늘어나기 때문이다.
        """
        n = len(goal_poses)
        if n <= 1:
            return [(0, n)]

        threshold = math.radians(self.mission_exec_cfg.get('direction_change_threshold_deg', 15.0))

        segments = []
        seg_start = 0
        prev_yaw = self._quaternion_to_yaw(goal_poses[0].pose.orientation)
        prev_record = self.final_path[0]['header'].get('record_pcd', True)

        for i in range(1, n):
            curr_yaw = self._quaternion_to_yaw(goal_poses[i].pose.orientation)
            dyaw = curr_yaw - prev_yaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))  # -pi ~ +pi 정규화

            curr_record = self.final_path[i]['header'].get('record_pcd', True)
            record_changed = (curr_record != prev_record)

            if abs(dyaw) > threshold or record_changed:
                segments.append((seg_start, i))
                seg_start = i

            prev_yaw = curr_yaw
            prev_record = curr_record

        segments.append((seg_start, n))

        # 코너와 코너가 바로 이웃해서 생기는 고립된 1점짜리 sub-segment(진짜 F2C 단일점 방이 '_single' 태그가 아닌 경우)는 별도로 세워서
        # 처리하지 않고, 다음 sub-segment 맨 앞에 편입시킴.
        # 그러면 그 코너점이 goThroughPoses의 경유점 중 하나로 포함되게 됨. 즉 혼자 남겨지는 상황을 방지하는 것.
        merged_segments = []
        i = 0
        while i < len(segments):
            s, e = segments[i]
            is_trivial_single = (
                (e - s == 1)
                and not self.final_path[s]['header']['task_type'].endswith('_single')
                and i + 1 < len(segments)
            )
            if is_trivial_single:
                _, next_e = segments[i + 1]
                merged_segments.append((s, next_e))
                i += 2
            else:
                merged_segments.append((s, e))
                i += 1

        return merged_segments

    # ------------------------------------------------------------------
    # AMCL 점프 감지 (여러 모니터링 루프에서 공용으로 재사용)
    # ------------------------------------------------------------------

    def _check_amcl_jump(self):
        """
        직전에 기록된 AMCL pose 대비 순간 이동 거리가 임계치(self.max_allowed_jump)를
        넘으면 '점프 후보'로 기록한다. 하지만 단발성 점프(예: 긴 직선 구간을 도는
        동안 누적된 dead-reckoning 오차가 AMCL의 정상적인 재정렬로 한 번에 보정되는
        경우)는 실제로는 위험이 아니라 오히려 위치 추정이 더 정확해진 것이므로,
        그것만으로 미션을 중단시키지 않는다. amcl_jump_window_sec 안에
        amcl_jump_count_threshold번 이상 반복될 때만 진짜 비상(로컬라이제이션
        붕괴, 텔레포트 등)으로 간주하여 True를 반환한다.

        중요: last_valid_amcl_pose는 점프 판정 여부와 무관하게 '항상' 현재 값으로
        갱신한다. 예전 코드는 점프로 판정된 순간 이 갱신을 건너뛰어서, 그 다음부터는
        AMCL이 이미 새로운 위치에 안정적으로 자리잡았어도 계속 옛날 기준점과
        비교해 '같은 점프'를 영원히 재판정하는 버그가 있었다(비상 상황에서 절대
        복구되지 않고 미션이 항상 중단됨). 매번 갱신하면 이 문제가 원천 해결된다.
        """
        if self.current_amcl_x is None or self.current_amcl_y is None:
            return False

        is_emergency = False

        if self.last_valid_amcl_pose is not None:
            dx = self.current_amcl_x - self.last_valid_amcl_pose[0]
            dy = self.current_amcl_y - self.last_valid_amcl_pose[1]
            jump_distance = (dx ** 2 + dy ** 2) ** 0.5

            if jump_distance > self.max_allowed_jump:
                now = time.time()
                # 윈도우 밖으로 벗어난 오래된 기록은 버린다
                self.amcl_jump_timestamps = [
                    t for t in self.amcl_jump_timestamps if now - t <= self.amcl_jump_window_sec
                ]
                self.amcl_jump_timestamps.append(now)

                if len(self.amcl_jump_timestamps) >= self.amcl_jump_count_threshold:
                    print(f"\n[!!! CRITICAL EMERGENCY !!!] AMCL jumped {len(self.amcl_jump_timestamps)} times "
                          f"within {self.amcl_jump_window_sec:.1f}s (latest: {jump_distance:.3f}m). "
                          f"Treating as localization failure.")
                    is_emergency = True
                else:
                    print(f"[*] AMCL correction observed: {jump_distance:.3f}m "
                          f"({len(self.amcl_jump_timestamps)}/{self.amcl_jump_count_threshold} within "
                          f"{self.amcl_jump_window_sec:.1f}s window). Treating as a normal re-localization, "
                          f"not aborting.")

        # 점프 판정 여부와 무관하게 항상 갱신 (고착 버그 수정의 핵심)
        self.last_valid_amcl_pose = (self.current_amcl_x, self.current_amcl_y)

        return is_emergency

    # ------------------------------------------------------------------
    # 직선 sub-segment 실행 (회전 -> 정착 -> 캡처 시작 -> 직선 주행+캡처 -> 정지 -> 캡처 종료)
    # coverage/transit 구분 없이 모든 직선 구간에 동일하게 적용된다.
    # ------------------------------------------------------------------

    def _execute_capture_subsegment(self, seg_poses, record_pcd=True, is_genuine_single=True):
        settle_sec = self.mission_exec_cfg.get('ignore_initial_seconds', 1.0)
        capture_sec_single = self.mission_exec_cfg.get('active_capture_seconds', 2.0)
        end_pose = seg_poses[-1]
        is_single_point = (len(seg_poses) == 1)

        # 1. 제자리 회전
        if is_single_point and not is_genuine_single:
            # 마지막 세그먼트처럼 합칠 대상이
            # 없는 잔여 케이스에 대한 안전망 - "이미 도착했다"고 가정하지 않고
            # 실제로 그 지점까지 주행한다.
            if not self._navigate_to_pose_blocking(seg_poses[0]):
                print("[!] Warning: failed to reach isolated corner point. Proceeding anyway.")
        elif not is_single_point:
            if not self._rotate_in_place_to(end_pose):   # seg_poses[0] -> end_pose
                print("[!] Warning: In-place rotation failed or skipped. Proceeding anyway.")
        # (is_single_point and is_genuine_single인 경우 - 기존 그대로, 회전/이동 없이 캡처만)


        # 2. 캡처 시작 신호
        started = False
        if record_pcd:
            self._spin_sleep(settle_sec)
            started = self._call_capture_service(self.start_capture_client, "start_waypoint_capture")
            if not started:
                print("[!] Skipping this sub-segment's capture window (start signal failed). "
                    "Still performing the drive so the mission continues.")
        else:
            print("  [Capture] record_pcd=False — already covered elsewhere, skipping capture, driving through.")

        # 3. 주행
        ok = True
        if not is_single_point:
            print(f"  [Drive] Straight sub-segment: {len(seg_poses)} points "
                f"({'continuous capture' if record_pcd else 'no capture'} while moving).")
            ok = self._navigate_through_poses_blocking(seg_poses)
        elif started:
            print("  [Drive] Single-point sub-segment: staying in place for capture.")
            self._spin_sleep(capture_sec_single)
        elif record_pcd:
            print("  [Drive] Single-point sub-segment: capture start failed, skipping dwell.")
        else:
            print("  [Drive] Single-point sub-segment: record_pcd=False, skipping dwell entirely.")

        if started:
            self._call_capture_service(self.stop_capture_client, "stop_waypoint_capture")


        return ok

    # ------------------------------------------------------------------
    # 제자리 회전 (nav2_msgs/action/Spin, 절대각 차이를 상대 회전량으로 변환)
    # ------------------------------------------------------------------

    def _quaternion_to_yaw(self, q):
        # 평면 회전만 다루므로 x=y=0을 가정하고 z, w만으로 yaw를 계산한다.
        return 2.0 * math.atan2(q.z, q.w)

    def _get_current_yaw_from_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return self._quaternion_to_yaw(trans.transform.rotation)
        except Exception as e:
            print(f"[!] TF lookup failed for current heading (map->base_link): {e}")
            return None

    def _get_current_pose_from_tf(self):
        """TF(map->base_link)에서 현재 (x, y, yaw)를 함께 읽어온다."""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            yaw = self._quaternion_to_yaw(trans.transform.rotation)
            return x, y, yaw
        except Exception as e:
            print(f"[!] TF lookup failed for current pose (map->base_link): {e}")
            return None, None, None


    def _rotate_in_place_to(self, target_pose):
        """
        현재 위치에서 target_pose(위치)를 향하도록 회전한다.

        target_pose.orientation을 그대로 쓰지 않는다 - 그 값은 "target_pose에
        도착한 뒤 다음 지점을 향해야 할 방향"으로 저장된 값이라(translator.py의
        forward-looking 방식), 아직 target_pose에 도착 전인 지금 그 방향을 미리
        향하면, 실제로는 엉뚱한 방향으로 도는 버그가 있었다(코너 직전 지점에서
        코너를 건너뛰고 그 다음 방향을 미리 보게 됨).

        따라서 "현재 위치 -> target_pose 위치"를 atan2로 직접 계산해서 목표각으로 쓴다.
        """
        current_x, current_y, current_yaw = self._get_current_pose_from_tf()
        if current_yaw is None:
            return False

        dx = target_pose.pose.position.x - current_x
        dy = target_pose.pose.position.y - current_y
        if math.hypot(dx, dy) < 1e-3:
            print("  [Rotate] Target is at current position. Skipping spin.")
            return True

        target_yaw = math.atan2(dy, dx)
        delta_yaw = target_yaw - current_yaw
        delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))

        min_rotation_rad = math.radians(self.mission_exec_cfg.get('min_rotation_deg', 3.0))
        if abs(delta_yaw) < min_rotation_rad:
            print(f"  [Rotate] Already aligned (delta={math.degrees(delta_yaw):.1f}°). Skipping spin.")
            return True

        print(f"  [Rotate] current={math.degrees(current_yaw):.1f}°, "
            f"target={math.degrees(target_yaw):.1f}° (toward next goal), delta={math.degrees(delta_yaw):.1f}°")

        spin_time_allowance = self.mission_exec_cfg.get('spin_time_allowance_sec', 15.0)
        self.navigator.spin(spin_dist=delta_yaw, time_allowance=int(spin_time_allowance))

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.01)
            self.spin_executor.spin_once(timeout_sec=0.0)
            if self._check_amcl_jump():
                self.navigator.cancelTask()
                return False
            time.sleep(0.05)

        result = self.navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            print(f"[!] Warning: Spin action did not succeed (result={result}).")
            return False
        return True

    # ------------------------------------------------------------------
    # 직선 주행 (nav2_msgs/action/NavigateToPose, coverage run의 끝점으로 1회 전송)
    # ------------------------------------------------------------------

    def _navigate_to_pose_blocking(self, pose):
        self.navigator.goToPose(pose)

        last_debug_print_time = time.time()
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.01)
            self.spin_executor.spin_once(timeout_sec=0.0)
            current_time = time.time()

            if self._check_amcl_jump():
                self.navigator.cancelTask()
                return False

            if current_time - last_debug_print_time >= 1.0:
                feedback = self.navigator.getFeedback()
                remaining = getattr(feedback, 'distance_remaining', None) if feedback else None
                if remaining is not None:
                    print(f"  ├─ Driving swath... distance remaining: {remaining:.2f}m")
                last_debug_print_time = current_time

            time.sleep(0.05)

        result = self.navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            print(f"[-] Swath drive ended without SUCCEEDED (result={result}).")
            return False
        return True

    def _navigate_through_poses_blocking(self, seg_poses):
        """
        seg_poses 전체를 NavigateThroughPoses(goThroughPoses)로 한 번에 전달한다.
        goToPose(end_pose)만 보내면 중간 지점(특히 코너 꼭짓점)을 글로벌 플래너가
        반드시 지나가야 할 이유가 없어 코너를 넓게 잘라가며 지나가는 문제가
        있었다. NavigateThroughPoses는 리스트의 모든 (x,y)를 반드시 통과해야
        하는 지점으로 취급하므로 코너 꼭짓점(seg_poses[0])을 실제로 스치듯
        지나가도록 강제할 수 있다.

        주의: 중간 지점들의 orientation은 글로벌 플래너가 강제하지 않는다
        (위치만 통과 지점으로 취급됨). 최종 목표(seg_poses[-1])의 orientation만
        도착 시 정렬 대상이 된다.
        """
        self.navigator.goThroughPoses(seg_poses)

        last_debug_print_time = time.time()
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator, timeout_sec=0.01)
            self.spin_executor.spin_once(timeout_sec=0.0)
            current_time = time.time()

            if self._check_amcl_jump():
                self.navigator.cancelTask()
                return False

            if current_time - last_debug_print_time >= 1.0:
                feedback = self.navigator.getFeedback()
                remaining = getattr(feedback, 'distance_remaining', None) if feedback else None
                n_left = getattr(feedback, 'number_of_poses_remaining', None) if feedback else None
                if remaining is not None:
                    print(f"  ├─ Driving through {len(seg_poses)} points... "
                        f"distance remaining: {remaining:.2f}m, poses left: {n_left}")
                last_debug_print_time = current_time

            time.sleep(0.05)

        result = self.navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            print(f"[-] Through-poses drive ended without SUCCEEDED (result={result}).")
            return False
        return True

    # ------------------------------------------------------------------
    # 캡처 시퀀스 보조 유틸 (settle 대기, surface_profiler 서비스 호출)
    # ------------------------------------------------------------------

    def _spin_sleep(self, duration_sec):
        """AMCL/네트워크 통신이 끊기지 않도록 spin을 유지하면서 duration_sec만큼 대기한다."""
        end_time = time.time() + duration_sec
        while time.time() < end_time:
            self.spin_executor.spin_once(timeout_sec=0.05)
            if self.navigator is not None:
                rclpy.spin_once(self.navigator, timeout_sec=0.01)
            time.sleep(0.02)

    def _call_capture_service(self, client, label):
        """
        surface_profiler.py의 start/stop_waypoint_capture Trigger 서비스를 호출한다.
        best-effort: 서버가 없거나 응답이 없어도 미션 자체를 막지 않고 경고만 남긴다.
        """
        service_name = client.srv_name
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"[!] Warning: Service '{service_name}' not available. "
                  f"Is surface_profiler.py running on the notebook? Skipping {label}.")
            return False

        request = Trigger.Request()
        future = client.call_async(request)

        spin_start = time.time()
        while not future.done() and (time.time() - spin_start) < 5.0:
            self.spin_executor.spin_once(timeout_sec=0.1)

        if future.done() and future.result() is not None:
            response = future.result()
            print(f"[*] {label}: success={response.success}, message='{response.message}'")
            return response.success
        else:
            print(f"[!] Warning: No response from '{service_name}' within timeout.")
            return False

    # ------------------------------------------------------------------
    # surface_profiling(노트북) 측에 PCD 수집 종료 신호 전달
    # ------------------------------------------------------------------

    def _notify_surface_profiling_stop(self, success: bool, message: str = ""):
        """
        노트북에서 구동 중인 SurfaceProfiler에게 수집 종료를 알린다.
        success=True면 정상 종료 서비스, False면 비정상(즉시 강제 종료) 서비스를 호출한다.
        서버가 아직 떠 있지 않거나 응답이 없어도 미션 자체의 종료를 막지는 않는다
        (호출은 best-effort로 처리하고, 결과만 로그로 남긴다).
        """
        client = self.stop_collection_success_client if success else self.stop_collection_abort_client
        service_name = client.srv_name

        if not client.wait_for_service(timeout_sec=3.0):
            print(f"[!] Warning: Service '{service_name}' not available. "
                  f"Is surface_profiler.py running on the notebook? Skipping notification.")
            return

        request = Trigger.Request()
        future = client.call_async(request)

        spin_start = time.time()
        while not future.done() and (time.time() - spin_start) < 5.0:
            self.spin_executor.spin_once(timeout_sec=0.1)

        if future.done() and future.result() is not None:
            response = future.result()
            print(f"[*] Notified '{service_name}': success={response.success}, message='{response.message}'")
        else:
            print(f"[!] Warning: No response from '{service_name}' within timeout.")

    # ------------------------------------------------------------------
    # 결과 저장
    # ------------------------------------------------------------------

    def _save_mission_results(self):
        # FollowWaypoints는 coverage 지점마다 별도의 goal로 나뉘어 순차 전송되므로,
        # navigator.getResult()는 "마지막으로 보낸 세그먼트" 하나의 결과만 반영한다.
        # 미션 전체의 성공/실패는 execute_mission()에서 추적한 self.mission_succeeded를
        # 우선 참조하고, result는 로그 참고용으로만 사용한다.
        result = self.navigator.getResult()
        overall_success = self.mission_succeeded

        if overall_success:
            print("[+] Mission Successfully Completed!")
            self._notify_surface_profiling_stop(success=True, message="Mission completed successfully.")

            # CSV 데이터 저장
            output_path_dir = os.path.join(self.workspace_root, self.mission_exec_cfg.get('output_path_dir', 'analytics/paths'))
            os.makedirs(output_path_dir, exist_ok=True)
            csv_filename = os.path.join(output_path_dir, f"robot_path_{int(time.time())}.csv")

            try:
                with open(csv_filename, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["timestamp", "x", "y"])
                    writer.writerows(self.path_history)
                print(f"[+] Successfully saved robot path history to '{csv_filename}'.")

                # 결과 시각화(PNG) 저장
                vis_dir = os.path.join(self.workspace_root, self.mission_exec_cfg.get('visualization_dir', 'visualization/mission_execution'))
                os.makedirs(vis_dir, exist_ok=True)
                img_out_path = os.path.join(vis_dir, f"robot_path_{int(time.time())}_plot.png")

                visualize_paths(csv_filename, self.final_path, img_out_path)

            except Exception as e:
                print(f"[-] Failed to save outputs due to error: {e}")
                traceback.print_exc()

        elif result == TaskResult.CANCELED:
            print(f"\n[!] Mission was canceled! (last segment result={result})")
            self._notify_surface_profiling_stop(success=False, message="Mission was canceled.")
        else:
            print(f"\n[-] Mission failed! (last segment result={result})")
            self._notify_surface_profiling_stop(success=False, message="Mission failed.")

        # ROS 2 자원 안전 셧다운 (데드락 방지: subscription들은 execute_mission/_initialize_localization
        # 단계에서 이미 정리되었으므로 여기서는 executor/노드/컨텍스트 종료만 수행)
        self.spin_executor.remove_node(self)
        self.destroy_node()
        if self.navigator is not None:
            self.navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 외부 실행 엔트리포인트
    # ------------------------------------------------------------------

    def run(self):
        self._setup_ros_environment()
        self._initialize_localization()
        self.execute_mission()
        self._save_mission_results()


def main(args=None):
    if args is None:
        args = sys.argv

    if not rclpy.ok():
        rclpy.init(args=args)

    mission_executor = MissionExecutor()
    mission_executor.run()


if __name__ == "__main__":
    main(args=sys.argv)