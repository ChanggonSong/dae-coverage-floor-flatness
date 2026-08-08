# mission_generation/mission_planning/mission_planner.py

import json
import numpy as np
import cv2
import os
import math
import time

from mission_planning.algorithms import tsp, coverage, transit
from mission_planning.utils import visualizer, geometry, sampler
from mission_planning import translator

class MissionPlanner:
    # 파라미터 업데이트
    def __init__(self, topomap_path, visualization_dir="./debug", robot_width=0.28, path_safety_margin=0.25, lidar_range=8.4, overlap=0.2, turn_weight=2.0, wall_weight=5.0, lidar_mount_height=0.338, lidar_vertical_fov_deg=15.0,
             blind_radius_m=None, **kwargs):
        if not os.path.exists(topomap_path):
            raise FileNotFoundError(f"[!] Topomap file not found at: {topomap_path}")
            
        data = np.load(topomap_path, allow_pickle=True)
        
        masks = data['nodes']
        nondriveable_masks = data.get('nondriveable_nodes', [])
        
        self.map_resolution = float(data.get('resolution', 0.05)) 
        self.origin = data.get('origin', [0, 0])
        
        self.robot_width = robot_width
        self.path_safety_margin = path_safety_margin
        self.visualization_dir = os.path.abspath(visualization_dir)

        self.turn_weight = float(turn_weight)
        self.wall_weight = float(wall_weight)

        default_r = lidar_mount_height / math.tan(math.radians(lidar_vertical_fov_deg))
        self.blind_radius_m = blind_radius_m if blind_radius_m is not None else max(default_r, 1.0)
        self.blind_radius_px = self.blind_radius_m / self.map_resolution

        print(f"[*] map_resolution={self.map_resolution}, blind_radius_m={self.blind_radius_m:.3f}, blind_radius_px={self.blind_radius_px:.1f}")

        self.nodes = []
        for i in range(len(masks)):
            node_dict = {
                'id': i + 1,
                'driveable_mask': masks[i],
                'nondriveable_mask': nondriveable_masks[i] if i < len(nondriveable_masks) else None
            }
            self.nodes.append(node_dict)

        if not self.nodes:
            print("[ERROR] No nodes found in topomap file.")
            return

        # 전체 구동 가능 영역 병합 (A* 등에서 활용)
        self.global_mask = np.zeros_like(self.nodes[0]['driveable_mask'])
        for node in self.nodes:
            if node['driveable_mask'] is not None:
                self.global_mask = cv2.bitwise_or(self.global_mask, node['driveable_mask'])
        
        # 알고리즘 모듈들에 넘겨줄 로봇 파라미터 캡슐화
        self.robot_params = {
            'width_px': robot_width / self.map_resolution,
            'effective_swath_m': lidar_range * (1.0 - overlap),
            'swath_width_px': (lidar_range * (1.0 - overlap)) / self.map_resolution
        }
        
        # 결과 저장 리스트
        self.path_segments = []

        print(f"[*] MissionPlanner Initialized")

        if kwargs:
            print(f"[WARN] MissionPlanner received unexpected kwargs (ignored): {list(kwargs.keys())}")

    def generate_cost_map(self, mask):
        dist_px = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
        dist_m = dist_px * self.map_resolution
        
        cost_map = np.zeros_like(mask, dtype=np.uint8)
        
        # 1. 완벽한 안전 영역 (벽으로부터 로봇반경+마진 이상 떨어짐): 255
        safe_threshold = self.robot_width + self.path_safety_margin
        cost_map[dist_m >= safe_threshold] = 255
        
        # 2. 소프트 페널티 영역 (벽과 가깝지만 통과는 가능함, ex: 문지방): 50 ~ 250 그라데이션
        penalty_mask = (dist_m >= self.robot_width) & (dist_m < safe_threshold)
        if np.any(penalty_mask):
            normalized_dist = (dist_m[penalty_mask] - self.robot_width) / self.path_safety_margin
            cost_map[penalty_mask] = (50 + normalized_dist * 200).astype(np.uint8)
            
        # 3. 절대 불가 영역 (물리적 로봇 반경 이내): 0 (기본값이 0이므로 별도 대입 생략)
        return cost_map

    def generate_coverage_mask(self, mask):
        dist_px = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
        dist_m = dist_px * self.map_resolution
        
        coverage_mask = np.zeros_like(mask, dtype=np.uint8)
        safe_threshold = self.robot_width + self.path_safety_margin
        
        coverage_mask[dist_m >= safe_threshold] = 255
        
        # 폴백 방어 로직: 맵이 너무 좁아서 마진 적용 시 영역이 아예 사라지면 로봇 반경까지만 깎음
        if cv2.countNonZero(coverage_mask) == 0:
            coverage_mask[dist_m >= self.robot_width] = 255
            
        return coverage_mask

    def _orient_raw_points(self, raw_points, entry_hint, exit_hint, fallback_current_pos):
        """
        coverage 경로(raw_points, F2C 스와스들을 이어붙인 좌표 목록)의 전체 진행
        방향(정방향 vs 전체 역순)을 결정한다.

        TSP 방문 순서상 이 노드의 '진입 방향'(entry_hint: 이전 노드와의 연결
        waypoint)과 '진출 방향'(exit_hint: 다음 노드와의 연결 waypoint)을 모두
        고려해서, (entry_hint와 시작점 거리) + (exit_hint와 끝점 거리)의 합이
        최소가 되는 쪽을 선택한다. 예전 로직은 진입 방향(current_pos 기준
        근접성)만 봤는데, 그러면 진출 지점이 다음 노드와 정반대 방향에 남아서
        불필요한 우회 transit이 생길 수 있었다.

        F2C가 만든 스와스 내부의 지그재그 연결 순서는 그대로 유지하고, 리스트
        전체를 뒤집을지 말지만 결정한다 - 개별 스와스를 재배치하는 건 F2C가
        이미 최적화해둔 커버리지 효율(이동거리 최소화)을 해칠 수 있어 다루지
        않는다.

        Args:
            raw_points: [(px, py), ...] F2C 스와스 좌표 목록
            entry_hint: 이전 노드와의 연결 지점 (px, py) 또는 None (미션의 첫 노드 등)
            exit_hint: 다음 노드와의 연결 지점 (px, py) 또는 None (미션의 마지막 노드 등)
            fallback_current_pos: entry_hint가 없을 때 대신 사용할 로봇의 실제 마지막 위치
                (하위 호환. entry_hint가 있으면 그쪽을 우선한다 - 그래프 상의 연결
                지점이 실제 로봇 위치보다 더 원칙적인 기준이기 때문)

        Returns:
            list: 방향이 결정된 raw_points (원본 또는 전체 역순)
        """
        if len(raw_points) <= 1:
            return raw_points

        start_pt = raw_points[0]
        end_pt = raw_points[-1]
        entry_ref = entry_hint if entry_hint is not None else fallback_current_pos

        if entry_ref is None and exit_hint is None:
            # 아무 기준도 없으면(예: 정말 첫 노드) F2C 기본 순서 그대로 사용
            return raw_points

        def _dist(a, b):
            if a is None or b is None:
                return 0.0
            return float(np.hypot(a[0] - b[0], a[1] - b[1]))

        cost_forward = _dist(entry_ref, start_pt) + _dist(exit_hint, end_pt)
        cost_reversed = _dist(entry_ref, end_pt) + _dist(exit_hint, start_pt)

        if cost_reversed < cost_forward:
            return list(reversed(raw_points))
        return raw_points
    

    def execute_full_mission(self):
        """
        하위 알고리즘 모듈들 오케스트레이션해서 전체 로봇 mission plan
        """
        print(f"\n{'-'*14} [Full Mission Planning Start] {'-'*14}")
        start_time = time.time()
        
        # [Step 1] TSP 순서 및 상세 경유 시퀀스 계산
        print("[Step 1/3] Calculating TSP and transit sequence...")
        tsp_sequence, detailed_sequence, planning_mask, node_waypoints, connection_widths_px, connection_masks = tsp.solve_tsp_sequence(
            nodes=self.nodes,
            global_mask=self.global_mask
        )
        
        # Transit A* 전용 글로벌 비용 지도(Cost Map) 생성
        print("[*] Generating Safe Cost Map for Transit Paths...")
        global_cost_map = self.generate_cost_map(self.global_mask)
        
        # [Step 2] 각 타겟 노드별 F2C 측정(Coverage) 경로 계산
        print("[Step 2/3] Generating F2C coverage paths...")
        mission_plan = {}
        width_bucket_counts = {'wide': 0, 'narrow': 0, 'ultra_narrow': 0}
        width_bucket_log = []  # (node_id, safe_width_px, bucket) - 필요 시 CSV로 dump 가능

        for node_idx in range(len(self.nodes)):
            # 원본 도면이 아닌 마진이 확보된 Coverage 전용 마스크 전달
            safe_node_mask = self.generate_coverage_mask(self.nodes[node_idx]['driveable_mask'])
            safe_width_px = geometry.estimate_min_width_px(safe_node_mask)

            node_id = self.nodes[node_idx]['id']
            contours, _ = cv2.findContours(safe_node_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if node_id == 2:
                print(f"[DEBUG] Node {node_id}의 safe_node_mask 윤곽선 개수: {len(contours)}")
                if len(contours) > 1:
                    areas = [cv2.contourArea(c) for c in contours]
                    print(f"[DEBUG] 각 조각의 면적: {areas}")

            if contours:
                c = max(contours, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
                box_pts = cv2.boxPoints(rect).astype(int)

                debug_img = cv2.cvtColor(self.nodes[node_idx]['driveable_mask'], cv2.COLOR_GRAY2BGR)
                cv2.drawContours(debug_img, [box_pts], 0, (0, 0, 255), 2)
                cv2.drawContours(debug_img, [c], -1, (0, 255, 0), 1)
                cv2.putText(debug_img, f"node {node_id}: {safe_width_px:.1f}px", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                width_debug_dir = os.path.join(self.visualization_dir, "width_debug")
                os.makedirs(width_debug_dir, exist_ok=True)
                cv2.imwrite(os.path.join(width_debug_dir, f"node_{node_id:03d}.png"), debug_img)

            assist_needed = safe_width_px < self.blind_radius_px
            bucket = ('ultra_narrow' if assist_needed
                    else 'narrow' if safe_width_px < 2 * self.blind_radius_px
                    else 'wide')
            width_bucket_counts[bucket] += 1
            width_bucket_log.append((node_id, round(safe_width_px, 1), bucket))

            self.nodes[node_idx]['assist_needed'] = assist_needed
            self.nodes[node_idx]['bucket'] = bucket
            self.nodes[node_idx]['safe_node_mask'] = safe_node_mask

        # 너비 측정 로그
        print(f"\n[*] Node width classification (blind_radius_px={self.blind_radius_px:.1f}):")
        print(f"    wide={width_bucket_counts['wide']}, narrow={width_bucket_counts['narrow']}, "
            f"ultra_narrow={width_bucket_counts['ultra_narrow']}  (total={len(width_bucket_log)})")
        for node_id, w, bucket in sorted(width_bucket_log, key=lambda t: t[1]):
            print(f"      node {node_id:>3}: safe_width_px={w:>6}  -> {bucket}")

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            from matplotlib.patches import Patch

            os.makedirs(self.visualization_dir, exist_ok=True)

            sorted_log = sorted(width_bucket_log, key=lambda t: t[1])
            node_labels = [f"node {nid}" for nid, _, _ in sorted_log]
            widths = [w for _, w, _ in sorted_log]
            buckets = [b for _, _, b in sorted_log]

            bucket_colors = {'wide': '#2ca02c', 'narrow': '#ff9900', 'ultra_narrow': '#d62728'}
            bar_colors = [bucket_colors[b] for b in buckets]

            fig, ax = plt.subplots(figsize=(max(8, len(sorted_log) * 0.9), 6))
            bars = ax.bar(range(len(sorted_log)), widths, color=bar_colors, edgecolor='black')

            for bar, w in zip(bars, widths):
                ax.text(bar.get_x() + bar.get_width() / 2, w + max(widths) * 0.015,
                        f"{w:.1f}", ha='center', va='bottom', fontsize=9)

            ax.set_xticks(range(len(sorted_log)))
            ax.set_xticklabels(node_labels, rotation=45, ha='right')
            ax.axhline(self.blind_radius_px, color='orange', linestyle='--',
                    label=f'blind_radius_px ({self.blind_radius_px:.0f})')
            ax.axhline(2 * self.blind_radius_px, color='red', linestyle='--',
                    label=f'2x blind_radius_px ({2 * self.blind_radius_px:.0f})')
            ax.set_ylabel('safe_width_px')
            ax.set_title('Node width classification (per-node)')

            bucket_handles = [Patch(facecolor=bucket_colors[b], edgecolor='black', label=b)
                            for b in ['wide', 'narrow', 'ultra_narrow']]
            line_handles, line_labels = ax.get_legend_handles_labels()
            ax.legend(handles=bucket_handles + line_handles, loc='upper left')

            summary_line = (f"wide={width_bucket_counts['wide']}, narrow={width_bucket_counts['narrow']}, "
                            f"ultra_narrow={width_bucket_counts['ultra_narrow']}  (total={len(width_bucket_log)})")
            fig.suptitle(summary_line, fontsize=10, y=0.98)

            detail_lines = [f"node {nid:>3}: safe_width_px={w:>6} -> {b}" for nid, w, b in sorted_log]
            fig.text(0.02, -0.02, "\n".join(detail_lines), fontsize=7, family='monospace', va='top')

            plt.tight_layout(rect=[0, 0.02, 1, 0.95])
            hist_path = os.path.join(self.visualization_dir, 'width_classification_histogram.png')
            plt.savefig(hist_path, dpi=150, bbox_inches='tight')
            plt.close()
            print(f"[*] Histogram saved to: {hist_path}")
        except ImportError:
            print("[WARN] matplotlib not available - counts above are still printed.")

        self.assist_mask = np.zeros_like(self.global_mask)
        for node in self.nodes:
            if node.get('assist_needed'):
                self.assist_mask = cv2.bitwise_or(self.assist_mask, node['driveable_mask'])

        # 노드 자체는 넓어도 두 노드가 만나는 연결부(문지방 등)가 좁으면
        # 그 연결부만 별도로 assist_mask에 추가한다. connection_masks는 (i,j)/(j,i)
        # 양방향에 같은 배열을 담고 있으므로 canonical (min,max) 키로 한 번씩만 처리.
        narrow_connections = []
        seen_pairs = set()
        for (i, j), width_px in connection_widths_px.items():
            pair = (min(i, j), max(i, j))
            if pair in seen_pairs:
                continue
            seen_pairs.add(pair)

            if width_px < self.blind_radius_px:
                self.assist_mask = cv2.bitwise_or(self.assist_mask, connection_masks[pair])
                narrow_connections.append((self.nodes[pair[0]]['id'], self.nodes[pair[1]]['id'], round(width_px, 1)))

        if narrow_connections:
            print(f"\n[*] Narrow connections needing supplementary transit capture "
                f"(threshold={self.blind_radius_px:.1f}px):")
            for a, b, w in narrow_connections:
                print(f"      node {a} <-> node {b}: local_width_px={w}")
        else:
            print(f"\n[*] No narrow connections detected (threshold={self.blind_radius_px:.1f}px).")
        
        # [Step 3] 최종 궤적 생성 및 연결 (Transit via A*)
        print("[Step 3/3] Finalizing trajectory (Linking all paths)...")
        self.path_segments = [] 
        current_pos = None
        tsp_idx = 0 
        
        coverage_count = 0
        transit_count = 0

        def _split_by_assist_mask(path, assist_mask):
            if not path:
                return []
            runs, cur_flag, cur_run = [], None, []
            for (x, y) in path:
                flag = bool(assist_mask[y, x]) if 0 <= y < assist_mask.shape[0] and 0 <= x < assist_mask.shape[1] else False
                if flag != cur_flag and cur_run:
                    runs.append((cur_flag, cur_run))
                    cur_run = [cur_run[-1]]  # 이어붙임 (끊김 방지)
                cur_run.append((x, y))
                cur_flag = flag
            if cur_run:
                runs.append((cur_flag, cur_run))
            return runs

        def simplify_path(path, epsilon_px=3.0):
            """
            A* 8방향 격자 이동이 만드는 계단식(staircase) 지그재그를 제거하고 진짜
            꺾이는 지점만 남긴다. 격자는 임의 각도의 직선을 정확히 못 그리고 두
            방향을 번갈아 밟아 근사하는데, 이 계단 하나하나를 sampler.py의 1도
            앵커 감지 로직이 '진짜 코너'로 착각하는 문제를 막기 위함이다.
            """
            if len(path) < 3:
                return path
            arr = np.array(path, dtype=np.int32).reshape((-1, 1, 2))
            simplified = cv2.approxPolyDP(arr, epsilon_px, closed=False)
            return [tuple(map(int, pt[0])) for pt in simplified]

        for i in range(len(detailed_sequence)):
            curr_node = detailed_sequence[i]
            
            # 1. 측정(Coverage) 대상 노드 처리
            if tsp_idx < len(tsp_sequence) and curr_node == tsp_sequence[tsp_idx]:
                prev_node = detailed_sequence[i - 1] if i > 0 else None
                entry_hint = node_waypoints.get((prev_node, curr_node)) if prev_node is not None else None

                bucket = self.nodes[curr_node]['bucket']
                safe_node_mask = self.nodes[curr_node]['safe_node_mask']

                if bucket in ('narrow', 'ultra_narrow'):
                    forced_angle = geometry.get_long_axis_angle_rad(safe_node_mask)
                    swath_pairs = coverage.generate_raw_swaths(safe_node_mask, self.robot_params, decompose=True, split_angle_rad=forced_angle)
                else:
                    swath_pairs = coverage.generate_raw_swaths(safe_node_mask, self.robot_params)

                if self.nodes[curr_node]['id'] == 2:
                    print(f"[DEBUG] Node 2: forced_angle_deg={math.degrees(forced_angle):.1f}, "
                          f"num_swaths={len(swath_pairs)}, swath_pairs={swath_pairs}")

                    debug_img = cv2.cvtColor(safe_node_mask, cv2.COLOR_GRAY2BGR)
                    contours, _ = cv2.findContours(safe_node_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(debug_img, contours, -1, (0, 255, 0), 1)  # 초록: 폴리곤 전체
                    for p1, p2 in swath_pairs:
                        cv2.line(debug_img, p1, p2, (0, 0, 255), 2)   # 빨강: 실제 생성된 스와스
                        cv2.circle(debug_img, p1, 4, (255, 0, 0), -1)  # 파랑: 스와스 시작점
                        cv2.circle(debug_img, p2, 4, (0, 255, 255), -1)  # 노랑: 스와스 끝점

                    dbg_dir = os.path.join(self.visualization_dir, "width_debug")
                    os.makedirs(dbg_dir, exist_ok=True)
                    cv2.imwrite(os.path.join(dbg_dir, "node_002_swaths.png"), debug_img)
                    print(f"[DEBUG] Saved node_002_swaths.png")

                raw_points = []
                if swath_pairs:
                    ordered_pairs = geometry.order_swaths_by_entry(swath_pairs, entry_hint)
                    for p1, p2 in ordered_pairs:
                        raw_points.extend([p1, p2])
                else:
                    centroid = geometry.get_centroid(self.nodes[curr_node]['driveable_mask'])
                    if centroid: raw_points.append(centroid)
                
                if raw_points:
                    # 진입/진출 힌트 확보: detailed_sequence 상에서 이 노드의 바로 앞/뒤
                    # 노드와의 연결 지점(waypoints, tsp.py의 extract_waypoints가 계산한
                    # '문지방 등 안전 통과 지점'). 두 노드가 항상 그래프 상 인접하도록
                    # detailed_sequence가 Dijkstra 최단경로로 구성되어 있으므로, 이
                    # 조회는 항상 유효한 값을 반환한다(첫/마지막 노드의 바깥쪽 방향 제외).

                    # 방향 최적화: 진입점 근접성뿐 아니라 진출점(다음 노드로 가는 방향)까지
                    # 함께 고려해서 정방향/역방향을 선택한다. 이렇게 해야 예를 들어 방 A ->
                    # 복도 B -> 복도 C로 이동할 때, B의 coverage가 A쪽에서 들어와서 C쪽으로
                    # 나가도록 자연스럽게 정렬되어 불필요한 되돌아가기(우회 transit)가 줄어든다.
                    
                    # 노드 진입 경로 (Transit) 계산 (A* 알고리즘)
                    if current_pos:
                        via_point = entry_hint  # curr_node로 들어가는 연결부의 로컬 중심점
                        enter_path = []

                        if via_point is not None and via_point != current_pos:
                            print(f"[TRACE_ENTER] Leg 1 (via doorway center): {current_pos} -> {via_point}")
                            _, leg1 = transit.find_path_with_penalty(
                                start=current_pos, goal=via_point, planning_mask=global_cost_map,
                                turn_weight=self.turn_weight, wall_weight=self.wall_weight
                            )
                            if leg1:
                                enter_path.extend(leg1)
                            else:
                                print(f"[WARN] Leg1 (doorway center 경유) 실패 - 직접 경로로 폴백.")

                        leg2_start = enter_path[-1] if enter_path else current_pos
                        print(f"[TRACE_ENTER] Leg 2: {leg2_start} -> {raw_points[0]}")
                        _, leg2 = transit.find_path_with_penalty(
                            start=leg2_start, goal=raw_points[0], planning_mask=global_cost_map,
                            turn_weight=self.turn_weight, wall_weight=self.wall_weight
                        )

                        if leg2:
                            if enter_path and enter_path[-1] == leg2[0]:
                                full_enter_path = enter_path + leg2[1:]
                            else:
                                full_enter_path = enter_path + leg2

                            for flag, run in _split_by_assist_mask(full_enter_path, self.assist_mask):
                                self.path_segments.append({'type': 'transit', 'path': run, 'record_pcd': flag})
                            transit_count += 1
                        else:
                            print(f"[ERROR] Cannot find safe path to Node {curr_node+1}. Wall detected!")
                    
                    # 측정 경로 추가
                    self.path_segments.append({'type': 'coverage', 'path': raw_points, 'record_pcd': True})
                    coverage_count += 1
                    current_pos = raw_points[-1]
                
                print(f"    -> Completed Coverage task in Node {curr_node+1}")
                tsp_idx += 1 
            else:
                print(f"    -> Transiting through Node {curr_node+1}")
            
        total_time = time.time() - start_time
        print(f"\n[DEBUG] Path Segments Created: Coverage({coverage_count}), Transit({transit_count})")
        print(f"{'-'*20} [Planning Completed in {total_time:.2f}s] {'-'*20}\n")
        
        return tsp_sequence, mission_plan
    
    def plan(self, save_debug=True, show_plot=False, output_dir=None):
        # output_dir 미지정 시, 현재 작업 디렉토리(cwd)에 의존하는 상대경로
        # "analytics/metrics" 대신 외부 저장소(workspace_root) 기준 절대경로로 fallback.
        if output_dir is None:
            default_workspace_root = os.path.expanduser("~/dae_floor_maps")
            output_dir = os.path.join(default_workspace_root, "analytics/metrics")
            print(f"[WARN] 'output_dir' not provided to plan(). Falling back to: {output_dir}")

        # 1. 전역 미션 계획 실행 (Pixel 단위 경로 생성)
        self.execute_full_mission()

        # 2. 경로 생성 실패 시 예외 처리
        if not self.path_segments:
            print("[WARN] No path generated. Mission aborted.")
            return None
        
        # 3. 결과 시각화 (Visualizer)
        if save_debug:
            print(f"[*] Saving debug visualization to centralized storage...")
            os.makedirs(self.visualization_dir, exist_ok=True)
            visualizer.save_debug_image(
                nodes=self.nodes,
                path_segments=self.path_segments,
                global_mask=self.global_mask,
                output_dir=self.visualization_dir,
                filename="full_mission_path.png"
            ) # full_mission_path.png 저장
            
        if show_plot:
            print("[*] Displaying mission state on screen.")
            visualizer.plot_mission_state(
                nodes=self.nodes,
                path_segments=self.path_segments,
                global_mask=self.global_mask
            )

        # 4. Translator를 통한 좌표 변환 및 메시지 포맷팅 (Pixel -> Meter)
        # Y축 대칭 반전 역산을 위해 전역 마스크 이미지의 세로 픽셀 크기(Height)를 추출함.
        map_height = self.global_mask.shape[0]

        print("[*] Translating path segments to Metric coordinates...")

        raw_nav2_path = translator.convert_segments_to_nav2(
            path_segments=self.path_segments,
            origin=self.origin,
            resolution=self.map_resolution,
            map_height=map_height
        )

        # sampling_step만큼의 거리마다 샘플링
        sampled_nav2_path = sampler.interpolate_with_semantics(
            raw_nav2_path
        )
        
        raw_flat_path = []
        for seg in raw_nav2_path:
            for p in seg['poses']:
                p_copy = json.loads(json.dumps(p))
                p_copy['header'] = {
                    'frame_id': 'map',
                    'task_type': seg['type'],
                    'record_pcd': seg.get('record_pcd', seg['type'] == 'coverage'),
                }
                x, y = p_copy['pose']['position']['x'], p_copy['pose']['position']['y']
                
                # 거리 기반 비교
                if not raw_flat_path or math.hypot(raw_flat_path[-1]['pose']['position']['x'] - x,
                                                raw_flat_path[-1]['pose']['position']['y'] - y) > 0.001:
                    raw_flat_path.append(p_copy)

        os.makedirs(output_dir, exist_ok=True)
        raw_output_file = os.path.join(output_dir, "raw_path.json")
        sampled_output_file = os.path.join(output_dir, "final_path.json")

        with open(raw_output_file, 'w') as f:
            json.dump(raw_flat_path, f, indent=4)
            
        with open(sampled_output_file, 'w') as f:
            json.dump(sampled_nav2_path, f, indent=4)

        print(f"[*] Mission Planner Successfully Completed.")
        print(f"    -> Raw Keypoints Path saved to: {raw_output_file} ({len(raw_flat_path)} pts)")
        print(f"    -> Sampled Path saved to: {sampled_output_file} ({len(sampled_nav2_path)} pts)")

        if save_debug:
            print("[*] Drawing path points on debug images...")
            
            # 1. 픽셀 좌표 변환 함수
            def get_pixel_points(pose_list_or_segments, is_raw=False):
                pts = []
                # 원본(raw)인 경우 중첩 리스트 구조, sampled된 경로인 경우 포인트들의 단일 리스트임.
                poses = []
                if is_raw:
                    for seg in pose_list_or_segments:
                        poses.extend(seg['poses'])
                else:
                    poses = pose_list_or_segments

                for p in poses:
                    mx = p['pose']['position']['x']
                    my = p['pose']['position']['y']
                    px = int((mx - self.origin[0]) / self.map_resolution)
                    py = int(map_height - (my - self.origin[1]) / self.map_resolution)
                    if 0 <= px < self.global_mask.shape[1] and 0 <= py < self.global_mask.shape[0]:
                        pts.append((px, py))
                return pts

            # 좌표 추출
            raw_pixel_points = get_pixel_points(raw_nav2_path, is_raw=True)
            sampled_pixel_points = get_pixel_points(sampled_nav2_path, is_raw=False)

            # 오버레이할 베이스 이미지 경로
            base_img_path = os.path.join(self.visualization_dir, "full_mission_path.png")

            # 2. 이미지 로드 및 오버레이
            def create_overlay_image(output_path, points):
                if os.path.exists(base_img_path):
                    img = cv2.imread(base_img_path)
                    if img is not None:
                        img = visualizer.draw_waypoint_on_image(img, points)
                        cv2.imwrite(output_path, img)
                        print(f"[*] Overlay saved to: {output_path}")
                    else:
                        print(f"[!] Failed to load base image: {base_img_path}")
                else:
                    print(f"[!] Base image not found: {base_img_path}")

            create_overlay_image(os.path.join(self.visualization_dir, "raw_waypoint.png"), raw_pixel_points)
            create_overlay_image(os.path.join(self.visualization_dir, "sampled_waypoint.png"), sampled_pixel_points)
            
        # 샘플링된 웨이포인트 반환. 필요하다면 원본(raw) 포인트를 반환해도 됨.
        return sampled_nav2_path
