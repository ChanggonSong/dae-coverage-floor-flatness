# mission_generation/run_generation_pipeline.py

import os
import yaml
import json
import traceback

from ament_index_python.packages import get_package_share_directory

# 코어 모듈 - 맵 분할과 경로 생성
from mission_generation.environment_modeling.environment_modeler import EnvironmentModeler
from mission_generation.mission_planning.mission_planner import MissionPlanner

def run_generation_pipeline():
    print("\n=======================================================")
    print("[*] Mission Generator (Workstation)")
    print("    : Environment Modeling and Mission Planning")
    print("=======================================================\n")

    try:
        package_share_dir = get_package_share_directory('dae_coverage_floor_flatness')
        config_path = os.path.join(package_share_dir, 'config', 'params.yaml')
    except Exception:
        # 현재 파일(__file__)의 부모 디렉터리(..)로 이동 후 config/params.yaml 추적
        config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "params.yaml"))
    
    # 1. Load Global Config
    print(f"[*] Resolving parameters from: {config_path}")
    if not os.path.exists(config_path):
        print(f"[!] Critical Error: Global Configuration file not found at {config_path}")
        return

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # 2. params.yaml 내부 'global'
    global_cfg = config.get('global', {})
    workspace_root = os.path.expanduser(global_cfg.get('workspace_root', '~/dae_floor_maps'))

    # 3. 'environment_modeling'
    env_cfg = config.get('environment_modeling', {})
    topology_dir = os.path.join(workspace_root, env_cfg.get('output_topology_dir', 'maps/topology'))
    grid_dir = os.path.join(workspace_root, env_cfg.get('output_grid_dir', 'maps/grid'))

    map_file = os.path.normpath(os.path.join(topology_dir, "final_topological_map.npz"))
    yaml_path = os.path.normpath(os.path.join(grid_dir, "map_from_dae.yaml"))

    # 'mission_planner'의 결과인 웨이포인트 경로 지정
    mission_cfg = config.get('mission_planner', {})
    sampling_step = mission_cfg.pop('sampling_step', 0.5)
    metric_dir = os.path.join(workspace_root, mission_cfg.pop('output_metric_dir', 'analytics/metrics'))
    cache_file = os.path.normpath(os.path.join(metric_dir, "final_path.json"))

    # 2. Map Processing
    need_process = False

    if not os.path.exists(map_file):
        print(f"[*] Preprocessed topological asset missing at: {map_file}")
        need_process = True
    else:
        ans = input(f"[*] Topological map asset exists. Re-process 3D Map to 2D? (y/n): ")
        need_process = ans.lower() in ['y', 'yes']

    if need_process:
        print("[*] Instantiating EnvironmentModeling...")
        try:
            env_modeler = EnvironmentModeler(config)
            if hasattr(env_modeler, 'build_environment_model'):
                success = env_modeler.build_environment_model()
            else:
                success = env_modeler.run_all()
                
            if not success:
                print("[!] Map preprocessing returned failure. Aborting mission planning.")
                return
        except Exception as e:
            print(f"[!] Map Pre-processing System Crash: {e}")
            traceback.print_exc()
            return

    # 3. Mission Planning (global waypoint) 
    raw_path_file = os.path.join(metric_dir, "raw_path.json") # 샘플링 이전의 웨이포인트들

    regenerate = True
    if os.path.exists(cache_file) and os.path.exists(raw_path_file) and not need_process:
        ans = input(f"[*] Target waypoint registry files exist. Re-generate Path? (y/n): ")
        regenerate = ans.lower() in ['y', 'yes']

    if regenerate:
        print("[*] Launching MissionPlanner Engine...")
        try:
            # plan() 메서드용 인자와 시각화 경로 분리
            planner_vis_rel = mission_cfg.pop('visualization_dir', 'visualization/mission_generation/mission_planning')
            planner_vis_path = os.path.join(workspace_root, planner_vis_rel)
            
            robot_width = env_cfg.get('robot_width', 0.28)
            path_safety_margin = env_cfg.get('path_safety_margin', 0.25)
            lidar_mount_height = env_cfg.get('lidar_mount_height', 0.338)
            lidar_vertical_fov_deg = env_cfg.get('lidar_vertical_fov_deg', 15.0)

            
            planner = MissionPlanner(
                topomap_path=map_file,
                visualization_dir=planner_vis_path,
                robot_width=robot_width,
                path_safety_margin=path_safety_margin,
                lidar_mount_height=lidar_mount_height,
                lidar_vertical_fov_deg=lidar_vertical_fov_deg,
                **mission_cfg
            )
            
            final_path = planner.plan(
                save_debug=True,
                show_plot=False,
                output_dir=metric_dir,
            )
            
            if final_path:
                print(f"[+] Success! New full coverage path (sampled) saved to: {cache_file}")
                print(f"    Total Generated Waypoints: {len(final_path)}")
                print(f"\n[!] Mission execution assets generated. Please sync the workspace root '{workspace_root}' to Jetson Orin Nano.")
            else:
                print("[!] Mission Planner returned empty route. Path generation aborted.")
                return
        except Exception as e:
            print(f"[!] Mission Planning System Crash: {e}")
            traceback.print_exc()
            return
    else:
        print("[*] Safe Mode: Reusing existing final_path.json registry. Skip optimization.")

if __name__ == "__main__":
    run_generation_pipeline()
