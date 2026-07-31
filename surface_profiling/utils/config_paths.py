# surface_profiling/utils/config_paths.py
"""
params.yaml의 상대 경로/폴더명 값들을 실제 파일 시스템 절대 경로로 조합하는
로직을 한 곳에 모아둔다.

배경: 예전에는 이 조합 로직이 surface_profiler.py의 _resolve_directories()와
reprocess_pcd.py의 main() 두 곳에 각각 따로 구현되어 있었다. 그런데 두 구현이
서로 달라서(한쪽은 workspace_root + 폴더 + 파일명을 다 조합, 다른 쪽은 조합된
값을 다시 폴더명만 있는 원본 값으로 덮어씀) map_yaml_dir가 "maps/grid"라는
미완성 값인 채로 generate_floor_heatmap에 전달되는 버그가 있었다.

앞으로 경로 조합이 필요한 곳은 전부 이 모듈의 함수를 통해서만 하도록 하여,
같은 종류의 불일치가 재발하지 않도록 한다.
"""

import os


def load_config(config_path=None):
    """params.yaml을 읽어 (workspace_root, profiling_cfg) 튜플을 반환한다.

    config_path를 명시하지 않으면 ROS 2 패키지 공유 디렉토리
    (ament_index_python)를 우선 시도하고, 실패하면 이 파일 기준 상대 경로
    (../config/params.yaml)로 폴백한다. surface_profiler.py, reprocess_pcd.py
    양쪽 모두 이 함수 하나로 설정을 읽는다.
    """
    import sys
    import yaml

    if config_path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share_dir = get_package_share_directory('dae_coverage_floor_flatness')
            config_path = os.path.join(package_share_dir, 'config', 'params.yaml')
        except Exception:
            base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
            config_path = os.path.join(base_dir, "config", "params.yaml")

    print(f"[*] Resolving parameters from: {config_path}")
    if not os.path.exists(config_path):
        print(f"[!] Critical Error: params.yaml file not found at {config_path}")
        sys.exit(1)

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    global_cfg = config.get('global', {})
    workspace_root = os.path.expanduser(global_cfg.get('workspace_root', '~/dae_floor_maps'))
    profiling_cfg = config.get('surface_profiling', {})
    return workspace_root, profiling_cfg


def resolve_pointcloud_dir(workspace_root, profiling_cfg):
    path = os.path.join(workspace_root, profiling_cfg.get('pointcloud_dir', 'analytics/pointclouds'))
    os.makedirs(path, exist_ok=True)
    return path


def resolve_visualization_dir(workspace_root, profiling_cfg):
    path = os.path.join(workspace_root, profiling_cfg.get('visualization_dir', 'visualization/surface_profiling'))
    os.makedirs(path, exist_ok=True)
    return path


def resolve_map_yaml_path(workspace_root, profiling_cfg, map_filename="map_from_dae.yaml"):
    """params.yaml의 map_yaml_dir(예: 'maps/grid', 폴더명)을
    workspace_root와 결합하고 실제 yaml 파일명까지 붙여 완전한 절대 경로로 만든다.

    params.yaml에 map_yaml_dir 키가 아예 없으면 None을 반환한다
    (맵 오버레이 없이 히트맵만 단독 생성하는 기존 동작 유지).
    """
    rel_map_dir = profiling_cfg.get('map_yaml_dir', None)
    if rel_map_dir is None:
        return None

    map_folder = os.path.join(workspace_root, rel_map_dir)
    os.makedirs(map_folder, exist_ok=True)
    return os.path.join(map_folder, map_filename)
