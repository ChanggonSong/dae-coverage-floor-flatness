# mission_generation/environment_modeling/__main__.py

import yaml
import os
from ament_index_python.packages import get_package_share_directory

from environment_modeling.environment_modeler import EnvironmentModeler

def main(args=None):

    try:
        # colcon build를 통해 install/ 폴더에 설치된 share 디렉토리 참조
        package_share_dir = get_package_share_directory('dae_coverage_floor_flatness')
        config_path = os.path.join(package_share_dir, 'config', 'params.yaml')
    except Exception as e:
        # 빌드 전 작업 공간에서 직접 실행(로컬 디버깅)할 때를 위한 Fallback 경로 계산
        current_dir = os.path.dirname(os.path.abspath(__file__)) # environment_modeling/
        config_path = os.path.abspath(os.path.join(current_dir, "..", "..", "..", "config", "params.yaml"))
    
    print(f"[*] Loading config from: {config_path}")

    if not os.path.exists(config_path):
        print(f"[!] Critical Error: Config file not found at {config_path}")
        return

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    modeler = EnvironmentModeler(config)
    success = modeler.build_environment_model()
    
    if not success:
        print("[!] Environment modeling failed. Aborting process.")
        
    print("[+] Environment modeling module completed successfully.")

if __name__ == "__main__":
    main()
