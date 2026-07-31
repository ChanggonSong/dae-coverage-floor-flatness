# mission_execution/utils/map_utils.py

import os
import yaml
import cv2


def get_map_bounds(yaml_path, safety_margin=0.1):
    """YAML 파일과 연결된 PGM 이미지를 읽어 맵의 물리적 좌표 boundary를 계산."""
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"[-] YAML file not found: {yaml_path}")

    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)

    resolution = map_data['resolution']
    origin = map_data['origin']

    yaml_dir = os.path.dirname(yaml_path)
    image_filename = map_data['image']
    image_path = os.path.join(yaml_dir, image_filename)

    img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"[-] Map image not found: {image_path}")

    height_px, width_px = img.shape[:2]

    bounds = {
        'min_x': origin[0] + safety_margin,
        'min_y': origin[1] + safety_margin,
        'max_x': origin[0] + (width_px * resolution) - safety_margin,
        'max_y': origin[1] + (height_px * resolution) - safety_margin
    }
    return bounds