import collada
import numpy as np
import cv2
import yaml
import os

class MapGenerator:
    def __init__(self, dae_path, output_dir, resolution):
        # main.py에서 resolution 기본 설정이 되지 않았다면, 해상도(resolution)는 0.05 (5cm/px)로 설정됨
        # 다만 맵을 너무 고해상도로 높이면, 이후 연산 속도가 매우 느려짐.
        """
        3D 도면(.dae)을 2D Map(.yaml, .pgm)으로 변환하는 생성기
        """
        self.dae_path = dae_path
        self.output_dir = output_dir
        self.resolution = resolution
        
        # 출력 폴더가 없으면 생성
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

    def generate_2d_map(self):
        print(f"Using {self.dae_path}")
        
        mesh = collada.Collada(self.dae_path)

        # 1. 맵 범위 파악 및 스케일 계산
        all_vertices = []
        for geom in mesh.geometries:
            for prim in geom.primitives:
                if isinstance(prim, collada.triangleset.TriangleSet):
                    all_vertices.append(prim.vertex.reshape(-1, 3))

        if not all_vertices:
            raise ValueError("[!] No valid geometry found in DAE file.")

        all_v = np.vstack(all_vertices)
        
        raw_x_range = all_v[:,0].max() - all_v[:,0].min()
        raw_z_range = all_v[:,2].max() - all_v[:,2].min()
        
        print("\n" + "="*50)
        print(f"[DEBUG] Raw Z Range: {raw_z_range:.4f}")

        # 값을 확인해 도면의 단위를 임의로 추정하는 로직.
        # Z가 100 내외이고 X가 1000 내외라면 Inches일 확률 매우 높음.
        if 80 < raw_z_range < 150:
            print("[INFO] Unit Detected: Inches (Based on Ceiling Height)")
            scale_factor = 0.0254
        elif raw_x_range > 1000:
            print("[INFO] Unit Detected: Millimeters")
            scale_factor = 0.001
        else:
            print("[INFO] Unit Detected: Meters")
            scale_factor = 1.0
        
        print(f"Applying Scale Factor: {scale_factor}")
        print("="*50 + "\n")

        all_v = all_v * scale_factor
        
        padding = 2.0

        x_min, x_max = all_v[:,0].min()- padding, all_v[:,0].max() + padding
        y_min, y_max = all_v[:,1].min()- padding, all_v[:,1].max() + padding

        print(f"x_min = {x_min}")
        print(f"x_max = {x_max}")
        print(f"y_min = {y_min}")
        print(f"y_max = {y_max}")

        width = int(np.ceil((x_max - x_min) / self.resolution))
        height = int(np.ceil((y_max - y_min) / self.resolution))
        
        pgm = np.ones((height, width), dtype=np.uint8) * 255
        
        def to_px(v):
            px_x = int((v[0] - x_min) / self.resolution)
            px_y = height - 1 - int((v[1] - y_min) / self.resolution)
            return (max(0, min(width - 1, px_x)), max(0, min(height - 1, px_y)))

        # 3. 메쉬 데이터를 순회하며 벽면(장애물) 그리기
        obstacle_triangles = 0
        
        for geom in mesh.geometries:
            for prim in geom.primitives:
                if isinstance(prim, collada.triangleset.TriangleSet):
                    vertex_array = prim.vertex
                    
                    for tri_indices in prim.vertex_index:
                        v0, v1, v2 = vertex_array[tri_indices] * scale_factor
                        
                        z_min, z_max = min(v0[2], v1[2], v2[2]), max(v0[2], v1[2], v2[2])
                        
                        if z_max > 0.5 and z_min < 1.5:
                            obstacle_triangles += 1
                            # 실제 좌표(m)를 이미지 픽셀(px)로 변환
                            pt0, pt1, pt2 = to_px(v0), to_px(v1), to_px(v2)
                            
                            # OpenCV의 C++ 라인 드로잉 사용 (0: 검은색/장애물로 처리)
                            # 벽면 안쪽을 채워주는 것은 이후 로직에서 구현됨.
                            thickness = 1  # or 2
                            pts = np.array([pt0, pt1, pt2], dtype=np.int32)
                            cv2.polylines(pgm, [pts], isClosed=True, color=0, thickness=thickness)
                            cv2.fillPoly(pgm, [pts], color=0)

        print(f"[DEBUG] Obstacle Triangles (Slice 0.5m~1.5m): {obstacle_triangles}")
        print(f" -> Map size calculated: {width} x {height}")
        print("Debug Info:")
        print("Map size (px):", width, height)
        print("Map size (m):", width * self.resolution, height * self.resolution)
        
        # 4. 파일 저장 로직 (경로 동적 할당)
        pgm_filename = "map_from_dae.pgm"
        yaml_filename = "map_from_dae.yaml"
        pgm_path = os.path.join(self.output_dir, pgm_filename)
        yaml_path = os.path.join(self.output_dir, yaml_filename)

        success = cv2.imwrite(pgm_path, pgm)
        print("save result:", success)
        debug_dir = os.path.join(os.path.dirname(self.output_dir), "debug_image")
        if not os.path.exists(debug_dir):
            os.makedirs(debug_dir)

        # PGM 시각화 이미지 저장
        visual_path = os.path.join(debug_dir, "pgm_visualize.png")
        cv2.imwrite(visual_path, pgm)  # 필요시 컬러맵 적용 가능
        print(f" -> PGM visual image saved at: {visual_path}")

        map_yaml = {
            "image": pgm_filename,
            "resolution": self.resolution,
            "origin": [float(x_min), float(y_min), 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196
        }
        with open(yaml_path, "w") as f:
            yaml.dump(map_yaml, f, default_flow_style=False)
            
        print(f" -> Generation Complete. Saved at: {yaml_path}")
        
        return yaml_path
        
