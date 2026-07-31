# mission_execution/utils/visualizer.py

import csv
import matplotlib
matplotlib.use('Agg')  # GUI 백엔드 비활성화 (헤드리스 환경/Jetson 안전성 확보)
import matplotlib.pyplot as plt


def visualize_paths(csv_filename, json_path_data, img_out_path):
    """
    실제 AMCL 주행 데이터(CSV)와 계획된 경로(JSON)를 비교 시각화하여 PNG로 저장합니다.
    """
    print("\n[*] Generating Path Tracking Performance Graph...")
    try:
        # 1. JSON 파싱 (계획된 웨이포인트)
        planned_x = [wp['pose']['position']['x'] for wp in json_path_data]
        planned_y = [wp['pose']['position']['y'] for wp in json_path_data]

        # 2. CSV 파싱 (실제 AMCL 주행 궤적)
        actual_x = []
        actual_y = []
        with open(csv_filename, 'r') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                actual_x.append(float(row[1]))
                actual_y.append(float(row[2]))

        # 3. 그래프 그리기 (맵 원점 기준 1:1 매칭)
        plt.figure(figsize=(10, 8))

        # 계획된 경로 (파란색 점선)
        plt.plot(planned_x, planned_y, 'b--o', label='Planned Path (Waypoints)', markersize=4, alpha=0.6)

        # 실제 주행 궤적 (빨간색 실선)
        plt.plot(actual_x, actual_y, 'r-', label='Actual Driven Path (AMCL)', linewidth=2)

        # 시작점과 목표점 강조
        plt.plot(planned_x[0], planned_y[0], 'go', label='Start', markersize=8)
        plt.plot(planned_x[-1], planned_y[-1], 'ko', label='Goal', markersize=8)

        plt.title('Nav2 Path Tracking Performance (Map Frame)')
        plt.xlabel('X coordinate (m)')
        plt.ylabel('Y coordinate (m)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')  # 맵 비율 유지 (왜곡 방지)

        # 4. 이미지 저장 (GUI 에러 방지)
        plt.savefig(img_out_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"[+] Visualization successfully saved to '{img_out_path}'.")
    except Exception as e:
        print(f"[-] Failed to generate visualization: {e}")