import cv2
import numpy as np

# 시각화
import colorsys

def split_by_doors(driveable_mask, nondriveable_mask, resolution, max_door_radius, segmenter=None):
    """
    문을 기준으로 맵을 분할하는 함수.
    1. 전체 실내 공간 계산: 벽과 기둥을 제외한 맵의 모든 실내 영역을 계산함. 이때 주행 가능 여부와 무관하게, 전체 실내 공간을 대상으로 문지방 후보지를 탐색하기 위함임.
    2. - 거리 변환(Distance Transform)을 통해 벽으로부터의 거리가 '문 폭(최대 1.0m)'보다 큰 지점들을 Seed로 잡음.
    - 폭 1m 이하의 문지방 구역은 Seed가 생성되지 않아 빈 공간(Unassigned)으로 남음.
    3. Connected Components로 seed에서 분리된 각각의 덩어리를 노드로 등록함. 
    - 이때 하나의 라벨 안에 물리적으로 떨어진 덩어리가 있다면 4방향 검사로 분리하여, 완전히 독립된 노드로 등록함.
    4. 미할당 영역 계산: 초기 노드로부터 할당되지 않은 영역을 계산하여, 문지방, 벽 근처 여백, 진입 불가한 고립된 방 등을 포함하는 '미할당 영역'을 생성함.
    5. 영토(territory) 확장: 미할당 영역이 존재할 경우, 각 노드의 영토를 한 픽셀씩 확장하여, 미할당 영역이 가장 가까운 노드에 할당되도록 함. 
    - 즉 어느 노드에도 속하지 않는 영역이 있는 경우가 발생하지 않도록 함. 
    - 이때 확장된 영역이 전체 실내 공간을 벗어나지 않도록 제한함.
    6. 최종 노드 반환: 각 노드는 고유 ID, 주행 가능 마스크, 비주행 마스크, 면적 등의 정보를 포함하는 딕셔너리 형태로 반환됨.
    """
    # 1. 전체 실내 공간 
    # 벽과 기둥을 제외한 맵의 모든 실내 영역 (로봇의 주행 가능 여부와 무관하게)
    # 영토 확장은 '벽' 끝까지 가야 하기 때문.
    total_indoor_area = cv2.bitwise_or(driveable_mask, nondriveable_mask)
    
    # 2. 거리 변환 (로봇 통로 기준)
    dist_map = cv2.distanceTransform(driveable_mask, cv2.DIST_L2, 5)
    max_val = np.max(dist_map)
    
    door_threshold_px = max_door_radius / resolution

    # 맵 자체가 너무 작아 Seed가 아예 안 생기는 경우 방어
    adaptive_threshold = door_threshold_px
    if max_val < adaptive_threshold:
        adaptive_threshold = max_val * 0.8 # 맵이 너무 좁으면 중심이라도 잡음

    adaptive_threshold = max(adaptive_threshold, 1.0)

    print(f"    -> Max Depth (Driveable): {max_val:.2f} px")
    print(f"    -> Physical Door Threshold: {door_threshold_px:.2f} px")
    print(f"    -> Final Threshold: {adaptive_threshold:.2f} px")
    
    # 3. 문지방 후보지 생성 (거리 맵에서 임계값보다 먼 지점들을 seed로 사용)
    seeds_mask = np.zeros_like(driveable_mask)
    seeds_mask[dist_map > adaptive_threshold] = 255

    # 기초 노드(Seed) 생성
    # 로봇이 도달할 수 있는 '주행 가능 영역' 내의 seed만 사용하여 초기 노드로 활용함.
    seeds = cv2.bitwise_and(seeds_mask, driveable_mask)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(seeds, connectivity=4)
    
    print(f"    -> Found {num_labels - 1} potential room seeds.")
    
    nodes = []
    initial_assigned = np.zeros_like(driveable_mask)

    for i in range(1, num_labels):
        m_seed = np.zeros_like(driveable_mask)
        m_seed[labels == i] = 255
        
        # 만약 하나의 라벨 안에 물리적으로 떨어진 덩어리가 있다면 4방향 검사로 분리
        sub_num, sub_labels = cv2.connectedComponents(m_seed, connectivity=4)
        for j in range(1, sub_num):
            sub_mask = np.zeros_like(m_seed)
            sub_mask[sub_labels == j] = 255
            
            # 노드 추가
            nodes.append({
                'id': len(nodes) + 1, # ID 부여
                'driveable_mask': sub_mask.copy(),
                'nondriveable_mask': np.zeros_like(driveable_mask),
                'area_px': cv2.countNonZero(sub_mask)
            })
            # 할당된 영역 기록
            initial_assigned = cv2.bitwise_or(initial_assigned, sub_mask)

    if not nodes: 
        print("    -> No valid room nodes(seeds) found. Returning empty node list. Check your map or threshold.")
        return []
    
    if hasattr(segmenter, 'visualization_dir') and segmenter.visualization_dir:
        cv2.imwrite(f"{segmenter.visualization_dir}/step_2_debug_seeds_mask.png", seeds)
        cv2.imwrite(f"{segmenter.visualization_dir}/step_2_debug_dist_map.png", (dist_map / (max_val if max_val>0 else 1) * 255).astype(np.uint8))
        
    # 4. 미할당 영역 계산 (문지방, 벽 근처 여백, 진입 불가한 고립된 방 등)
    unassigned_area = cv2.bitwise_and(total_indoor_area, cv2.bitwise_not(initial_assigned))
    
    # 5. 영토 확장 (가까운 노드에 빈 공간 배분)
    if cv2.countNonZero(unassigned_area) > 0:
        territories = [n['driveable_mask'].copy() for n in nodes]
        dil_kernel = np.ones((3, 3), np.uint8)

        # 무한 루프 방지를 위한 안전 장치
        max_iterations = int(max(driveable_mask.shape) * 0.5)
        iteration_count = 0
        
        # 모든 미할당 영역이 주인을 찾을 때까지 반복
        while cv2.countNonZero(unassigned_area) > 0 and iteration_count < max_iterations:
            iteration_count += 1
            prev_count = cv2.countNonZero(unassigned_area)
            
            for i, node in enumerate(nodes):
                # 한 픽셀씩 확장
                dilated = cv2.dilate(territories[i], dil_kernel, iterations=1)
                dilated = cv2.bitwise_and(dilated, total_indoor_area)
                
                # 새로 점유한 영역 탐색
                claimed = cv2.bitwise_and(dilated, unassigned_area)
                
                if cv2.countNonZero(claimed) > 0:
                    # 점유한 영역을 주행 가능/불가능 특성에 맞춰 분리하여 저장
                    c_drive = cv2.bitwise_and(claimed, driveable_mask)
                    c_non = cv2.bitwise_and(claimed, nondriveable_mask)
                    
                    node['driveable_mask'] = cv2.bitwise_or(node['driveable_mask'], c_drive)
                    node['nondriveable_mask'] = cv2.bitwise_or(node['nondriveable_mask'], c_non)
                    
                    # 마스터 미할당 마스크에서 제거
                    unassigned_area = cv2.bitwise_and(unassigned_area, cv2.bitwise_not(claimed))
                
                # 다음 루프를 위해 현재 영토 업데이트
                territories[i] = dilated
                
            # 더 이상 확장할 곳이 없으면 탈출 (무한 루프 방지)
            if cv2.countNonZero(unassigned_area) == prev_count: break
            
    return nodes