import cv2
import numpy as np

def split_node_at_mid(node, x, y, w, h):
    """
    노드의 긴 축을 정중앙에서 절단하여 두 개의 자식 노드로 분할함.
    """
    mask = node['driveable_mask']
    non_drive = node.get('nondriveable_mask')
    
    # 새로운 마스크를 담을 배열 초기화
    m1, m2 = np.zeros_like(mask), np.zeros_like(mask)
    # n1, n2는 일단 전체 벽면 정보를 복사하여 유실을 원천 차단
    temp_n1, temp_n2 = non_drive.copy(), non_drive.copy()
    
    # 1. 축 결정 및 주행 영역 분할
    if w >= h:
        split_idx = x + (w // 2)
        m1[:, :split_idx] = mask[:, :split_idx]
        m2[:, split_idx:] = mask[:, split_idx:]
    else:
        split_idx = y + (h // 2)
        m1[:split_idx, :] = mask[:split_idx, :]
        m2[split_idx:, :] = mask[split_idx:, :]

    # 2. 결과 노드 생성
    res_pieces = []
    for m, n_all in [(m1, temp_n1), (m2, temp_n2)]:
        if cv2.countNonZero(m) > 0:
            num_labels, labels = cv2.connectedComponents(m, connectivity=8)
            
            for j in range(1, num_labels):
                sub_m = np.zeros_like(m)
                sub_m[labels == j] = 255
                
                # 유동적 팽창(dilate) 필터링
                # 자른 조각의 크기를 기반으로 팽창 범위 결정
                _, _, sub_w, sub_h = cv2.boundingRect(cv2.findNonZero(sub_m))
                # 단축 또는 장축의 절반만큼 충분히 확장 (최소 20px 정도 여유)
                dynamic_iter = max(sub_w, sub_h) // 2 + 15 
                
                dil_kernel = np.ones((5, 5), np.uint8)
                expanded_sub_m = cv2.dilate(sub_m, dil_kernel, iterations=int(dynamic_iter))
                sub_n = cv2.bitwise_and(n_all, expanded_sub_m)
                
                new_node = {
                    'id': None, 
                    'driveable_mask': sub_m,
                    'nondriveable_mask': sub_n,
                    'area_px': cv2.countNonZero(sub_m)
                }
                res_pieces.append(new_node)
                
    return res_pieces

def subdivide_long_nodes(nodes, max_aspect_ratio, min_area_px):
    """
    이미 볼록하게 분할된 노드들 중, 지나치게 긴 노드(복도 등)를 
    종횡비 기준으로 추가 분할하는 함수.
    """
    refined_nodes = []
    
    for node in nodes:
        mask = node['driveable_mask']

        area_px = node.get('area_px', cv2.countNonZero(mask))

        x, y, w, h = cv2.boundingRect(cv2.findNonZero(mask))
        
        # 종횡비 계산
        aspect_ratio = max(w, h) / min(w, h)
        
        if aspect_ratio > max_aspect_ratio:
            # 긴 축을 기준으로 절반 분할 로직 실행
            split_nodes = split_node_at_mid(node, x, y, w, h) 
            if any(n['area_px'] < min_area_px for n in split_nodes):
                refined_nodes.append(node)
            else:
                refined_nodes.extend(
                    subdivide_long_nodes(
                        split_nodes,
                        max_aspect_ratio,
                        min_area_px
                    )
                )
        else:
            refined_nodes.append(node)
            
    return refined_nodes
