import math
import cv2
import fields2cover as f2c

def mask_to_f2c_cells(mask):
    """
    OpenCV의 Contours를 사용하여 이진 마스크(Binary Mask)에서 외곽선을 추출하고, 
    이를 Fields2Cover(F2C) 연산에 필요한 Cell 객체로 변환합니다.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: 
        return None
        
    ring = f2c.LinearRing()
    # 외곽선 좌표를 F2C Polygon 형태(LinearRing)로 변환
    for pt in contours[0]:
        ring.addPoint(float(pt[0][0]), float(pt[0][1]))
    # 폐곡선을 만들기 위해 시작점 다시 추가
    ring.addPoint(float(contours[0][0][0][0]), float(contours[0][0][0][1]))
    
    cell = f2c.Cell()
    cell.addRing(ring)
    
    cells = f2c.Cells()
    cells.addGeometry(cell)
    return cells

def generate_raw_swaths(mask, robot_params, decompose=False, split_angle_rad=None):
    """
    decompose=True면, mask 폴리곤을 문지방 경계마다 볼록 조각으로 분할
    (Boustrophedon Decomposition)한 뒤, 가장 큰 조각(본체)만 골라 로봇의
    실제 물리 폭(width_px) 기준으로 중심선 스와스 1개를 생성한다.

    문지방 돌출부(작은 조각들)는 F2C에 넘기지 않는다 - Leg2(문지방 중심점 ->
    coverage 시작점)가 이미 record_pcd=True로 그 위를 지나가므로 F2C가 별도로
    커버할 필요가 없다(검증: node_002 사례에서 이미 확인됨).

    기존 swath_width_px(라이다 측정반경 기반) 방식은 wide 노드 및
    decompose=False일 때 그대로 유지한다 - 그 용도(넓은 방에서 평행선 간격)엔
    원래도 맞는 값이었다.
    """
    f2c_cells = mask_to_f2c_cells(mask)
    if f2c_cells is None:
        return []

    if decompose:
        decomp = f2c.DECOMP_Boustrophedon()
        decomp.setSplitAngle(split_angle_rad if split_angle_rad is not None else 0.0)
        decomp_cells = decomp.decompose(f2c_cells)

        const_hl = f2c.HG_Const_gen()
        decomp_cells = const_hl.generateHeadlands(decomp_cells, 0.0)

        if decomp_cells.size() == 0:
            return []

        areas = [decomp_cells.getGeometry(i).area() for i in range(decomp_cells.size())]
        big_idx = areas.index(max(areas))

        main_body_cells = f2c.Cells()
        main_body_cells.addGeometry(decomp_cells.getGeometry(big_idx))

        op_width = robot_params['width_px']
        target_cells = main_body_cells
    else:
        op_width = robot_params['swath_width_px']
        target_cells = f2c_cells

    robot = f2c.Robot(op_width, op_width)
    swath_gen = f2c.SG_BruteForce()

    if split_angle_rad is not None and decompose:
        raw_swaths_by_cells = swath_gen.generateSwaths(split_angle_rad, robot.getWidth(), target_cells)
    else:
        swath_gen.setStepAngle(math.pi / 36.0)
        obj = f2c.OBJ_SwathLength()
        raw_swaths_by_cells = swath_gen.generateBestSwaths(obj, robot.getWidth(), target_cells)

    swath_pairs = []
    n_cells = target_cells.size()
    for cell_idx in range(n_cells):
        try: cell_swaths = raw_swaths_by_cells[cell_idx]
        except Exception:
            try: cell_swaths = raw_swaths_by_cells.at(cell_idx)
            except Exception: continue

        n_swaths = cell_swaths.size()
        if n_swaths == 0:
            continue

        # [핵심] decompose 모드에서는 여러 줄(narrow 폭 대비 촘촘한 간격) 중
        # 가운데 하나만 골라 왕복 없이 중심선 하나로 지나간다 - 이동하면서
        # 사각지대가 메워진다는 전제(assist_mask/capture_tail 로직)와 일관됨.
        indices_to_use = [n_swaths // 2] if decompose else range(n_swaths)

        for i in indices_to_use:
            try: sw = cell_swaths[i]
            except Exception: sw = cell_swaths.at(i)
            p1 = (int(sw.startPoint().getX()), int(sw.startPoint().getY()))
            p2 = (int(sw.endPoint().getX()), int(sw.endPoint().getY()))
            swath_pairs.append((p1, p2))

    return swath_pairs

def generate_swath_path(mask, robot_params):
    """
    단일 노드의 마스크와 로봇 파라미터를 기반으로 Boustrophedon 커버리지 경로를 생성합니다.
    
    Args:
        mask (np.ndarray): 탐색할 노드의 이진 마스크
        robot_params (dict): width_px, swath_width_px 등을 포함하는 파라미터 딕셔너리
        
    Returns:
        f2c.Route: 최적화된 스와스 연결 경로 (실패 시 None)
    """
    f2c_cells = mask_to_f2c_cells(mask)
    if f2c_cells is None: 
        return None

    # 로봇 객체 생성 (전달받은 파라미터 활용)
    robot = f2c.Robot(robot_params['swath_width_px'], robot_params['swath_width_px'])

    # 1. Swath Generation (Brute Force 방식으로 최적 각도 탐색)
    swath_gen = f2c.SG_BruteForce()
    swath_gen.setStepAngle(math.pi / 36.0) # 5도 단위 탐색
    obj = f2c.OBJ_SwathLength() # 스와스 길이가 최소가 되는 방향을 선호
    
    # 생성된 원시 스와스 배열
    raw_swaths_by_cells = swath_gen.generateBestSwaths(obj, robot.getWidth(), f2c_cells)
    
    # F2C 버전 호환성을 위한 예외 처리
    try: 
        actual_swaths = raw_swaths_by_cells[0]
    except:
        try: 
            actual_swaths = raw_swaths_by_cells.at(0)
        except: 
            return None
            
    # 2. Route Planning (Boustrophedon: 소 결이 밭을 가는 형태의 지그재그 경로 연결)
    route_planner = f2c.RP_Boustrophedon()
    try: 
        route = route_planner.genSortedSwaths(actual_swaths)
    except: 
        route = route_planner.genSortedSwaths(actual_swaths, 0)
              
    return route
