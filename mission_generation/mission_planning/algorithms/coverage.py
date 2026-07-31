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

def generate_raw_swaths(mask, robot_params, forced_angle_rad=None):
    """
    F2C의 route planner(RP_Boustrophedon/RP_CustomOrder)는 쓰지 않는다.
    필요한 건 SG_BruteForce가 뽑은 스와스들의 (시작점, 끝점) 좌표뿐이고,
    스와스 간 순서/방향 최적화는 mission_planner.py가 진입/진출 문지방
    정보를 갖고 직접 담당한다 - F2C route planner는 현재 프로젝트에서는 필요없는
    턴 경로 삽입까지 포함된 기능이라 이 프로젝트엔 과함.

    Returns:
        list[tuple]: [(start_pt, end_pt), ...] 스와스별 (시작, 끝) 픽셀 좌표.
        빈 리스트면 실패(호출부가 centroid 폴백으로 처리).
    """
    f2c_cells = mask_to_f2c_cells(mask)
    if f2c_cells is None:
        return []

    robot = f2c.Robot(robot_params['swath_width_px'], robot_params['swath_width_px'])
    swath_gen = f2c.SG_BruteForce()
    if forced_angle_rad is not None:
        raw_swaths_by_cells = swath_gen.generateSwaths(forced_angle_rad, robot.getWidth(), f2c_cells)
    else:
        swath_gen.setStepAngle(math.pi / 36.0)
        obj = f2c.OBJ_SwathLength()
        raw_swaths_by_cells = swath_gen.generateBestSwaths(obj, robot.getWidth(), f2c_cells)

    try: actual_swaths = raw_swaths_by_cells[0]
    except Exception:
        try: actual_swaths = raw_swaths_by_cells.at(0)
        except Exception: return []

    swath_pairs = []
    for i in range(actual_swaths.size()):
        try: sw = actual_swaths[i]
        except Exception: sw = actual_swaths.at(i)
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
