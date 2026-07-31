# mission_execution/utils/ros_utils.py

import os
import rclpy
from geometry_msgs.msg import PoseStamped

try:
    from gazebo_msgs.srv import SetEntityState
    GAZEBO_AVAILABLE = True
except ModuleNotFoundError:
    GAZEBO_AVAILABLE = False


def create_pose_stamped(navigator, waypoint_dict, map_bounds):
    """파이썬 딕셔너리를 ROS 2 PoseStamped 메시지로 변환 및 클램핑."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    target_x = waypoint_dict['pose']['position']['x']
    target_y = waypoint_dict['pose']['position']['y']

    clamped_x = max(map_bounds['min_x'], min(target_x, map_bounds['max_x']))
    clamped_y = max(map_bounds['min_y'], min(target_y, map_bounds['max_y']))

    if clamped_x != target_x or clamped_y != target_y:
        print(f"[!] Waypoint Out of Bounds Corrected: ({target_x:.2f}, {target_y:.2f}) -> ({clamped_x:.2f}, {clamped_y:.2f})")

    pose.pose.position.x = clamped_x
    pose.pose.position.y = clamped_y
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = waypoint_dict['pose']['orientation']['x']
    pose.pose.orientation.y = waypoint_dict['pose']['orientation']['y']
    pose.pose.orientation.z = waypoint_dict['pose']['orientation']['z']
    pose.pose.orientation.w = waypoint_dict['pose']['orientation']['w']

    return pose


def teleport_gazebo_entity(start_pose):
    """가제보 시뮬레이션 상의 로봇을 특정 위치로 순간이동."""
    if not GAZEBO_AVAILABLE:
        print("[WARN] gazebo_msgs package not available. Skipping physical teleport.")
        return

    node = rclpy.create_node('gazebo_teleporter')
    client = node.create_client(SetEntityState, '/gazebo/set_entity_state')

    if not client.wait_for_service(timeout_sec=5.0):
        client = node.create_client(SetEntityState, '/set_entity_state')
        if not client.wait_for_service(timeout_sec=2.0):
            print("[WARN] Gazebo SetEntityState service not found. Skipping physical teleport.")
            node.destroy_node()
            return

    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    req = SetEntityState.Request()
    req.state.name = tb3_model
    req.state.pose = start_pose.pose
    req.state.reference_frame = 'world'

    print(f"[*] Teleporting {tb3_model} to the starting point in Gazebo...")
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None and future.result().success:
        print("[+] Gazebo Teleportation Successful.")
    else:
        print("[-] Gazebo Teleportation Failed.")

    node.destroy_node()