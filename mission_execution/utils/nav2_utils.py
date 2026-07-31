# mission_execution/utils/nav2_utils.py

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


def _patched_set_initial_pose(self):
    """BasicNavigator._setInitialPose: 발행 시점의 실시간 타임스탬프를 사용."""
    msg = PoseWithCovarianceStamped()
    msg.pose.pose = self.initial_pose.pose
    msg.header.frame_id = self.initial_pose.header.frame_id
    msg.header.stamp = self.get_clock().now().to_msg()

    msg.pose.covariance[0] = 0.09   # X 오차 허용치 (약 30cm)
    msg.pose.covariance[7] = 0.09   # Y 오차 허용치 (약 30cm)
    msg.pose.covariance[35] = 0.068  # Yaw 회전 오차 허용치 (약 15도)

    self.info('Publishing Initial Pose (Real-time Stamp Patched)')
    self.initial_pose_pub.publish(msg)
    return


def _patched_wait_for_initial_pose(self):
    """BasicNavigator._waitForInitialPose: 2.0초 간격으로 throttle 재발행."""
    last_pub_time = 0.0
    while not self.initial_pose_received:
        curr_time = time.time()
        if curr_time - last_pub_time > 2.0:
            self.info('Setting initial pose (Throttled 2.0s)')
            self._setInitialPose()
            last_pub_time = curr_time
            self.info('Waiting for amcl_pose to be received...')
        rclpy.spin_once(self, timeout_sec=0.1)
    return


def apply_nav2_monkey_patches(navigator=None):
    """
    navigator 인자는 호출 시점 일관성을 위해 받지만, 패치는 클래스 자체에 적용되므로
    인스턴스 유무와 무관하게 한 번만 적용되면 모든 BasicNavigator 인스턴스에 영향을 줌.
    """
    BasicNavigator._setInitialPose = _patched_set_initial_pose
    BasicNavigator._waitForInitialPose = _patched_wait_for_initial_pose
    return navigator