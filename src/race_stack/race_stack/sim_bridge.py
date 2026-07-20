#!/usr/bin/env python3
"""
sim_bridge — f1tenth_gym_ros ↔ stanley_controller 어댑터.

시뮬레이터 ground-truth 위치(/ego_racecar/odom, frame_id='map')를
stanley_controller 가 기대하는 /amcl_pose + /odom 으로 변환한다.
/race/enabled=True 도 발행해 mission_manager 없이 즉시 주행을 시작한다.

시뮬에서는 amcl과 odom이 같은 map 프레임이므로 stanley의 amcl+odom 융합이
사실상 최신 odom 값(ground-truth)을 그대로 사용하는 것과 동일하다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool


class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.amcl_pub    = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.odom_pub    = self.create_publisher(Odometry,                  '/odom',      10)
        self.enabled_pub = self.create_publisher(Bool,                      '/race/enabled', latched)

        self.create_subscription(Odometry, '/ego_racecar/odom', self._odom_cb, 10)

        # latched — 나중에 구독하는 stanley_controller 도 True 를 받는다
        self.enabled_pub.publish(Bool(data=True))
        self.get_logger().info('SimBridge ready: /ego_racecar/odom → /amcl_pose + /odom')

    def _odom_cb(self, msg: Odometry):
        self.odom_pub.publish(msg)

        amcl = PoseWithCovarianceStamped()
        amcl.header = msg.header   # frame_id='map' 그대로
        amcl.pose.pose = msg.pose.pose
        self.amcl_pub.publish(amcl)


def main():
    rclpy.init()
    node = SimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
