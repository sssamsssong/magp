#!/usr/bin/env python3
"""
Safety Node (AEB) — Time-To-Collision 기반 자동 비상 브레이크.

TTC = range / max(속도의 해당 방향 성분, ε)
TTC가 임계값 미만이면 mux의 최우선 토픽(/teleop, priority 100)으로
정지 명령을 발행해 어떤 컨트롤러 명령보다 우선한다.

주의: 대회 규정상 '수동' 비상브레이크는 금지지만, 온보드에서 자율적으로
동작하는 충돌 방지 로직은 자율주행의 일부이다. 하드웨어 킬스위치(토글식)는
별도로 반드시 장착해야 한다.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.declare_parameter('ttc_threshold', 0.6)
        self.declare_parameter('hold_sec', 1.0)

        self.speed = 0.0
        self.braking_until = None

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        # mux에서 /teleop 이 priority 100 → 컨트롤러(/drive, 10)보다 우선
        self.brake_pub = self.create_publisher(AckermannDriveStamped, '/teleop', 10)
        self.get_logger().info('SafetyNode(AEB) 시작')

    def odom_cb(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x

    def scan_cb(self, scan: LaserScan):
        now = self.get_clock().now()
        if self.braking_until is not None:
            if now < self.braking_until:
                self.publish_brake()
                return
            self.braking_until = None

        if abs(self.speed) < 0.1:
            return

        ranges = np.asarray(scan.ranges, dtype=float)
        n = len(ranges)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        # 진행 방향 성분 속도 (후진 시 뒤쪽 스캔 고려)
        proj = self.speed * np.cos(angles)
        valid = np.isfinite(ranges) & (ranges > 0.05) & (proj > 0.05)
        if not valid.any():
            return
        ttc = ranges[valid] / proj[valid]
        if float(np.min(ttc)) < self.get_parameter('ttc_threshold').value:
            self.get_logger().warn(f'AEB! min TTC={float(np.min(ttc)):.2f}s')
            self.braking_until = now + rclpy.duration.Duration(
                seconds=self.get_parameter('hold_sec').value)
            self.publish_brake()

    def publish_brake(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        self.brake_pub.publish(msg)


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
