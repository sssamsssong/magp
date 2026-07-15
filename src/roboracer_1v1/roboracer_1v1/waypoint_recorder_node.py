#!/usr/bin/env python3
"""
waypoint_recorder_node.py
--------------------------
수동 주행 중 /ego_racecar/odom 을 기록해서 Waypoint CSV 생성.

사용법:
  1. 수동 주행 시작
  2. 이 노드 실행
  3. 한 바퀴 돌고 나서 Ctrl+C
  4. waypoints.csv 생성됨
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import csv
import math
import os


class WaypointRecorderNode(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_node')

        self.declare_parameter('output_file',    '/home/orin/waypoints.csv')
        self.declare_parameter('min_distance',   0.1)   # 최소 기록 간격 [m]
        self.declare_parameter('default_speed',  2.0)   # 기본 속도 [m/s]

        self.output_file   = self.get_parameter('output_file').value
        self.min_distance  = self.get_parameter('min_distance').value
        self.default_speed = self.get_parameter('default_speed').value

        self.waypoints     = []
        self.last_x        = None
        self.last_y        = None
        self.recording     = True

        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, 10)
        self.sub_stop = self.create_subscription(Bool,     '/waypoint_recorder/stop', self.stop_cb, 10)

        self.get_logger().info(f'WaypointRecorder started. Saving to {self.output_file}')
        self.get_logger().info('Drive the car manually for one lap, then Ctrl+C or publish /waypoint_recorder/stop')

    def odom_cb(self, msg):
        if not self.recording:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # quaternion → yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw  = math.atan2(siny, cosy)

        # 속도
        vx = msg.twist.twist.linear.x
        speed = max(abs(vx), self.default_speed)

        # 최소 거리 체크
        if self.last_x is not None:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            if dist < self.min_distance:
                return

        self.waypoints.append([x, y, yaw, speed])
        self.last_x = x
        self.last_y = y

        if len(self.waypoints) % 50 == 0:
            self.get_logger().info(f'Recorded {len(self.waypoints)} waypoints...')

    def stop_cb(self, msg):
        if msg.data:
            self.save_and_exit()

    def save_and_exit(self):
        self.recording = False
        self._save()
        self.get_logger().info(f'Saved {len(self.waypoints)} waypoints to {self.output_file}')

    def _save(self):
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        with open(self.output_file, 'w', newline='') as f:
            writer = csv.writer(f, delimiter=';')
            writer.writerow(['# waypoints recorded by waypoint_recorder_node'])
            writer.writerow(['# s_m; x_m; y_m; psi_rad; vx_mps'])
            writer.writerow(['s_m', 'x_m', 'y_m', 'psi_rad', 'vx_mps'])
            s = 0.0
            for i, wp in enumerate(self.waypoints):
                if i > 0:
                    dx = wp[0] - self.waypoints[i-1][0]
                    dy = wp[1] - self.waypoints[i-1][1]
                    s += math.hypot(dx, dy)
                writer.writerow([f'{s:.4f}', f'{wp[0]:.7f}', f'{wp[1]:.7f}',
                                 f'{wp[2]:.7f}', f'{wp[3]:.4f}'])

    def destroy_node(self):
        if self.recording and self.waypoints:
            self.save_and_exit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
