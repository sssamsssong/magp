#!/usr/bin/env python3
"""
Pure Pursuit — Q1 / Q3용 레이싱라인 추종 컨트롤러.

waypoints CSV (x, y[, speed]) 를 읽어 lookahead 지점을 추종한다.
위치는 AMCL(/amcl_pose) 또는 tf(map→base_link)에서 얻는다.

구독: /amcl_pose, /race/enabled
발행: /drive
"""
import csv
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('min_speed', 1.0)
        self.declare_parameter('lookahead', 1.8)
        self.declare_parameter('lookahead_speed_gain', 0.3)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steer', 0.36)
        self.declare_parameter('curvature_slowdown', 1.5)

        wp_file = self.get_parameter('waypoint_file').value
        self.waypoints = self.load_waypoints(wp_file)
        if self.waypoints is None:
            self.get_logger().error(
                f'웨이포인트 파일을 못 읽음: "{wp_file}" — pure_pursuit 정지 상태. '
                f'./race.sh log <이름> 으로 웨이포인트를 먼저 기록하세요.')
        else:
            self.get_logger().info(f'웨이포인트 {len(self.waypoints)}개 로드')

        self.enabled = True
        self.last_speed = 0.0
        self.create_subscription(Bool, '/race/enabled', self.enabled_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.pose_cb, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def load_waypoints(self, path):
        try:
            pts = []
            with open(path) as f:
                for row in csv.reader(f):
                    if not row or row[0].startswith('#'):
                        continue
                    x, y = float(row[0]), float(row[1])
                    v = float(row[2]) if len(row) > 2 else -1.0
                    pts.append((x, y, v))
            return np.array(pts) if pts else None
        except (OSError, ValueError):
            return None

    def p(self, name):
        return self.get_parameter(name).value

    def enabled_cb(self, msg: Bool):
        if self.enabled and not msg.data:
            self.publish_stop()
        self.enabled = msg.data

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        if not self.enabled or self.waypoints is None:
            return
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        # 속도 비례 lookahead
        L = self.p('lookahead') + self.p('lookahead_speed_gain') * self.last_speed

        # 가장 가까운 웨이포인트부터 앞으로 진행하며 lookahead 지점 탐색
        d = np.hypot(self.waypoints[:, 0] - px, self.waypoints[:, 1] - py)
        nearest = int(np.argmin(d))
        n = len(self.waypoints)
        target = None
        for k in range(n):
            i = (nearest + k) % n
            if d[i] >= L:
                target = self.waypoints[i]
                break
        if target is None:
            target = self.waypoints[(nearest + n // 4) % n]

        # 차량 좌표계로 변환
        dx = target[0] - px
        dy = target[1] - py
        lx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        ly = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        if lx <= 0.0:   # 목표가 뒤에 있으면 강하게 회전만
            steer = self.p('max_steer') * (1.0 if ly > 0 else -1.0)
            curvature = 999.0
        else:
            Ld = math.hypot(lx, ly)
            curvature = 2.0 * ly / (Ld * Ld)
            steer = math.atan(self.p('wheelbase') * curvature)
        steer = float(np.clip(steer, -self.p('max_steer'), self.p('max_steer')))

        # 속도: 웨이포인트에 speed 열이 있으면 그 값, 없으면 곡률 기반 감속
        if target[2] > 0:
            speed = min(target[2], self.p('max_speed'))
        else:
            speed = self.p('max_speed') / (1.0 + self.p('curvature_slowdown')
                                           * abs(curvature))
        speed = float(np.clip(speed, self.p('min_speed'), self.p('max_speed')))
        self.last_speed = speed

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = speed
        out.drive.steering_angle = steer
        self.drive_pub.publish(out)

    def publish_stop(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        self.drive_pub.publish(msg)


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
