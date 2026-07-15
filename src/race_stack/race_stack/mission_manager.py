#!/usr/bin/env python3
"""
Mission Manager — 랩 카운트 + 제한시간 + 자동 정지.

/amcl_pose로 출발선(start_x, start_y, start_radius) 통과를 감지해
랩을 센다. 설정 랩수 완주 또는 제한시간 도달 시 /race/enabled=False를
발행해 컨트롤러를 멈춘다 (Fully Autonomous: 사람이 손 안 대고 자동 정지).

발행: /race/enabled (Bool, latched), /race/lap (Int32)
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int32


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('laps', 0)            # 0 = 무제한
        self.declare_parameter('time_limit_sec', 0)  # 0 = 무제한
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_radius', 1.5)

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enabled_pub = self.create_publisher(Bool, '/race/enabled', latched)
        self.lap_pub = self.create_publisher(Int32, '/race/lap', latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.pose_cb, 10)

        self.lap = 0
        self.inside = True          # 시작 시 출발선 안에 있다고 가정
        self.armed = False          # 한 번 출발선을 벗어나야 카운트 시작
        self.start_time = self.get_clock().now()
        self.finished = False
        self.lap_start = self.start_time

        self.set_enabled(True)
        self.create_timer(0.5, self.check_time)
        self.get_logger().info(
            f"MissionManager: laps={self.p('laps')}, "
            f"time_limit={self.p('time_limit_sec')}s")

    def p(self, name):
        return self.get_parameter(name).value

    def set_enabled(self, val: bool):
        self.enabled_pub.publish(Bool(data=val))

    def check_time(self):
        tl = self.p('time_limit_sec')
        if self.finished or tl <= 0:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed >= tl:
            self.get_logger().info(f'제한시간 {tl}s 도달 → 정지')
            self.finish()

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        if self.finished:
            return
        dx = msg.pose.pose.position.x - self.p('start_x')
        dy = msg.pose.pose.position.y - self.p('start_y')
        inside_now = math.hypot(dx, dy) < self.p('start_radius')

        if not inside_now:
            self.armed = True
        if inside_now and not self.inside and self.armed:
            # 출발선 재진입 = 1랩 완료
            self.lap += 1
            now = self.get_clock().now()
            lap_time = (now - self.lap_start).nanoseconds * 1e-9
            self.lap_start = now
            self.lap_pub.publish(Int32(data=self.lap))
            self.get_logger().info(f'LAP {self.lap} 완료! ({lap_time:.2f}s)')
            if 0 < self.p('laps') <= self.lap:
                self.get_logger().info('목표 랩수 완주 → 정지')
                self.finish()
        self.inside = inside_now

    def finish(self):
        self.finished = True
        self.set_enabled(False)


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_enabled(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
