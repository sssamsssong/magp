#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from enum import Enum, auto


class Stage(Enum):
    IDLE       = auto()
    WAIT_START = auto()
    Q1         = auto()
    Q2         = auto()
    Q3         = auto()
    DONE       = auto()


class FullyAutonomousManager(Node):
    def __init__(self):
        super().__init__('fully_autonomous_manager')

        self.declare_parameter('q1_required_laps',    3)
        self.declare_parameter('q2_required_laps',    5)
        self.declare_parameter('q3_required_laps',   20)
        self.declare_parameter('start_timeout_sec', 120.0)

        self.q1_laps = self.get_parameter('q1_required_laps').value
        self.q2_laps = self.get_parameter('q2_required_laps').value
        self.q3_laps = self.get_parameter('q3_required_laps').value

        self.stage            = Stage.IDLE
        self.lap_count        = 0
        self.stage_lap_offset = 0

        self.sub_start = self.create_subscription(Bool,  '/race/start', self.start_cb, 10)
        self.sub_lap   = self.create_subscription(Int32, '/lap_count',  self.lap_cb,   10)

        self.pub_stage      = self.create_publisher(String, '/race/stage',                   10)
        self.pub_enable_obs = self.create_publisher(Bool,   '/obstacles/enabled',            10)
        self.pub_enable_opp = self.create_publisher(Bool,   '/opponent/tracking_enabled',    10)
        self.pub_go         = self.create_publisher(Bool,   '/drive/enabled',                10)
        self.pub_estop      = self.create_publisher(Bool,   '/emergency_stop',               10)

        self.create_timer(1.0, self.status_timer)
        self._set_stage(Stage.WAIT_START)
        self.get_logger().info('FA Manager ready. Waiting for /race/start ...')

    def _set_stage(self, new_stage):
        self.stage = new_stage
        self.stage_lap_offset = self.lap_count
        self.get_logger().info(f'[FA] → {new_stage.name}')

        if new_stage == Stage.WAIT_START:
            self._go(False); self._obs(False); self._opp(False)
        elif new_stage == Stage.Q1:
            self._obs(False); self._opp(False); self._go(True)
        elif new_stage == Stage.Q2:
            self._obs(True);  self._opp(False); self._go(True)
        elif new_stage == Stage.Q3:
            self._obs(True);  self._opp(True);  self._go(True)
        elif new_stage == Stage.DONE:
            self._go(False)

        s = String(); s.data = new_stage.name
        self.pub_stage.publish(s)

    def start_cb(self, msg):
        if msg.data and self.stage == Stage.WAIT_START:
            self.get_logger().info('[FA] Start signal received!')
            self._set_stage(Stage.Q1)

    def lap_cb(self, msg):
        self.lap_count = msg.data
        laps = self.lap_count - self.stage_lap_offset
        if   self.stage == Stage.Q1 and laps >= self.q1_laps: self._set_stage(Stage.Q2)
        elif self.stage == Stage.Q2 and laps >= self.q2_laps: self._set_stage(Stage.Q3)
        elif self.stage == Stage.Q3 and laps >= self.q3_laps: self._set_stage(Stage.DONE)

    def status_timer(self):
        s = String(); s.data = self.stage.name
        self.pub_stage.publish(s)

    def _go(self, v):  m = Bool(); m.data = v; self.pub_go.publish(m)
    def _obs(self, v): m = Bool(); m.data = v; self.pub_enable_obs.publish(m)
    def _opp(self, v): m = Bool(); m.data = v; self.pub_enable_opp.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FullyAutonomousManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()