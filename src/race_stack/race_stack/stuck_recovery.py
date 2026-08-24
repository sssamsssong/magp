#!/usr/bin/env python3
"""stuck_recovery — 충돌로 끼어서 못 가면 조금 후진하고, 서서 위치를 다시 잡는다.

왜 필요한가
───────────
AMCL 은 `update_min_d`(2.5cm) / `update_min_a`(1.7°) 를 넘게 **움직여야만**
파티클 필터를 갱신한다. 그래서 충돌로 차가 멈추면 그 순간의 (충격으로 틀어진)
자세가 **그대로 얼어붙는다** — 정지해 있는 한 스스로 못 고친다.
움직이면 갱신이 다시 돌아 자세가 따라온다(실차에서 확인된 거동).

그래서 복구의 핵심은 "정확한 자세를 계산하는 것" 이 아니라 **일단 움직이는
것** 이다. 이 노드는 그 최소한만 한다:

    ① 끼임 감지  — 컨트롤러는 가라고 하는데(cmd_v) 실제 속도가 0 인 상태가
                   stall_sec 이상 지속
    ② 조금 후진  — back_dist(기본 0.40m) 또는 back_max_sec 까지만. 규정 7.3
                   "장애물에 너무 붙어 정지해 회피 불가 시 뒤로 조금 이동 가능"
    ③ 정지 대기  — auto_localize 는 주행 중(>0.05㎧) 재시드를 거부한다
    ④ 재정렬     — /race/relocalize 발행 → auto_localize 가 **현재 자세 주변**
                   을 다시 훑는다 (relocalize_from_current, 그쪽 파일 참고)
    ⑤ 복귀       — 발행을 멈추면 mux timeout 뒤 컨트롤러(/drive)로 자동 복귀

★ 라인까지 후진으로 "복귀" 하지는 않는다. 그건 좋은 위치추정을 전제로 하는데,
  지금 고치려는 게 바로 그 위치추정이라 순환이다. 끼임만 풀고 자세를 다시 잡은
  뒤, 라인 복귀는 평소대로 컨트롤러가 전진하며 하게 둔다.

⚠⚠ 후방 사각지대 — 반드시 읽을 것 ⚠⚠
  2호차 라이다는 **±135°** 라 **정후방 ±45° 가 안 보인다.** 바로 뒤에 차가
  있어도 이 노드는 모른다. 볼 수 있는 후방 대각선(|각도| ≥ rear_from_deg)만
  감시하며, 그마저 없으면 경고만 남기고 진행한다(steer_trim.py 가 같은 한계를
  만나 "뒤쪽 공간도 사람이 직접 확보해둘 것" 이라고 적어둔 것과 같은 문제).
  그래서 후진량·시간·시도횟수를 모두 작게 못박아 둔다 — 사각지대에서 오래
  움직이지 않는 것이 유일한 방어다.

★ 사람이 항상 이긴다: 발행 채널 recovery 는 mux 우선순위 50 이고 조이스틱
  /teleop 은 100 이다. LB 로 수동 전환하면 복구는 즉시 무시된다.
"""
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32

IDLE, BACKUP, SETTLE, RELOC, COOLDOWN = 0, 1, 2, 3, 4
STATE_NAME = {IDLE: 'IDLE', BACKUP: 'BACKUP', SETTLE: 'SETTLE',
              RELOC: 'RELOC', COOLDOWN: 'COOLDOWN'}


class StuckRecovery(Node):
    def __init__(self):
        super().__init__('stuck_recovery')
        d = self.declare_parameter

        # ★ 기본 False — 실차 미검증 기능이라 명시적으로 켜야 돈다.
        d('enabled', False)

        # ── 끼임 판정 ────────────────────────────────────────────────
        #   "가라고 하는데 안 간다" 가 유일한 근거다. 컨트롤러가 스스로 멈춘
        #   경우(BLOCKED 감속 등)는 정상 동작이므로 건드리면 안 된다.
        d('stall_cmd_min',  0.30)   # 컨트롤러가 이 이상 요구하는데
        d('stall_speed_max', 0.10)  # 실제 속도가 이 밑이고
        d('stall_sec',       1.5)   # 이만큼 지속되면 끼임

        # ── 후진 ─────────────────────────────────────────────────────
        d('back_speed',   -0.6)     # m/s (음수 = 후진)
        d('back_dist',     0.40)    # m — 규정 7.3 "뒤로 조금"
        d('back_max_sec',  2.0)     # s — 거리 적산이 틀려도 여기서 끊는다
        d('back_steer',    0.0)     # rad — 기본 직선 후진

        # ── 후방 감시 (볼 수 있는 범위만) ────────────────────────────
        d('rear_from_deg', 100.0)   # |각도| 이상을 후방으로 본다
        d('rear_safety_m', 0.35)    # 이보다 가까우면 후진 중단/시작 안 함

        # ── 마무리 ───────────────────────────────────────────────────
        d('settle_sec',    0.6)     # 완전히 선 뒤 재정렬 (AMCL 재시드 조건)
        d('reloc_wait_sec', 1.5)    # auto_localize 가 맞출 시간
        d('cooldown_sec',  5.0)     # 연달아 재발동 방지
        d('max_attempts',  3)       # 이 횟수를 넘으면 포기하고 손 뗀다
        # ★ 정상 주행이 이만큼 이어지면 시도 횟수를 0 으로 되돌린다.
        #   안 그러면 경기 **초반**에 3번 끼인 것만으로 남은 경기 내내 복구가
        #   죽는다. 잘 달리고 있다는 건 그 끼임이 해결됐다는 뜻이다.
        d('reset_after_sec', 20.0)

        d('scan_topic', '/scan')
        d('odom_topic', '/odom_imu')

        p = lambda n: self.get_parameter(n).value
        self.on = bool(p('enabled'))

        self.state = IDLE
        self.t_state = self._now()      # 현재 상태 진입 시각
        self.stall_since = None
        self.attempts = 0
        self._ok_since = None           # 정상 주행이 이어진 시작 시각
        self.back_dist_acc = 0.0
        self._last_odom_t = None

        self.cmd_v = 0.0                # 컨트롤러가 원하는 속도
        self.speed = 0.0                # 실제 속도
        self.rear_min = float('inf')
        self._rear_seen = None          # 후방 빔이 있긴 한가
        self.enabled_race = False       # /race/enabled
        self.manual = False             # /race/manual

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/recovery_drive', 10)
        self.reloc_pub = self.create_publisher(Bool, '/race/relocalize', 10)
        self.state_pub = self.create_publisher(Int32, '/recovery/state', 10)

        self.create_subscription(AckermannDriveStamped, '/drive',
                                 self._drive_cb, 10)
        self.create_subscription(Odometry, str(p('odom_topic')), self._odom_cb, 20)
        self.create_subscription(LaserScan, str(p('scan_topic')),
                                 self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/race/enabled', self._enabled_cb, latched)
        self.create_subscription(Bool, '/race/manual', self._manual_cb, latched)

        self.create_timer(0.05, self._tick)     # 20Hz

        if self.on:
            self.get_logger().info(
                f'StuckRecovery 켜짐 — 끼임 판정: cmd_v≥{p("stall_cmd_min")}㎧ 인데 '
                f'실속도<{p("stall_speed_max")}㎧ 가 {p("stall_sec")}s 지속. '
                f'후진 {p("back_dist")}m / 최대 {p("back_max_sec")}s, '
                f'최대 {p("max_attempts")}회. '
                f'⚠ 정후방 ±45°는 라이다 사각지대 — 뒤쪽 공간을 확인할 것')
        else:
            self.get_logger().info(
                'StuckRecovery 꺼짐 (recovery.enabled=false) — 감시만 하지 않고 '
                '아무것도 하지 않습니다')

    # ── 입력 ──────────────────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _drive_cb(self, msg: AckermannDriveStamped):
        self.cmd_v = float(msg.drive.speed)

    def _odom_cb(self, msg: Odometry):
        v = float(msg.twist.twist.linear.x)
        self.speed = v
        t = self._now()
        if self._last_odom_t is not None and self.state == BACKUP:
            dt = t - self._last_odom_t
            if 0.0 < dt < 0.5:
                self.back_dist_acc += abs(v) * dt
        self._last_odom_t = t

    def _scan_cb(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=float)
        if r.size == 0:
            return
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        # ★ 볼 수 있는 후방만. 2호차는 ±135° 라 정후방(180°)은 애초에 없다.
        rear = ok & (np.abs(a) >= math.radians(
            float(self.get_parameter('rear_from_deg').value)))
        if rear.any():
            self.rear_min = float(r[rear].min())
            if self._rear_seen is None:
                self._rear_seen = True
        else:
            self.rear_min = float('inf')
            if self._rear_seen is None:
                self._rear_seen = False
                self.get_logger().warn(
                    '후방 빔이 하나도 없습니다 — 후진 중 뒤를 전혀 못 봅니다. '
                    '뒤쪽 공간은 사람이 확보해야 합니다')

    def _enabled_cb(self, msg: Bool):
        self.enabled_race = bool(msg.data)

    def _manual_cb(self, msg: Bool):
        self.manual = bool(msg.data)
        if self.manual and self.state != IDLE:
            self.get_logger().warn('수동 개입 — 복구 중단')
            self._to(IDLE)

    # ── 상태 전이 ─────────────────────────────────────────────────────
    def _to(self, s):
        if s != self.state:
            self.get_logger().info(
                f'[recovery] {STATE_NAME[self.state]} → {STATE_NAME[s]}')
        self.state = s
        self.t_state = self._now()
        self.state_pub.publish(Int32(data=int(s)))

    def _publish_drive(self, v, steer):
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drive.speed = float(v)
        m.drive.steering_angle = float(steer)
        self.drive_pub.publish(m)

    def _rear_blocked(self):
        return (self._rear_seen is True
                and self.rear_min < float(self.get_parameter('rear_safety_m').value))

    # ── 본체 ──────────────────────────────────────────────────────────
    def _tick(self):
        if not self.on:
            return
        p = lambda n: self.get_parameter(n).value
        t = self._now()
        el = t - self.t_state

        # 경기 중이 아니거나 사람이 잡고 있으면 아무것도 안 한다.
        if not self.enabled_race or self.manual:
            if self.state != IDLE:
                self._to(IDLE)
            self.stall_since = None
            return

        if self.state == IDLE:
            stuck = (self.cmd_v >= float(p('stall_cmd_min'))
                     and abs(self.speed) < float(p('stall_speed_max')))
            if not stuck:
                self.stall_since = None
                # 정상 주행이 충분히 이어지면 시도 횟수를 되돌린다.
                if abs(self.speed) >= float(p('stall_speed_max')):
                    if self._ok_since is None:
                        self._ok_since = t
                    elif (self.attempts > 0
                          and t - self._ok_since >= float(p('reset_after_sec'))):
                        self.get_logger().info(
                            f'정상 주행 {p("reset_after_sec")}s 지속 — '
                            f'복구 시도 횟수를 0 으로 되돌립니다')
                        self.attempts = 0
                        self._ok_since = None
                return
            self._ok_since = None
            if self.stall_since is None:
                self.stall_since = t
                return
            if t - self.stall_since < float(p('stall_sec')):
                return
            # 끼임 확정
            if self.attempts >= int(p('max_attempts')):
                self.get_logger().error(
                    f'끼임이 계속되지만 복구를 {self.attempts}회 시도해 포기합니다 — '
                    f'사람이 개입해야 합니다 (조이스틱 LB)',
                    throttle_duration_sec=5.0)
                return
            if self._rear_blocked():
                self.get_logger().warn(
                    f'끼임 감지했지만 후방 {self.rear_min:.2f}m 로 막혀 후진 못 합니다',
                    throttle_duration_sec=2.0)
                return
            self.attempts += 1
            self.back_dist_acc = 0.0
            self.get_logger().warn(
                f'⚠ 끼임 감지 ({self.attempts}/{int(p("max_attempts"))}) — '
                f'요구 {self.cmd_v:.2f}㎧ / 실제 {self.speed:.2f}㎧ 가 '
                f'{p("stall_sec")}s 지속. 후진 복구를 시작합니다')
            self._to(BACKUP)
            return

        if self.state == BACKUP:
            if self._rear_blocked():
                self.get_logger().warn(
                    f'후방 장애물 {self.rear_min:.2f}m — 후진 중단')
                self._to(SETTLE)
                return
            if self.back_dist_acc >= float(p('back_dist')):
                self.get_logger().info(f'후진 {self.back_dist_acc:.2f}m 완료')
                self._to(SETTLE)
                return
            if el >= float(p('back_max_sec')):
                self.get_logger().warn(
                    f'후진 시간 상한 {p("back_max_sec")}s 도달 '
                    f'(이동 {self.back_dist_acc:.2f}m) — 중단')
                self._to(SETTLE)
                return
            self._publish_drive(float(p('back_speed')), float(p('back_steer')))
            return

        if self.state == SETTLE:
            # 완전히 서야 auto_localize 가 재시드를 받아준다(>0.05㎧ 면 거부).
            self._publish_drive(0.0, 0.0)
            if el >= float(p('settle_sec')) and abs(self.speed) < 0.05:
                self.reloc_pub.publish(Bool(data=True))
                self.get_logger().info('/race/relocalize 발행 — 자세 재정렬 요청')
                self._to(RELOC)
            return

        if self.state == RELOC:
            self._publish_drive(0.0, 0.0)   # 재정렬 동안 계속 정지
            if el >= float(p('reloc_wait_sec')):
                self.get_logger().info('복구 완료 — 컨트롤러로 돌려줍니다')
                self._to(COOLDOWN)
            return

        if self.state == COOLDOWN:
            # 발행을 멈춘다 → mux timeout(0.3s) 뒤 /drive 로 자동 복귀.
            if el >= float(p('cooldown_sec')):
                self.stall_since = None
                self._to(IDLE)
            return


def main():
    rclpy.init()
    node = StuckRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
