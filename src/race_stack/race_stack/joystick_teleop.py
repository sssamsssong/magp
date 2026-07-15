#!/usr/bin/env python3
"""
Joystick Teleop — 조이스틱 수동 조종.

sensor_msgs/Joy → /teleop (AckermannDriveStamped)
               → /race/enabled (Bool)

조작법 (Logitech F710, Mode 버튼 OFF):
  LB              → 수동 ↔ 자율 모드 즉시 전환 (토글)
  RT 당김          → 전진 (당긴 만큼 선형 비례, 수동 모드에서만)
  RT 놓음          → 즉시 정지  ← 회생제동처럼
  RB 누른 채 RT    → 후진
  L스틱 좌/우      → 조향

★ 트리거 부호 규약 주의
  같은 RT 트리거라도 읽는 경로에 따라 부호가 반대다:
    - 커널 /dev/input/js0 (check_joystick.py): 안 누름 = -1, 끝까지 = +1
    - ROS2 joy 노드의 /joy 토픽        : 안 누름 = +1, 끝까지 = -1   ← 이게 기준
  이 노드는 /joy 를 구독하므로 trigger_invert=true 가 기본(안 누름=+1)이다.
  거꾸로 설정하면 RT를 안 눌러도 최고 속도가 나가므로, 아래 arm_on_neutral
  안전장치가 중립을 한 번 확인하기 전까지 스로틀을 0으로 묶는다.

속도 매핑 (완전 선형):
  u = (raw - 중립) / (최대 - 중립)      → 0.0(안 누름) ~ 1.0(끝까지)
  u = deadzone 제거 후 재정규화          → 살짝 스치는 입력 무시
  speed = u ** trigger_expo * max_speed  → expo=1.0 이면 순수 선형

파라미터:
  axis_steer      = 0     L스틱 좌우 (Mode OFF 아날로그)
  axis_speed      = 5     RT 트리거
  btn_reverse     = 5     RB 버튼 — 누르는 동안 후진
  btn_toggle      = 4     LB 버튼 — 수동↔자율 토글
  trigger_invert  = true  /joy 기준 안 누름=+1 이면 true
  trigger_deadzone= 0.05  트리거 유격 (0~1 정규화 기준)
  trigger_expo    = 1.0   1.0=선형, >1 이면 저속 구간이 완만해짐
  steer_deadzone  = 0.08  스틱 중립 유격 (joy 드라이버 deadzone=0 이므로 여기서 처리)
  arm_on_neutral  = true  시작 시 트리거 중립 확인 전까지 스로틀 잠금
  steer_invert, max_speed, max_steer, debug
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        self.declare_parameter('axis_steer',       0)      # L스틱 좌우 (Mode OFF)
        self.declare_parameter('axis_speed',       5)      # RT 트리거
        self.declare_parameter('btn_reverse',      5)      # RB — 후진 전환
        self.declare_parameter('btn_toggle',       4)      # LB — 수동↔자율 토글
        self.declare_parameter('steer_invert',     False)
        self.declare_parameter('trigger_invert',   True)   # /joy 기준 안 누름=+1
        self.declare_parameter('trigger_deadzone', 0.05)
        self.declare_parameter('trigger_expo',     1.0)    # 1.0 = 완전 선형
        self.declare_parameter('steer_deadzone',   0.08)
        self.declare_parameter('arm_on_neutral',   True)
        self.declare_parameter('debug',            False)
        self.declare_parameter('max_speed',        1.0)
        self.declare_parameter('max_steer',        0.36)

        def p(n): return self.get_parameter(n).value
        self.ax_steer      = p('axis_steer')
        self.ax_speed      = p('axis_speed')
        self.btn_rev       = p('btn_reverse')
        self.btn_tog       = p('btn_toggle')
        self.inv_steer     = p('steer_invert')
        self.inv_trigger   = p('trigger_invert')
        self.trig_dz       = min(max(float(p('trigger_deadzone')), 0.0), 0.9)
        self.trig_expo     = max(float(p('trigger_expo')), 0.1)
        self.steer_dz      = min(max(float(p('steer_deadzone')), 0.0), 0.9)
        self.arm_required  = p('arm_on_neutral')
        self.debug         = p('debug')
        self.max_speed     = p('max_speed')
        self.max_steer     = p('max_steer')

        # 트리거 양 끝점: 안 누름(중립) → 끝까지 당김
        self.trig_neutral = 1.0 if self.inv_trigger else -1.0
        self.trig_full    = -self.trig_neutral

        self.auto_mode  = False   # 시작은 수동 모드
        self.prev_lb    = False   # LB 이전 상태 (엣지 감지용)
        self.armed      = not self.arm_required   # 트리거 중립 확인 여부
        self._last_log  = 0.0
        self._warned    = False

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.pub         = self.create_publisher(AckermannDriveStamped, '/teleop', 10)
        self.pub_enabled = self.create_publisher(Bool, '/race/enabled', 10)

        self.get_logger().info(
            f'JoystickTeleop ready  [현재: 수동 모드]\n'
            f'  LB(버튼{self.btn_tog}) → 수동↔자율 토글\n'
            f'  RT(축{self.ax_speed}) → 전진 속도 (선형, 놓으면 즉시 정지)\n'
            f'  L스틱(축{self.ax_steer}) → 조향\n'
            f'  RB(버튼{self.btn_rev}) 누른 채 RT → 후진\n'
            f'  트리거 매핑: {self.trig_neutral:+.0f}(안 누름) → {self.trig_full:+.0f}(끝까지)'
            f'  = 0 → {self.max_speed} m/s'
            f'  [deadzone {self.trig_dz}, expo {self.trig_expo}]')

    # ── 트리거 raw 값 → 0.0~1.0 선형 스로틀 ────────────────────
    def _throttle(self, raw: float) -> float:
        u = (raw - self.trig_neutral) / (self.trig_full - self.trig_neutral)
        u = min(max(u, 0.0), 1.0)
        if u <= self.trig_dz:
            return 0.0
        u = (u - self.trig_dz) / (1.0 - self.trig_dz)   # 데드존 제거 후 재정규화
        return u ** self.trig_expo

    # ── 스틱 raw 값 → -1.0~1.0 (중립 유격 제거) ────────────────
    def _stick(self, raw: float) -> float:
        mag = abs(raw)
        if mag <= self.steer_dz:
            return 0.0
        mag = (mag - self.steer_dz) / (1.0 - self.steer_dz)
        return min(mag, 1.0) * (1.0 if raw > 0 else -1.0)

    def _joy_cb(self, msg: Joy):
        # ── 모드 토글 (LB 상승 엣지) ──────────────────────────────
        lb = len(msg.buttons) > self.btn_tog and bool(msg.buttons[self.btn_tog])
        if lb and not self.prev_lb:
            self.auto_mode = not self.auto_mode
            label = '자율' if self.auto_mode else '수동'
            self.get_logger().info(f'모드 전환 → [{label}]')
            enabled = Bool()
            enabled.data = self.auto_mode
            self.pub_enabled.publish(enabled)
            if not self.auto_mode:
                self._publish(0.0, 0.0)  # 수동 전환 시 즉시 정지
        self.prev_lb = lb

        # 자율 모드: /teleop 발행 안 함 → mux가 자율주행 채널로 전환
        if self.auto_mode:
            return

        # ── 수동 조종 ─────────────────────────────────────────────
        raw_trig = (msg.axes[self.ax_speed] if len(msg.axes) > self.ax_speed
                    else self.trig_neutral)
        raw_steer = msg.axes[self.ax_steer] if len(msg.axes) > self.ax_steer else 0.0
        reverse   = len(msg.buttons) > self.btn_rev and bool(msg.buttons[self.btn_rev])

        # 안전장치: 트리거가 중립 근처에 있는 걸 한 번 봐야 스로틀을 푼다.
        # trigger_invert가 거꾸로면 중립을 영영 못 보므로 차가 튀어나가지 않는다.
        if not self.armed:
            if abs(raw_trig - self.trig_neutral) <= 0.15:
                self.armed = True
                self.get_logger().info('트리거 중립 확인 — 스로틀 활성화')
            else:
                if not self._warned:
                    self._warned = True
                    self.get_logger().warn(
                        f'트리거 중립({self.trig_neutral:+.0f}) 대기 중 — 현재 {raw_trig:+.2f}. '
                        f'RT에서 손을 떼세요. 계속 이 경고가 뜨면 '
                        f'trigger_invert 설정이 반대입니다 (현재 {self.inv_trigger}).')
                self._publish(0.0, 0.0)
                return

        u     = self._throttle(raw_trig)          # 0.0 ~ 1.0 선형
        speed = u * self.max_speed
        if reverse:
            speed = -speed

        steer = self._stick(raw_steer)
        if self.inv_steer:
            steer = -steer

        steer = float(max(-self.max_steer, min(self.max_steer, steer * self.max_steer)))
        speed = float(max(-self.max_speed, min(self.max_speed, speed)))

        if self.debug:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_log > 0.5:
                self._last_log = now
                self.get_logger().info(
                    f'RT raw {raw_trig:+.3f} → 스로틀 {u*100:5.1f}% → {speed:+.2f} m/s '
                    f'| 조향 {steer:+.3f} rad')

        self._publish(steer, speed)

    def _publish(self, steer: float, speed: float):
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steer
        msg.drive.speed          = speed
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
