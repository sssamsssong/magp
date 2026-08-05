#!/usr/bin/env python3
"""slip_logger — 타이어 슬립 데이터를 수동 주행으로 모은다.

★ 왜 수동 주행인가
  슬립을 보려면 횡가속도가 마찰한계에 가까워야 하고, 우리 트랙 최급코너
  (κ 1.14) 기준으로 그건 **2.0 m/s 부터**다 (a_y 4.6 m/s² = μ0.7 한계의 66%).
  그런데 그 속도로 자율주행하려면 컨트롤러·위치추정·레이싱라인이 먼저 멀쩡해야
  한다. 지금은 셋 다 미완이다.

  하지만 슬립 데이터 수집 자체는 자율주행이 필요 없다. 정상원·스텝 스티어·
  사인 스윕은 전부 조이스틱으로 낼 수 있고, 필요한 건 IMU + 휠 오도메트리 +
  서보 명령뿐이다. AMCL 도 Stanley 도 레이싱라인도 안 쓴다. 그래서 지금 막혀
  있는 것들과 **완전히 분리해서** 진행할 수 있다.

★ 무엇을 주된 지표로 삼는가 — β 적분이 아니라 언더스티어 구배
  요청받은 관측기는 β̇ = a_y/v_x − ψ̇ 를 적분하는 방식이다. 수식은 맞지만
  우리 하드웨어에서 적분은 불리하다:
    · IMU 가 50 Hz 다 (vesc_driver.cpp:107 의 20ms 폴링 타이머 때문이고,
      센서 한계가 아니다)
    · 타임스탬프가 샘플 시각이 아니라 **패킷 파싱 시각**이다
      (vesc_driver.cpp:211 의 `now()`). 지터가 그대로 적분에 쌓인다

  그래서 **적분이 필요 없는** 정상상태 지표를 주된 산출물로 삼는다.
  정상선회에서 조향각은 기구학 항과 타이어 항의 합이다:

        δ  =  L·ψ̇/v  +  K_us · a_y
              └ 기구학 ┘   └ 타이어가 먹는 몫 ┘

  즉 (δ − L·ψ̇/v) 를 a_y 로 회귀하면 기울기가 **언더스티어 구배 K_us** 다.
  각 표본이 독립이라 드리프트가 없다. β 적분도 함께 기록하지만 참고용이다.

★ 축·단위·바이어스 (2026-08-02 imu_axis_check.py 실측)
      차량 전방 = IMU -y,  차량 좌측 = IMU +x,  위 = +z
      자이로 deg/s,  가속도 g   ← REP-145 아님. 드라이버가 변환 없이 대입한다
  가속도까지 쓰는 건 이 노드가 처음이라, x/y 스왑이 여기서 처음으로 문제가 된다
  (yaw 만 쓰던 imu_odom 은 회전축이 z 라 스왑이 무해했다).

★ 남은 오차 요인 (알고 쓸 것)
  · 레버암: `lever_x` 를 재서 넣지 않으면 a_y 에 ψ̈·r 항이 남는다. 기본 0.
  · 모터 진동: 1.5 m/s 에서 모터 회전이 이미 50 Hz 대라 **샘플링 주파수보다
    높다**. 즉 에일리어싱이라 저역통과로 못 지운다. IMU 를 100 Hz 로 올리는
    것이 근본 대책이고, 그 전까지 고주파 성분은 신뢰하지 말 것.

사용법
──────
    ./race.sh slip                      # 조이스틱으로 몰면서 기록
    ./race.sh slip --name circle_2ms    # 기동 이름을 붙여 저장

  기동 순서 (문서 권장):
    1) 정지 3초  — 바이어스 자동 측정 (로그에 "바이어스 확정" 이 떠야 한다)
    2) 정상원    — 조향을 고정하고 속도를 단계적으로 올린다. 이게 K_us 의 주재료다
    3) 스텝 스티어 / 사인 스윕 / 급가감속 중 조향
"""
import csv
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

_REPO = (os.path.expanduser(os.environ['RACE_WS']) if os.environ.get('RACE_WS')
         else os.path.dirname(os.path.dirname(os.path.dirname(
             os.path.dirname(os.path.abspath(__file__))))))


class SlipLogger(Node):
    def __init__(self):
        super().__init__('slip_logger')
        d = self.declare_parameter
        d('imu_topic', '/sensors/imu/raw')
        d('odom_topic', '/odom')          # 병진 속도는 휠 오도메트리가 정확하다
        d('servo_topic', '/commands/servo/position')
        d('name', '')
        d('wheelbase', 0.33)
        # 서보 → 조향각 (config/vesc.yaml 과 같은 값이어야 한다)
        d('servo_gain', -0.761)
        d('servo_offset', 0.425)
        # IMU 원시 단위
        d('gyro_units', 'deg')
        d('accel_units', 'g')
        # 레버암: IMU 가 무게중심보다 앞이면 +x (m). 재서 넣을 것.
        d('lever_x', 0.0)
        d('lpf_hz', 8.0)
        # 정상상태 판정 — 이 안에서 흔들리지 않고 hold_sec 유지되면 채택
        d('ss_hold_sec', 0.6)
        d('ss_v_tol', 0.10)        # m/s
        d('ss_delta_tol', 0.02)    # rad
        d('ss_wz_tol', 0.15)       # rad/s
        d('ss_min_ay', 1.0)        # 이보다 작은 횡가속도는 타이어 정보가 없다
        d('min_speed', 0.5)

        p = lambda n: self.get_parameter(n).value
        self.L = float(p('wheelbase'))
        self.sg, self.so = float(p('servo_gain')), float(p('servo_offset'))
        self.to_rad = (math.pi / 180.0
                       if str(p('gyro_units')).lower().startswith('deg') else 1.0)
        self.to_ms2 = 9.80665 if str(p('accel_units')).lower() == 'g' else 1.0
        self.lever_x = float(p('lever_x'))
        self.lpf_hz = float(p('lpf_hz'))
        self.min_speed = float(p('min_speed'))

        self.v = 0.0
        self.delta = 0.0
        self.t0 = None
        self.last_t = None
        self.wz_f = 0.0
        self.ay_f = 0.0
        self.ax_f = 0.0
        self.wz_prev = 0.0
        self.beta = 0.0

        # 바이어스 (정지 시 측정)
        self.bias = {'wz': 0.0, 'ax': 0.0, 'ay': 0.0}
        self.bias_ready = False
        self._warm = []
        self._warm_t0 = None

        self.rows = []
        self.ss = []              # 정상상태 표본 (delta, ay, v, wz)
        self._ss_buf = []
        self._last_print = 0.0

        self.create_subscription(Odometry, p('odom_topic'), self._odom_cb, 20)
        self.create_subscription(Float64, p('servo_topic'), self._servo_cb, 20)
        self.create_subscription(Imu, p('imu_topic'), self._imu_cb,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            '슬립 로거 시작 — 먼저 3초간 **정지**해 주세요 (바이어스 측정).\n'
            '  그다음 조이스틱으로: 정상원(조향 고정 + 속도 단계적 상승) → '
            '스텝 스티어 → 사인 스윕')

    # ── 입력 ───────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.v = float(msg.twist.twist.linear.x)

    def _servo_cb(self, msg: Float64):
        # servo = gain·δ + offset  →  δ = (servo - offset)/gain
        self.delta = (float(msg.data) - self.so) / self.sg

    def _imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        dt = (t - self.last_t) if self.last_t else 0.02
        self.last_t = t
        if not (1e-4 < dt < 0.5):
            dt = 0.02

        # ── 축 매핑: IMU → 차량 ────────────────────────────────────────
        # 전방(+x_car) = -y_imu,  좌측(+y_car) = +x_imu,  위 = +z
        ax = -msg.linear_acceleration.y * self.to_ms2
        ay = +msg.linear_acceleration.x * self.to_ms2
        wz = +msg.angular_velocity.z * self.to_rad

        # ── 바이어스: 정지 상태에서만 측정 ─────────────────────────────
        still = abs(self.v) < 0.05 and abs(wz - self.bias['wz']) < math.radians(2.0)
        if still and not self.bias_ready:
            if self._warm_t0 is None:
                self._warm_t0 = t
            self._warm.append((wz, ax, ay))
            if t - self._warm_t0 >= 3.0 and len(self._warm) > 40:
                a = np.asarray(self._warm)
                self.bias = {'wz': float(a[:, 0].mean()),
                             'ax': float(a[:, 1].mean()),
                             'ay': float(a[:, 2].mean())}
                self.bias_ready = True
                self.get_logger().info(
                    f'바이어스 확정 ({len(self._warm)}샘플) — '
                    f'ψ̇ {math.degrees(self.bias["wz"]):+.3f}°/s, '
                    f'a_x {self.bias["ax"]:+.3f}, a_y {self.bias["ay"]:+.3f} ㎧². '
                    '이제 기동을 시작하세요')
        if not self.bias_ready:
            return
        wz -= self.bias['wz']; ax -= self.bias['ax']; ay -= self.bias['ay']

        # ── 저역통과 (1차) ─────────────────────────────────────────────
        # ※ 모터 진동은 샘플링 주파수보다 높아 에일리어싱된다 — 이 필터로는
        #   못 지운다. 여기서 지우는 건 노면 요철 정도다.
        a_lpf = dt / (dt + 1.0 / (2 * math.pi * max(self.lpf_hz, 0.1)))
        self.wz_f += a_lpf * (wz - self.wz_f)
        self.ay_f += a_lpf * (ay - self.ay_f)
        self.ax_f += a_lpf * (ax - self.ax_f)

        # ── 레버암 보정: a_y_cg = a_y − ψ̈·r_x ──────────────────────────
        wz_dot = (self.wz_f - self.wz_prev) / dt
        self.wz_prev = self.wz_f
        ay_cg = self.ay_f - wz_dot * self.lever_x

        v = self.v
        # 기구학(무슬립) 예측 요레이트
        wz_kin = v * math.tan(self.delta) / self.L
        # β 적분 — 참고용. 직선에서 리셋해 드리프트를 끊는다.
        if abs(v) > self.min_speed:
            self.beta += (ay_cg / v - self.wz_f) * dt
        if abs(self.delta) < 0.02 and abs(self.wz_f) < 0.05:
            self.beta = 0.0
        self.beta = float(np.clip(self.beta, -0.5, 0.5))

        steady = self._steady(t, v, self.delta, self.wz_f, ay_cg)
        self.rows.append((t - self.t0, v, self.delta, self.wz_f, wz_kin,
                          self.ay_f, ay_cg, self.ax_f, self.beta, int(steady)))

    # ── 정상상태 판정 ──────────────────────────────────────────────────
    def _steady(self, t, v, delta, wz, ay):
        """속도·조향·요레이트가 모두 안정된 구간만 K_us 회귀에 쓴다.

        과도상태에서는 δ = L·ψ̇/v + K_us·a_y 가 성립하지 않는다 (관성 항이 빠져
        있다). 정상원 기동이 중요한 이유가 이것이다.
        """
        p = lambda n: self.get_parameter(n).value
        if abs(v) < self.min_speed or abs(ay) < float(p('ss_min_ay')):
            self._ss_buf.clear()
            return False
        self._ss_buf.append((t, v, delta, wz, ay))
        while self._ss_buf and t - self._ss_buf[0][0] > float(p('ss_hold_sec')):
            self._ss_buf.pop(0)
        if t - self._ss_buf[0][0] < float(p('ss_hold_sec')) * 0.9:
            return False
        a = np.asarray(self._ss_buf)
        if (a[:, 1].ptp() > float(p('ss_v_tol'))
                or a[:, 2].ptp() > float(p('ss_delta_tol'))
                or a[:, 3].ptp() > float(p('ss_wz_tol'))):
            return False
        self.ss.append((float(a[:, 2].mean()), float(a[:, 4].mean()),
                        float(a[:, 1].mean()), float(a[:, 3].mean())))
        self._ss_buf.clear()
        return True

    # ── 주행 중 실시간 피드백 ──────────────────────────────────────────
    def _tick(self):
        if not self.bias_ready or not self.rows:
            return
        _, v, delta, wz, wz_kin, ay, ay_cg, ax, beta, _ = self.rows[-1]
        ratio = (wz / wz_kin) if abs(wz_kin) > 0.05 else float('nan')
        # ψ̇/ψ̇_kin 이 1 보다 작으면 명령한 만큼 안 돈다 = 언더스티어(전륜 슬립)
        self.get_logger().info(
            f'v {v:4.2f}㎧  δ {math.degrees(delta):+6.2f}°  '
            f'a_y {ay_cg:+5.2f}㎧²  ψ̇ {math.degrees(wz):+6.1f}°/s  '
            f'실제/기구학 {ratio:5.2f}  β {math.degrees(beta):+5.1f}°  '
            f'정상상태 {len(self.ss)}점')

    # ── 저장 + 분석 ────────────────────────────────────────────────────
    def save(self):
        if not self.rows:
            self.get_logger().warn('기록된 표본이 없습니다')
            return
        name = str(self.get_parameter('name').value) or 'slip'
        ts = time.strftime('%Y%m%d_%H%M%S')
        run = os.path.join(_REPO, 'logs', f'{ts}_{name}')
        os.makedirs(run, exist_ok=True)
        path = os.path.join(run, 'slip.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'v', 'delta', 'wz', 'wz_kin', 'ay', 'ay_cg',
                        'ax', 'beta', 'steady'])
            for r in self.rows:
                w.writerow([f'{r[0]:.4f}'] + [f'{x:.5f}' for x in r[1:9]] + [r[9]])
        self.get_logger().info(f'{len(self.rows)}표본 → {path}')
        self._fit(run)

    def _fit(self, run):
        """정상상태 표본으로 언더스티어 구배 K_us 를 구한다.

            δ = L·ψ̇/v + K_us·a_y   →   (δ − L·ψ̇/v) 를 a_y 로 회귀
        """
        if len(self.ss) < 8:
            self.get_logger().warn(
                f'정상상태 표본 {len(self.ss)}점 — K_us 회귀에는 8점 이상 필요합니다. '
                '조향을 고정한 채 원을 그리며 속도를 단계적으로 올려 보세요')
            return
        a = np.asarray(self.ss)
        delta, ay, v, wz = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
        resid = delta - self.L * wz / np.maximum(np.abs(v), 1e-3) * np.sign(v)
        k, b = np.polyfit(ay, resid, 1)
        pred = k * ay + b
        r2 = 1.0 - ((resid - pred) ** 2).sum() / max(((resid - resid.mean()) ** 2).sum(), 1e-12)
        self.get_logger().info(
            f'\n── 언더스티어 구배 (정상상태 {len(a)}점) ──\n'
            f'   K_us = {k:.5f} rad/(㎧²)  = {math.degrees(k):.3f} °/(㎧²)   R² {r2:.3f}\n'
            f'   a_y 범위 {ay.min():+.2f} ~ {ay.max():+.2f} ㎧²,  '
            f'v 범위 {v.min():.2f} ~ {v.max():.2f} ㎧\n'
            f'   해석: {"언더스티어(전륜이 먼저 미끄러짐)" if k > 0 else "오버스티어"}. '
            f'a_y {ay.max():+.1f}㎧² 에서 기구학보다 조향을 '
            f'{math.degrees(k*ay.max()):+.1f}° 더 줘야 한다\n'
            f'   ※ a_y 범위가 좁으면(<3㎧²) 곡선의 선형 구간만 본 것이라 '
            f'한계 거동은 아직 모른다')
        with open(os.path.join(run, 'steady_state.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['delta', 'ay', 'v', 'wz', 'delta_minus_kinematic'])
            for i in range(len(a)):
                w.writerow([f'{delta[i]:.5f}', f'{ay[i]:.4f}', f'{v[i]:.4f}',
                            f'{wz[i]:.5f}', f'{resid[i]:.5f}'])


def main():
    rclpy.init()
    node = SlipLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
