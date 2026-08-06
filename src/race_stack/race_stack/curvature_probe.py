#!/usr/bin/env python3
"""curvature_probe.py — 속도별 "풀조향 달성곡률" 캘리브레이션 리그.

★ slip_logger.py(언더스티어 구배 K_us 회귀)와 완전히 별개의 도구다.
  그 파일과 race_script.yaml 의 slip: 섹션은 건드리지 않는다 — 다른 팀원이
  그 리그를 계속 쓴다. 여기는 새 토픽 이름(/curvprobe/...), 새 설정 섹션
  (curv_probe:), 새 로그 폴더(logs/curv_tests/)를 따로 쓴다.

★ 목적
  gen_waypoints_kappa.py 는 조향각 기하한계만으로 KAPPA_MAX 를 잡는다
  (무슬립 가정). 실제로는 그 곡률을 낼 만큼 빠르면 타이어가 미끄러져
  계획보다 넓게 밀려난다(2026-08-06 q1_max=2 실주행에서 실측). 그래서
  "이 곡률에서 슬립 없이 낼 수 있는 최대속도" v_max(κ) 를 실측하려 한다.

★ 왜 반경을 여러 개 안 재고 속도만 스윕하는가
  조향을 풀락(좌 또는 우 끝까지)으로 고정하고 속도만 올리면서, 매 순간
  IMU 요레이트(ψ̇, 실측)와 휠 오도메트리 속도(v)로
      달성곡률 κ_achieved = ψ̇ / v
  를 직접 계산한다. 속도 v 에서 풀락(최대 요구 a_y)을 슬립 없이 버텼다면,
  그보다 작은 곡률(더 작은 a_y=v²κ 요구)은 같은 속도에서 당연히 더 여유
  있게 버틴다. 그래서 "속도 → 풀락 달성곡률" 곡선 하나만 실측하면, 뒤집어서
  전체 곡률 범위(0~KAPPA_MAX)의 v_max(κ) 를 얻을 수 있다 — 반경×속도 그리드를
  다 채울 필요가 없다.

★ 조작 (조이스틱, F710)
  B   시작/종료 — 시작 시 반드시 정지 상태 (3초 바이어스 자동측정)
  풀조향: L스틱을 좌 또는 우로 끝까지 밀고 그대로 고정, 그 상태로 속도만
          서서히 올린다 (0.3 m/s² 이하 권장 — 빠르면 정상상태 판정에서 걸러짐)
  좌/우는 그때그때 원하는 대로 — 방향 선택 UI 없음. 기록된 부호(κ>0=좌)로
  분석 때 자동 분리된다.

★ 사용법
  ./race.sh curvprobe                                  # 실차 리그
  ros2 run race_stack curvature_probe -- --analyze      # 수집 데이터 분석만
    (ROS 노드 안 띄우고 logs/curv_tests/*.csv 를 모아 v_max(κ) 표를 뽑는다)
"""

import argparse
import csv
import math
import os
import sys
import time
import unicodedata

import numpy as np

from race_stack.paths import repo_root

_REPO = repo_root()
_OUT = os.path.join(_REPO, 'logs', 'curv_tests')

BOX_W = 64


def _w(s: str) -> int:
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in s)


def _row(text: str) -> str:
    while _w(text) > BOX_W:
        text = text[:-1]
    return '║' + text + ' ' * (BOX_W - _w(text)) + '║'


# ══════════════════════════════════════════════════════════════════════════
# 오프라인 분석 (ROS 불필요) — 여러 세션 CSV를 모아 v_max(κ) 계산
# ══════════════════════════════════════════════════════════════════════════
def _load_steady_samples():
    """logs/curv_tests/*.csv 전부에서 steady==1 행만 모은다.
    반환: [(v, kappa_achieved), ...]"""
    out = []
    if not os.path.isdir(_OUT):
        return out
    for fn in sorted(os.listdir(_OUT)):
        if not fn.endswith('.csv'):
            continue
        path = os.path.join(_OUT, fn)
        try:
            with open(path) as f:
                rd = csv.reader(f)
                next(rd)  # 헤더 주석행
                header = next(rd)
                iv = header.index('v')
                ik = header.index('kappa')
                ist = header.index('steady')
                for row in rd:
                    if len(row) <= max(iv, ik, ist) or row[ist] != '1':
                        continue
                    out.append((float(row[iv]), float(row[ik])))
        except Exception:
            continue
    return out


def analyze(kappa_max: float = 1.1406, speed_bin: float = 0.25,
            grid_step: float = 0.1, save: bool = True):
    """수집된 데이터를 (방향, 속도구간)별로 묶어 median κ_achieved 를 구하고,
    그걸 뒤집어 v_max(κ) 표를 만든다. 콘솔에 출력하고, save 면 CSV로도 저장."""
    samples = _load_steady_samples()
    print(f'steady 샘플: {len(samples)}개')
    if len(samples) < 5:
        print('표본이 너무 적음 (5개 미만) — 리그를 더 돌려야 함')
        return

    for direction, sign in (('좌회전(κ>0)', 1), ('우회전(κ<0)', -1)):
        pts = [(v, abs(k)) for v, k in samples if (k > 0) == (sign > 0) and v > 0.2]
        print(f'\n=== {direction} — {len(pts)}개 표본 ===')
        if len(pts) < 5:
            print('  표본 부족 — 이 방향은 더 재야 함')
            continue

        # 속도 구간별 median achieved kappa
        vs = np.array([p[0] for p in pts])
        ks = np.array([p[1] for p in pts])
        vmin, vmax = vs.min(), vs.max()
        bins = np.arange(math.floor(vmin / speed_bin) * speed_bin,
                          vmax + speed_bin, speed_bin)
        curve_v, curve_k = [], []
        print('  속도구간(m/s)   n     median|κ_achieved|   κ_max대비')
        for b in bins:
            mask = (vs >= b) & (vs < b + speed_bin)
            if mask.sum() < 2:
                continue
            med = float(np.median(ks[mask]))
            curve_v.append(b + speed_bin / 2)
            curve_k.append(med)
            print(f'    {b:4.2f}~{b+speed_bin:4.2f}   {mask.sum():3d}   '
                  f'{med:6.3f} 1/m         {med/kappa_max*100:5.1f}%')

        if len(curve_v) < 2:
            print('  구간이 2개 미만이라 v_max(κ) 역산 불가')
            continue

        # 속도 오름차순 정렬, kappa_achieved 는 보통 내림차순(슬립 시작하면 감소)
        order = np.argsort(curve_v)
        cv = np.array(curve_v)[order]
        ck = np.array(curve_k)[order]

        # v_max(κ): 목표 κ 이상을 낼 수 있는 가장 큰 속도를 선형보간으로 역산
        kappa_grid = np.arange(grid_step, min(kappa_max, ck.max()) + 1e-6, grid_step)
        print('\n  → v_max(κ) 역산 (선형보간):')
        rows_out = []
        for kg in kappa_grid:
            # ck 가 v 증가에 따라 대체로 감소한다고 가정하고, ck>=kg 를 만족하는
            # 마지막 v(가장 큰 v)를 찾는다.
            ok = np.where(ck >= kg)[0]
            if len(ok) == 0:
                continue
            i_last = ok[-1]
            if i_last + 1 < len(cv) and ck[i_last + 1] < kg:
                # 그 다음 구간과 보간
                v0, v1 = cv[i_last], cv[i_last + 1]
                k0, k1 = ck[i_last], ck[i_last + 1]
                v_max = v0 + (kg - k0) * (v1 - v0) / (k1 - k0)
            else:
                v_max = cv[i_last]
            print(f'    κ={kg:.2f}  v_max≈{v_max:.2f} m/s')
            rows_out.append((sign, kg, v_max))

        if save and rows_out:
            os.makedirs(_OUT, exist_ok=True)
            out_path = os.path.join(_OUT, 'v_max_by_kappa.csv')
            mode = 'a' if os.path.exists(out_path) else 'w'
            with open(out_path, mode, newline='') as f:
                w = csv.writer(f)
                if mode == 'w':
                    w.writerow(['direction_sign', 'kappa', 'v_max',
                                'generated_at'])
                ts = time.strftime('%Y-%m-%d %H:%M:%S')
                for s, kg, vmax in rows_out:
                    w.writerow([s, f'{kg:.3f}', f'{vmax:.3f}', ts])
            print(f'\n  저장: {out_path}')


# ══════════════════════════════════════════════════════════════════════════
# ROS 노드 (실차 수집)
# ══════════════════════════════════════════════════════════════════════════
def _run_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, Joy
    from std_msgs.msg import Float64

    IDLE, BIAS, REC = 0, 1, 2
    STATE_NAME = {IDLE: '대기', BIAS: '바이어스 측정', REC: '측정중'}

    class CurvatureProbe(Node):
        def __init__(self):
            super().__init__('curvature_probe')
            d = self.declare_parameter
            d('imu_topic', '/sensors/imu/raw')
            d('odom_topic', '/odom')
            d('servo_topic', '/commands/servo/position')
            d('joy_topic', '/joy')
            d('steer_limit_topic', '/curvprobe/steer_limit')
            d('speed_limit_topic', '/curvprobe/speed_limit')
            d('wheelbase', 0.33)
            d('servo_gain', -0.761)
            d('servo_offset', 0.425)
            d('gyro_units', 'deg')
            d('accel_units', 'g')
            d('lpf_hz', 8.0)
            d('kappa_max', 1.1406)
            d('mu', 0.7)
            d('max_speed', 3.5)
            d('full_lock_steer', 0.65)   # 실제 기구 락(0.361/0.558)보다 확실히 큼
            d('btn_toggle', 1)           # F710 B
            d('bias_sec', 3.0)
            d('ss_hold_sec', 0.5)
            d('ss_v_tol', 0.15)
            d('ss_delta_tol', 0.035)
            d('ss_wz_tol', 0.20)
            d('min_speed', 0.3)

            p = lambda n: self.get_parameter(n).value
            self.L = float(p('wheelbase'))
            self.sg, self.so = float(p('servo_gain')), float(p('servo_offset'))
            self.to_rad = (math.pi / 180.0
                           if str(p('gyro_units')).lower().startswith('deg') else 1.0)
            self.to_ms2 = 9.80665 if str(p('accel_units')).lower() == 'g' else 1.0
            self.lpf_hz = float(p('lpf_hz'))
            self.kappa_max = float(p('kappa_max'))
            self.mu = float(p('mu'))
            self.max_speed = float(p('max_speed'))
            self.full_lock = float(p('full_lock_steer'))
            self.bias_sec = float(p('bias_sec'))
            self.min_speed = float(p('min_speed'))

            self.state = IDLE
            self.t_state = 0.0
            self.v = 0.0
            self.delta = 0.0
            self.last_t = None
            self.wz_f = 0.0
            self.bias_wz = 0.0
            self._warm = []
            self.rows = []
            self._ss_buf = []
            self._prev_btn = False
            self._msg = ''
            self._seen = {'joy': 0.0, 'odom': 0.0, 'imu': 0.0}
            self._live_kappa = 0.0
            self._n_saved_sessions = 0

            os.makedirs(_OUT, exist_ok=True)

            self.lim_pub = self.create_publisher(Float64, p('steer_limit_topic'), 10)
            self.spd_pub = self.create_publisher(Float64, p('speed_limit_topic'), 10)
            self.create_timer(0.2, self._pub_limit)
            self.create_subscription(Odometry, p('odom_topic'), self._odom_cb, 20)
            self.create_subscription(Float64, p('servo_topic'), self._servo_cb, 20)
            self.create_subscription(Joy, p('joy_topic'), self._joy_cb, 10)
            self.create_subscription(Imu, p('imu_topic'), self._imu_cb,
                                     qos_profile_sensor_data)
            self.create_timer(0.3, self._draw)

            self.get_logger().info(
                f'CurvatureProbe ready — B로 시작/종료, 풀조향 후 속도 램프. '
                f'max_speed={self.max_speed} full_lock={math.degrees(self.full_lock):.1f}°')

        # ── 상한 발행 (joystick_teleop 이 구독) ──────────────────────────
        def _pub_limit(self):
            self.lim_pub.publish(Float64(data=self.full_lock))
            self.spd_pub.publish(Float64(data=self.max_speed))

        # ── 입력 ──────────────────────────────────────────────────────
        def _odom_cb(self, msg: Odometry):
            self._seen['odom'] = time.time()
            self.v = float(msg.twist.twist.linear.x)

        def _servo_cb(self, msg: Float64):
            self.delta = (float(msg.data) - self.so) / self.sg

        def _joy_cb(self, msg: Joy):
            self._seen['joy'] = time.time()
            bt = int(self.get_parameter('btn_toggle').value)
            b = bool(msg.buttons[bt]) if len(msg.buttons) > bt else False
            if b and not self._prev_btn:
                if self.state == IDLE:
                    if abs(self.v) > 0.1:
                        self._msg = '⚠ 정지 상태에서 시작하세요 (바이어스 측정 때문)'
                    else:
                        self.rows.clear(); self._ss_buf.clear(); self._warm.clear()
                        self.state = BIAS
                        self.t_state = time.time()
                        self._msg = ''
                else:
                    self._finish()
            self._prev_btn = b

        # ── IMU 처리 ──────────────────────────────────────────────────
        def _imu_cb(self, msg: Imu):
            self._seen['imu'] = time.time()
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t <= 0.0:
                t = self.get_clock().now().nanoseconds * 1e-9
            dt = (t - self.last_t) if self.last_t else 0.02
            self.last_t = t
            if not (1e-4 < dt < 0.5):
                dt = 0.02

            # 축 매핑: 전방(+x_car)=-y_imu, 좌측(+y_car)=+x_imu, 위=+z
            # (slip_logger.py 와 동일 — imu_axis_check.py 실측 기준)
            wz = +msg.angular_velocity.z * self.to_rad

            if self.state == BIAS:
                self._warm.append(wz)
                if time.time() - self.t_state >= self.bias_sec:
                    self.bias_wz = float(np.mean(self._warm))
                    self.state = REC
                    self.t_state = time.time()
                    self._msg = '바이어스 확정 — 풀조향 후 출발하세요!'
                return
            if self.state != REC:
                return

            wz -= self.bias_wz
            a_lpf = dt / (dt + 1.0 / (2 * math.pi * max(self.lpf_hz, 0.1)))
            self.wz_f += a_lpf * (wz - self.wz_f)

            v = self.v
            wz_kin = v * math.tan(self.delta) / self.L
            kappa = self.wz_f / v if abs(v) > self.min_speed else 0.0
            self._live_kappa = kappa

            steady = self._steady(t, v, self.delta, self.wz_f)
            self.rows.append((t, v, self.delta, self.wz_f, wz_kin, kappa, int(steady)))

        def _steady(self, t, v, delta, wz):
            if abs(v) < self.min_speed:
                self._ss_buf.clear(); return False
            self._ss_buf.append((t, v, delta, wz))
            hold = self.ss_hold_sec = float(self.get_parameter('ss_hold_sec').value)
            while self._ss_buf and t - self._ss_buf[0][0] > hold:
                self._ss_buf.pop(0)
            if not self._ss_buf or t - self._ss_buf[0][0] < hold * 0.9:
                return False
            a = np.asarray(self._ss_buf)
            v_tol = float(self.get_parameter('ss_v_tol').value)
            d_tol = float(self.get_parameter('ss_delta_tol').value)
            w_tol = float(self.get_parameter('ss_wz_tol').value)
            if a[:, 1].ptp() > v_tol or a[:, 2].ptp() > d_tol or a[:, 3].ptp() > w_tol:
                return False
            return True

        # ── 종료 → 저장 ───────────────────────────────────────────────
        def _finish(self):
            self.state = IDLE
            if len(self.rows) < 20:
                self._msg = '표본이 너무 적어 버립니다 (20개 미만)'
                return
            ts = time.strftime('%Y%m%d_%H%M%S')
            path = os.path.join(_OUT, f'{ts}_curvprobe.csv')
            t0 = self.rows[0][0]
            n_steady = sum(r[6] for r in self.rows)
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow([f'# curvature_probe', f'rows={len(self.rows)}',
                            f'steady={n_steady}'])
                w.writerow(['t', 'v', 'delta', 'wz', 'wz_kin', 'kappa', 'steady'])
                for row in self.rows:
                    w.writerow([f'{row[0]-t0:.4f}'] + [f'{x:.5f}' for x in row[1:6]]
                               + [row[6]])
            self._n_saved_sessions += 1
            self._msg = f'저장: {os.path.basename(path)} — steady {n_steady}점'

        # ── 화면 ──────────────────────────────────────────────────────
        def _draw(self):
            now = time.time()
            bar = '═' * BOX_W
            L = ['\033[2J\033[H', '╔' + bar + '╗']
            A = L.append
            A(_row(f'  곡률-속도 캘리브레이션{" "*24}[{STATE_NAME[self.state]:^10}]'))
            A('╠' + bar + '╣')
            A(_row(f'  속도 v      {self.v:6.2f} m/s'))
            A(_row(f'  조향각 δ    {math.degrees(self.delta):6.1f}°'))
            A(_row(f'  달성곡률 κ  {self._live_kappa:+6.3f} 1/m '
                   f'(κ_max={self.kappa_max:.3f})'))
            if abs(self.v) > 0.3:
                pct = abs(self._live_kappa) / self.kappa_max * 100
                A(_row(f'  κ_max 대비  {pct:5.1f}%'
                       f'{"  ⚠ 슬립 의심" if pct < 70 and abs(self.v) > 1.5 else ""}'))
            A('╠' + bar + '╣')
            for k, label in (('joy', 'joy'), ('odom', 'odom'), ('imu', 'imu')):
                age = now - self._seen[k]
                ok = age < 1.0
                A(_row(f'  {label:5s}  {"OK" if ok else f"{age:.1f}s 전"}'))
            A('╠' + bar + '╣')
            A(_row('  B: 시작/종료  |  풀조향 유지, 속도만 서서히'))
            if self._msg:
                A(_row(f'  {self._msg}'))
            A(_row(f'  세션 저장: {self._n_saved_sessions}회  '
                   f'(logs/curv_tests/, 분석: --analyze)'))
            A('╚' + bar + '╝')
            print('\n'.join(L))

    rclpy.init()
    node = CurvatureProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    if '--analyze' in sys.argv:
        p = argparse.ArgumentParser()
        p.add_argument('--analyze', action='store_true')
        p.add_argument('--kappa-max', type=float, default=1.1406)
        args = p.parse_args()
        analyze(kappa_max=args.kappa_max)
        return
    _run_node()


if __name__ == '__main__':
    main()
