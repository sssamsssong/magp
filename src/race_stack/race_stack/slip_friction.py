#!/usr/bin/env python3
"""slip_friction — LiDAR+IMU 실시간 슬립 판독 & 마찰계수 추정.

참고: Oeltjen et al. "Online Slip Detection and Friction Coefficient
      Estimation for Autonomous Racing" (arXiv:2509.15423)

원리
────
  · 연속 스캔 ICP  →  차체 속도 (v̂_x, v̂_y, ω̂) 추정  (LiDAR 기반)
  · Linear  slip: |v_wheel − v̂_x | ≥ δ_lin   (휠 vs 실속도)
  · Angular slip: |ω_kin   − ω̂   | ≥ δ_ang   (기구학 vs 실 요레이트)
                  ω_kin = v_wheel · tan(δ) / L
  · μ̂ = max_{no-slip} √(â_x² + â_y²) / g

일반 주행 중 상시 작동 — 원 그리는 테스트 불필요.
"""
import csv, math, os, time, unicodedata
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64

from race_stack.paths import repo_root

_REPO = repo_root()
_OUT  = os.path.join(_REPO, 'logs', 'slip_tests')
G     = 9.80665
BOX_W = 60


def _w(s):
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in s)

def _row(s):
    while _w(s) > BOX_W:
        s = s[:-1]
    return '║' + s + ' ' * (BOX_W - _w(s)) + '║'


# ── LiDAR ICP ────────────────────────────────────────────────────────
def _scan_xy(msg: LaserScan, max_r: float) -> np.ndarray:
    n = len(msg.ranges)
    a = np.arange(n, dtype=np.float32) * msg.angle_increment + msg.angle_min
    r = np.asarray(msg.ranges, dtype=np.float32)
    ok = np.isfinite(r) & (r > msg.range_min) & (r < max_r)
    return np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])].astype(np.float64)


def _icp(src: np.ndarray, dst: np.ndarray,
         max_iter: int = 12, tol: float = 5e-4) -> tuple[float, float, float]:
    """연속 스캔 ICP. 반환: (tx, ty, dθ) = 차체 변위 (dt로 나누면 속도).

    물리적 의미:
      · 차가 전진 dx   → tx = +dx  → v_x = tx/dt > 0
      · 차가 좌이동 dy → ty = +dy  → v_y = ty/dt > 0
      · 차가 좌회전 dθ → dθ = +dθ  → ω   = dθ/dt > 0
    """
    from scipy.spatial import cKDTree

    if len(src) > 200:
        src = src[np.random.choice(len(src), 200, replace=False)]
    if len(dst) > 200:
        dst = dst[np.random.choice(len(dst), 200, replace=False)]

    tree = cKDTree(dst)
    R = np.eye(2)
    t = np.zeros(2)

    for _ in range(max_iter):
        src_t  = src @ R.T + t
        d, ix  = tree.query(src_t, workers=1)
        thr    = np.percentile(d, 70) * 2.0
        mask   = d < thr
        if mask.sum() < 8:
            return 0.0, 0.0, 0.0

        sm, dm = src[mask], dst[ix[mask]]
        ms, md = sm.mean(0), dm.mean(0)
        H      = (sm - ms).T @ (dm - md)
        U, _, Vt = np.linalg.svd(H)
        Rn = Vt.T @ U.T
        if np.linalg.det(Rn) < 0:
            Vt[-1] *= -1;  Rn = Vt.T @ U.T
        tn  = md - Rn @ ms
        thn = math.atan2(Rn[1, 0], Rn[0, 0])
        dth = abs(thn - math.atan2(R[1, 0], R[0, 0]))

        if np.hypot(tn[0] - t[0], tn[1] - t[1]) + dth < tol:
            R, t = Rn, tn;  break
        R, t = Rn, tn

    return float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0])


# ── 노드 ─────────────────────────────────────────────────────────────
IDLE, BIAS, CAL, RUN = 0, 1, 2, 3
_PHASE = {IDLE: '대기', BIAS: '바이어스', CAL: '임계값교정', RUN: '추정중'}


class SlipFriction(Node):
    def __init__(self):
        super().__init__('slip_friction')
        d = self.declare_parameter

        # 토픽
        d('scan_topic',   '/scan')
        d('odom_topic',   '/odom')
        d('imu_topic',    '/sensors/imu/raw')
        d('servo_topic',  '/commands/servo/position')

        # 차량 기하
        d('wheelbase',    0.33)
        d('servo_gain',   -0.761)
        d('servo_offset', 0.425)
        d('gyro_units',   'deg')
        d('accel_units',  'g')

        # 교정
        d('bias_sec',     3.0)     # 정지 바이어스 측정
        d('cal_sec',      5.0)     # 임계값 교정 주행 시간

        # 슬립 임계값 (음수 = 자동 교정)
        d('delta_lin',   -1.0)    # m/s
        d('delta_ang',   -1.0)    # rad/s

        # ICP
        d('max_scan_m',   4.0)
        d('icp_hz',       10.0)   # 속도: 40Hz 스캔에서 몇 Hz로 ICP?
        d('lpf_hz',       8.0)

        # 출력
        d('min_speed',    0.3)     # 이 이상에서만 추정

        p = lambda n: self.get_parameter(n).value
        self.L   = float(p('wheelbase'))
        self.sg  = float(p('servo_gain'))
        self.so  = float(p('servo_offset'))
        self.to_rad = math.pi/180 if str(p('gyro_units')).lower().startswith('deg') else 1.0
        self.to_ms2 = G if str(p('accel_units')).lower() == 'g' else 1.0
        self.bias_sec = float(p('bias_sec'))
        self.cal_sec  = float(p('cal_sec'))
        self.d_lin    = float(p('delta_lin'))
        self.d_ang    = float(p('delta_ang'))
        self.max_scan_m = float(p('max_scan_m'))
        self.icp_dt_min = 1.0 / max(float(p('icp_hz')), 1.0)
        self.lpf_hz   = float(p('lpf_hz'))
        self.min_speed = float(p('min_speed'))

        # 센서 상태
        self.v_wheel = 0.0
        self.delta   = 0.0
        self.ax_f = self.ay_f = self.wz_f = 0.0
        self.bias = {'ax': 0.0, 'ay': 0.0, 'wz': 0.0}
        self._imu_t  = None

        # ICP 상태
        self._scan_prev   = None
        self._scan_prev_t = None
        self._icp_last    = 0.0
        self.vx_l = self.vy_l = self.wz_l = 0.0
        self._icp_ok = False
        self._icp_age = 999.0

        # 슬립 & μ
        self.slip_lin = self.slip_ang = False
        self.mu_hat   = 0.0
        self.rho_now  = 0.0

        # 단계 제어
        self.phase = IDLE
        self._phase_t = None
        self._bias_buf = []
        self._cal_lin  = []
        self._cal_ang  = []

        # 기록
        os.makedirs(_OUT, exist_ok=True)
        self._rows  = []
        self._t0    = None
        self._msg   = '정지 상태에서 B 버튼 또는 자동 시작 대기'

        # ROS
        from sensor_msgs.msg import Joy
        self.create_subscription(LaserScan, p('scan_topic'), self._scan_cb, 10)
        self.create_subscription(Odometry,  p('odom_topic'), self._odom_cb, 20)
        self.create_subscription(Imu, p('imu_topic'), self._imu_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Float64, p('servo_topic'), self._servo_cb, 20)

        self.create_timer(0.3, self._draw)
        self.create_timer(1.0, self._auto_save)

        self.get_logger().info('slip_friction 준비 완료')

    # ── 입력 콜백 ────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.v_wheel = float(msg.twist.twist.linear.x)

    def _servo_cb(self, msg: Float64):
        self.delta = (float(msg.data) - self.so) / self.sg

    def _scan_cb(self, msg: LaserScan):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pts = _scan_xy(msg, self.max_scan_m)

        if (self._scan_prev is not None
                and self._scan_prev_t is not None
                and now - self._icp_last >= self.icp_dt_min):

            dt = now - self._scan_prev_t
            if 0.005 < dt < 0.5 and len(pts) >= 20 and len(self._scan_prev) >= 20:
                try:
                    tx, ty, dth = _icp(self._scan_prev, pts, max_iter=12)
                    # LPF
                    a = dt / (dt + 1.0 / (2 * math.pi * max(self.lpf_hz, 0.1)))
                    self.vx_l  += a * (tx / dt - self.vx_l)
                    self.vy_l  += a * (ty / dt - self.vy_l)
                    self.wz_l  += a * (dth / dt - self.wz_l)
                    self._icp_ok  = True
                    self._icp_age = 0.0
                    self._icp_last = now
                except Exception:
                    self._icp_ok = False

        self._scan_prev   = pts
        self._scan_prev_t = now

    def _imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0:
            t = self.get_clock().now().nanoseconds * 1e-9
        dt = (t - self._imu_t) if self._imu_t else 0.02
        self._imu_t = t
        if not 1e-4 < dt < 0.5:
            dt = 0.02
        self._icp_age += dt

        ax_raw = -msg.linear_acceleration.y * self.to_ms2
        ay_raw = +msg.linear_acceleration.x * self.to_ms2
        wz_raw = +msg.angular_velocity.z * self.to_rad

        # ─ 바이어스 측정 ─────────────────────────────────────────────
        if self.phase == IDLE:
            # 첫 데이터 오면 바이어스 측정 시작
            self.phase = BIAS
            self._phase_t = t
            self._msg = f'바이어스 측정 중 ({self.bias_sec:.0f}초)...'
            self.get_logger().info('바이어스 측정 시작 — 정지 유지')

        if self.phase == BIAS:
            self._bias_buf.append((wz_raw, ax_raw, ay_raw))
            if t - self._phase_t >= self.bias_sec:
                a = np.asarray(self._bias_buf)
                self.bias = {'wz': float(a[:, 0].mean()),
                             'ax': float(a[:, 1].mean()),
                             'ay': float(a[:, 2].mean())}
                self.phase = CAL
                self._phase_t = t
                self._msg = (f'바이어스 완료. {self.cal_sec:.0f}초 천천히 직진하세요 — 임계값 교정중')
                self.get_logger().info(
                    f'바이어스 완료 (wz={math.degrees(self.bias["wz"]):+.2f}°/s). '
                    f'임계값 교정 — {self.cal_sec:.0f}초 천천히 직진')
            return

        # 바이어스 제거
        ax = ax_raw - self.bias['ax']
        ay = ay_raw - self.bias['ay']
        wz = wz_raw - self.bias['wz']

        # LPF
        a_lpf = dt / (dt + 1.0 / (2 * math.pi * self.lpf_hz))
        self.ax_f += a_lpf * (ax - self.ax_f)
        self.ay_f += a_lpf * (ay - self.ay_f)
        self.wz_f += a_lpf * (wz - self.wz_f)

        # ─ 임계값 교정 ────────────────────────────────────────────────
        if self.phase == CAL:
            if self._icp_ok and abs(self.v_wheel) > self.min_speed:
                kin = self.v_wheel * math.tan(self.delta) / self.L
                self._cal_lin.append(abs(self.v_wheel - self.vx_l))
                self._cal_ang.append(abs(kin - self.wz_l))

            if t - self._phase_t >= self.cal_sec:
                # 임계값 = mean + 2·σ (ICP 데이터가 없으면 기본값)
                if self.d_lin < 0:
                    if self._cal_lin:
                        m, s = np.mean(self._cal_lin), np.std(self._cal_lin)
                        self.d_lin = float(m + 2 * s)
                    else:
                        self.d_lin = 0.15
                if self.d_ang < 0:
                    if self._cal_ang:
                        m, s = np.mean(self._cal_ang), np.std(self._cal_ang)
                        self.d_ang = float(m + 2 * s)
                    else:
                        self.d_ang = 0.20
                self.phase = RUN
                self._msg = (f'교정 완료 δ_lin={self.d_lin:.3f}m/s '
                             f'δ_ang={math.degrees(self.d_ang):.1f}°/s')
                self.get_logger().info(
                    f'교정 완료 — δ_lin={self.d_lin:.3f}m/s  '
                    f'δ_ang={math.degrees(self.d_ang):.1f}°/s  '
                    f'(교정 샘플: lin={len(self._cal_lin)} ang={len(self._cal_ang)})')
            return

        # ─ 정상 추정 ──────────────────────────────────────────────────
        if self.phase != RUN:
            return
        if abs(self.v_wheel) < self.min_speed:
            return

        icp_fresh = self._icp_age < 0.2   # 0.2초 이내 ICP 결과

        kin_wz = self.v_wheel * math.tan(self.delta) / self.L

        # eq. 9, 13 (2509.15423)
        self.slip_lin = icp_fresh and (abs(self.v_wheel - self.vx_l) > self.d_lin)
        self.slip_ang = icp_fresh and (abs(kin_wz - self.wz_l) > self.d_ang)
        no_slip = not self.slip_lin and not self.slip_ang

        # eq. 17, 19 — 비슬립 구간 최대 traction coefficient
        self.rho_now = math.sqrt(self.ax_f ** 2 + self.ay_f ** 2) / G
        if no_slip:
            self.mu_hat = max(self.mu_hat, self.rho_now)

        # 기록
        if self._t0 is None:
            self._t0 = t
        self._rows.append((
            round(t - self._t0, 4),
            round(self.v_wheel, 5),
            round(self.delta,   5),
            round(self.wz_f,    5),
            round(self.vx_l,    5),
            round(self.vy_l,    5),
            round(self.wz_l,    5),
            round(self.ax_f,    5),
            round(self.ay_f,    5),
            round(kin_wz,       5),
            round(self.rho_now, 5),
            int(self.slip_lin),
            int(self.slip_ang),
            round(self.mu_hat,  5),
        ))

    # ── 저장 ─────────────────────────────────────────────────────────
    def _save(self):
        if not self._rows:
            return
        ts   = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(_OUT, f'{ts}_friction.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['# slip_friction — 2509.15423 방식'])
            w.writerow(['t', 'v_wheel', 'delta', 'wz_imu',
                        'vx_lidar', 'vy_lidar', 'wz_lidar',
                        'ax', 'ay', 'wz_kin',
                        'rho', 'slip_lin', 'slip_ang', 'mu_hat'])
            for row in self._rows:
                w.writerow(row)
        n_slip = sum(r[11] or r[12] for r in self._rows)
        self.get_logger().info(
            f'저장: {os.path.basename(path)}  '
            f'{len(self._rows)}행  슬립{n_slip}회  μ̂={self.mu_hat:.4f}')
        self._rows.clear()
        self._t0 = None
        return path

    def _auto_save(self):
        if len(self._rows) > 3000:   # ~60초분
            self._save()

    # ── 화면 ─────────────────────────────────────────────────────────
    def _draw(self):
        bar = '═' * BOX_W
        kin_wz = self.v_wheel * math.tan(self.delta) / self.L if self.phase == RUN else 0.0
        icp_ok  = self._icp_ok and self._icp_age < 0.2
        slip    = self.slip_lin or self.slip_ang

        lines = ['\033[2J\033[H', '╔' + bar + '╗']
        A = lines.append
        A(_row(f'  마찰계수 추정기 (2509.15423)     [{_PHASE[self.phase]:^10}]'))
        A('╠' + bar + '╣')

        if self.phase == RUN:
            A(_row(f'  v_wheel={self.v_wheel:5.2f}㎧   '
                   f'v̂_x_lidar={self.vx_l:+5.2f}   '
                   f'v̂_y={self.vy_l:+5.2f}'))
            lin_err = abs(self.v_wheel - self.vx_l)
            ang_err = abs(kin_wz - self.wz_l)
            A(_row(f'  Δv_lin={lin_err:.3f}㎧ (임계{self.d_lin:.3f})  '
                   f'Δω_ang={math.degrees(ang_err):.1f}°/s (임계{math.degrees(self.d_ang):.1f})'))
            A(_row(f'  ω_kin={math.degrees(kin_wz):+5.1f}°/s  '
                   f'ω_lidar={math.degrees(self.wz_l):+5.1f}°/s  '
                   f'ω_imu={math.degrees(self.wz_f):+5.1f}°/s'))
            A('╠' + bar + '╣')
            slip_txt = (('⚠ SLIP ' +
                         ('LIN' if self.slip_lin else '') +
                         ('+' if self.slip_lin and self.slip_ang else '') +
                         ('ANG' if self.slip_ang else ''))
                        if slip else '  정상주행')
            A(_row(f'  {slip_txt:<14}  '
                   f'ρ={self.rho_now:.3f}g   '
                   f'μ̂={self.mu_hat:.4f}  '
                   f'ICP={"✔" if icp_ok else "✘("+f"{self._icp_age:.1f}s)"}'))
        else:
            A(_row(f'  {self._msg}'))
            if self.phase == CAL and self._cal_lin:
                A(_row(f'  교정 샘플 lin={len(self._cal_lin)}  ang={len(self._cal_ang)}  '
                       f'v={self.v_wheel:.2f}㎧'))

        A('╠' + bar + '╣')
        A(_row(f'  기록 {len(self._rows)}행   ICP 나이 {self._icp_age:.2f}s'))
        A('╚' + bar + '╝')
        if self._msg and self.phase != RUN:
            lines.append('  ' + self._msg)
        print('\n'.join(lines), flush=True)


def main():
    rclpy.init()
    node = SlipFriction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._save()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
