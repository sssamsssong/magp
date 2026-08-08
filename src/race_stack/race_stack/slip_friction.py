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
def _scan_xy(msg: LaserScan, max_r: float,
             w: float = 0.0, v: float = 0.0,
             fallback_scan_time: float = 0.025) -> np.ndarray:
    """스캔을 차체 좌표 점군으로. w·v 를 주면 **모션 디스큐**까지 한다.

    왜 디스큐가 필수인가 (2026-08-09)
    ─────────────────────────────────
    2D LiDAR 는 한 바퀴 도는 데 시간이 걸린다 (Hokuyo 40Hz → 25ms). 그 사이
    차가 움직이면 첫 빔과 마지막 빔은 **다른 위치에서** 찍힌 것인데 메시지는
    한 순간처럼 담는다. 크기를 재보면 이게 부수적 오차가 아니다:

        2.5㎧ × 25ms        = 6.3cm   ← 스캔 내부 병진
        ω 1.5rad/s × 25ms  = 2.15°   → 5m 점이 18cm 어긋남
        정작 재려는 100ms 횡변위    = 2cm

    스캔 내부 왜곡이 신호보다 3배 크다. ICP 가 v_y 를 못 맞추던 주된 이유로
    유력하다 (실측 v_y 표준편차 0.238㎧ = α 로 ±6°).

    ★ ω 는 **IMU 자이로**를 쓴다. /odom 의 각속도는 서보 지령 기반 기구학값
      (vesc.yaml use_servo_cmd_to_calc_angular_velocity: true)이라 언더스티어
      상황에서 실제의 ~1.9배로 나온다 — 그걸 쓰면 보정이 과하게 걸린다.

    기준 시각은 header.stamp = **첫 빔**(urg_node 규약)이다. 빔 i 는 그보다
    i·time_increment 만큼 뒤에 찍혔으므로, 그 시점 차체 자세를 되돌린다:
        p_ref = R(ω·Δt)·p_i + (v·Δt, 0)
    """
    n = len(msg.ranges)
    idx = np.arange(n, dtype=np.float64)
    a = idx * msg.angle_increment + msg.angle_min
    r = np.asarray(msg.ranges, dtype=np.float64)
    ok = np.isfinite(r) & (r > msg.range_min) & (r < max_r)
    x = r[ok] * np.cos(a[ok])
    y = r[ok] * np.sin(a[ok])

    if abs(w) > 1e-3 or abs(v) > 1e-2:
        ti = float(msg.time_increment)
        if ti <= 0.0:
            st = float(msg.scan_time) if msg.scan_time > 0 else fallback_scan_time
            ti = st / max(n, 1)
        dt = idx[ok] * ti
        th = w * dt
        c, s = np.cos(th), np.sin(th)
        x, y = c * x - s * y + v * dt, s * x + c * y

    return np.c_[x, y]


def _icp(src: np.ndarray, dst: np.ndarray,
         init: tuple[float, float, float] = (0.0, 0.0, 0.0),
         n_src: int = 400,
         max_iter: int = 12,
         tol: float = 5e-4) -> tuple[float, float, float, bool, float]:
    """스캔 정합 ICP. 반환: (tx, ty, dθ, ok) = 차체 변위 (dt로 나누면 속도).

    물리적 의미:
      · 차가 전진 dx   → tx = +dx  → v_x = tx/dt > 0
      · 차가 좌이동 dy → ty = +dy  → v_y = ty/dt > 0
      · 차가 좌회전 dθ → dθ = +dθ  → ω   = dθ/dt > 0

    ★★ 반환값은 **차체 운동**이다 — ICP 가 직접 찾는 src→dst 변환의 역 (2026-08-07).
      차가 앞으로 가면 정지한 벽은 차체 좌표계에서 **뒤로** 흐르므로, ICP 가
      찾는 변환은 차의 운동과 부호가 반대다. 예전 코드는 그 변환을 그대로
      속도로 썼다 — 전진 중 v̂_x 가 음수로 나왔고, 따라서 |v_wheel − v̂_x| 는
      항상 ≈2v 가 되어 슬립 판정과 자동 임계값 교정이 통째로 무의미했다.
      (합성 점군 검증: 차 +0.20m 전진 → 옛 코드 반환 −0.20)

    ★ init 도 같은 **차체 운동** 규약으로 받는다. 베이스라인 100ms 에서는
      항등 초기값으로 수렴하지 않는다 — 2㎧면 그 사이 20cm 를 움직이는데
      이는 ICP 수렴 반경 밖이다. 호출자가 여러 후보를 넣고 **잔차(rms)로
      고르도록** 되어 있다. 초기값 하나만 쓰면 0 에 락인된다 (2026-08-07:
      vx_l=0 → init=0 → ICP 가 0 반환 → vx_l 이 영원히 0. 실측에서 v_wheel
      2.1㎧ 정속인데 vx_lidar 가 0.0~0.4 에 갇힌 원인이 정확히 이것).

    반환 rms 는 인라이어 평균 대응거리 — 후보 간 비교에만 쓴다.

    ★ dst 는 **절대 솎지 않는다** (2026-08-07). KD-tree 를 200점으로 만들면
      최근접점 간격이 실제 점간격의 5배가 되고, 그 양자화 오차(3m 거리에서
      수 cm)가 정작 우리가 재려는 횡변위(수 mm~cm)보다 커진다. 비용이 드는
      건 질의 쪽(src)뿐이라 src 만 솎으면 된다 — 트리 생성은 O(n log n) 로
      1000점이어도 무시할 만하다.
    """
    from scipy.spatial import cKDTree

    if len(src) > n_src:
        src = src[np.random.choice(len(src), n_src, replace=False)]

    tree = cKDTree(dst)

    # 차체 운동 규약의 init 을 ICP 내부 규약(src→dst 변환)으로 뒤집어 넣는다:
    #   R_icp = R(−dθ),  t_icp = −R_icp · (dx, dy)
    dx0, dy0, dth0 = float(init[0]), float(init[1]), float(init[2])
    c, s = math.cos(-dth0), math.sin(-dth0)
    R = np.array([[c, -s], [s, c]])
    t = -R @ np.array([dx0, dy0])
    ok = False
    rms = float('inf')

    for _ in range(max_iter):
        src_t  = src @ R.T + t
        d, ix  = tree.query(src_t, workers=1)
        thr    = np.percentile(d, 70) * 2.0
        mask   = d < thr
        if mask.sum() < 8:
            return 0.0, 0.0, 0.0, False, float('inf')
        rms = float(np.sqrt(np.mean(d[mask] ** 2)))

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
        ok  = True

        if np.hypot(tn[0] - t[0], tn[1] - t[1]) + dth < tol:
            R, t = Rn, tn;  break
        R, t = Rn, tn

    # src→dst 변환 (R, t) 의 역 (Rᵀ, −Rᵀt) 이 곧 차가 움직인 양이다.
    d = -R.T @ t
    return float(d[0]), float(d[1]), -math.atan2(R[1, 0], R[0, 0]), ok, rms


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
        d('wheelbase',    0.32)
        d('servo_gain',   -0.761)
        d('servo_offset', 0.425)
        # 기구 스토퍼. vesc.yaml 의 servo_min/max 와 같은 값이어야 한다.
        d('servo_min',    0.15)   # → 좌 락 +0.361 rad (20.7°)
        d('servo_max',    0.85)   # → 우 락 −0.558 rad (32.0°)

        # ★ base_link → laser 전방 거리 [m] (2026-08-07 추가).
        #   ICP 가 재는 건 **LiDAR 위치**의 속도인데 슬립각은 차체 기준이다.
        #   레버암 때문에 v_y_laser = v_y_base + ω·x 만큼 요레이트가 새어든다.
        #   ω 가 80°/s 면 0.27×1.4 = 0.38㎧ — 우리가 재려는 횡속도보다 크다.
        #   실측(2026-08-07): 보정 전 |β| 중앙값 8.9°, 보정 후 1.8°.
        #   값은 f1tenth_stack bringup 의 static TF (0.27 0 0.11) 에서 왔다.
        #   ※ 데이터로 회귀하면 0.37~0.385m 가 나오는데 그 초과분은 실제
        #     후륜 슬립이다(횡속도 0 인 점이 후축 뒤로 밀린다). 그걸 레버암에
        #     넣어버리면 정작 재려는 신호를 지운다 — 반드시 TF 기하값을 쓸 것.
        d('lidar_x',      0.27)
        d('gyro_units',   'deg')
        d('accel_units',  'g')

        # 교정
        d('bias_sec',     3.0)     # 정지 바이어스 측정
        d('cal_sec',      5.0)     # 임계값 교정 주행 시간

        # 슬립 임계값 (음수 = 자동 교정)
        d('delta_lin',   -1.0)    # m/s
        d('delta_ang',   -1.0)    # rad/s

        # ★ 종방향 슬립 판정은 기본 OFF 다 (2026-08-07 실측·검증).
        #   D_lin 은 |v_wheel − v̂_x| 인데, 연속 스캔 정합으로는 v̂_x 를 얻을 수
        #   없다. 트랙처럼 **진행 방향과 나란한 벽**이 지배적인 장면에서는
        #   종방향으로 얼마를 움직였든 스캔이 똑같아 보인다 — 잔차 지형을
        #   재보면 dx 를 0→0.36m 로 훑어도 잔차가 20.2~21.2mm 로 평평하다.
        #   정보가 없으므로 초기값·반복횟수·사거리 어떤 튜닝으로도 안 된다.
        #   (실측: v_wheel 2.1㎧ 정속인데 vx_lidar 가 0.0~0.4 에 갇힘)
        #
        #   같은 기하가 횡방향은 오히려 강하게 구속한다 — dy 를 ±0.1m 옮기면
        #   잔차가 20.6 → 107mm 로 치솟는다. 그래서 v̂_y 와 ω̂ 는 쓸 수 있다
        #   (v_y 오차 0.018㎧). 슬립각 α 에 필요한 건 이쪽이다.
        #
        #   참고한 2509.15423 이 v̂_x 를 쓴 건 스캔 정합이 아니라 **지도 기반
        #   EKF 위치추정**이라서다. 지도가 있으면 절대 위치로 종방향이 관측된다.
        #   켜려면 AMCL 을 띄우고 그 pose 를 미분해 쓰는 쪽이 맞다.
        d('enable_lin_slip', False)

        # ICP
        # ★ 4.0 은 너무 짧다 (2026-08-07 실측). Hokuyo 는 20m 를 보는데 4m 로
        #   자르면 복도 끝·모서리가 잘려 나가고 **진행 방향과 나란한 옆벽만**
        #   남는다. 그러면 종방향 이동이 기하학적으로 관측 불가능해진다
        #   (개구 문제). 실제로 v_wheel=2.1㎧ 정속 주행 중 vx_lidar 가 0.0~0.4
        #   로 죽어 있었고, 회전만 멀쩡히 추정됐다(ω corr 0.71 vs v corr −0.17)
        #   — 회전은 옆벽만으로도 구속되기 때문이다.
        # ★ 모션 디스큐 기본 OFF (2026-08-09 실측). 물리적으로는 옳지만
        #   기존 bag 재생에서 이득이 없었다 — |β| 중앙 2.5°→2.4°, 저요레이트
        #   95%p 는 14.8°→16.4° 로 오히려 악화. 두 스캔이 같은 방식으로
        #   찌그러지면 ICP 의 **상대** 변환에서 공통 성분이 상쇄되기 때문이다.
        #   급가감속이 많은 데이터에서는 달라질 수 있어 옵션은 남긴다.
        d('deskew',       False)
        d('max_scan_m',   10.0)
        d('icp_points',   400)    # ICP 질의점 수 (많을수록 분산↓, 비용↑)
        d('icp_hz',       10.0)   # ★ 정합 베이스라인이기도 하다 (10Hz → 100ms)
        d('lpf_hz',       8.0)    # IMU (a_x, a_y, ω_z) LPF 컷오프
        # LiDAR 속도는 IMU 와 갱신율이 10배 차이라 컷오프를 따로 갖는다.
        # 2.0Hz 는 옛 코드의 **실효** 컷오프(≈1.3Hz)를 그대로 재현하는 값이다
        # — 옛 lpf_hz=8.0 은 25ms 로 계산한 alpha 를 100ms 마다 적용한 결과라
        # 이름값(8Hz)과 전혀 달랐다. 갱신율 10Hz 에서 8Hz 컷오프는 나이퀴스트를
        # 넘어 사실상 필터가 없는 것과 같으니 그 값을 물려받으면 안 된다.
        d('lidar_lpf_hz', 2.0)

        # 출력
        d('min_speed',    0.3)     # 이 이상에서만 추정

        p = lambda n: self.get_parameter(n).value
        self.L   = float(p('wheelbase'))
        self.sg  = float(p('servo_gain'))
        self.so  = float(p('servo_offset'))
        self.s_lo = min(float(p('servo_min')), float(p('servo_max')))
        self.s_hi = max(float(p('servo_min')), float(p('servo_max')))
        self.lidar_x = float(p('lidar_x'))
        self.to_rad = math.pi/180 if str(p('gyro_units')).lower().startswith('deg') else 1.0
        self.to_ms2 = G if str(p('accel_units')).lower() == 'g' else 1.0
        self.bias_sec = float(p('bias_sec'))
        self.cal_sec  = float(p('cal_sec'))
        self.d_lin    = float(p('delta_lin'))
        self.d_ang    = float(p('delta_ang'))
        self.en_lin   = bool(p('enable_lin_slip'))
        self.deskew     = bool(p('deskew'))
        self.max_scan_m = float(p('max_scan_m'))
        self.icp_pts    = int(p('icp_points'))
        self.icp_dt_min = 1.0 / max(float(p('icp_hz')), 1.0)
        self.lpf_hz   = float(p('lpf_hz'))
        self.lidar_lpf_hz = float(p('lidar_lpf_hz'))
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
        self.vx_l = self.vy_l = self.wz_l = 0.0
        self._icp_ok = False
        self._icp_age = 999.0
        self.icp_rms = float('nan')

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

    def vy_base(self) -> float:
        """LiDAR 위치의 횡속도를 차체(base_link) 기준으로 옮긴다.

        v_y_base = v_y_laser − ω · x_laser  (레버암 제거). 슬립각 α 계산은
        반드시 이 값을 써야 한다 — 위 lidar_x 파라미터 주석 참고.
        """
        return self.vy_l - self.lidar_x * self.wz_l

    # ── 입력 콜백 ────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.v_wheel = float(msg.twist.twist.linear.x)

    def _servo_cb(self, msg: Float64):
        # ★ 이 토픽은 **클립 전** 지령값이다 (2026-08-07). 실제로 자르는 건
        #   vesc_driver 의 servo_min/max 이므로 여기서 같이 잘라야 실각도다.
        #   안 자르면 좌회전에서 20.7°(기구 락) 를 31.5°(지령) 로 기록하고,
        #   ω_kin = v·tan(δ)/L 이 1.5배 부풀어 그게 통째로 가짜 각슬립이 된다
        #   — 2026-08-07 로그에서 각슬립이 79~91% 행에 찍힌 주원인.
        s = min(max(float(msg.data), self.s_lo), self.s_hi)
        self.delta = (s - self.so) / self.sg

    def _scan_cb(self, msg: LaserScan):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # 디스큐에 쓸 ω 는 IMU 자이로(바이어스 제거·필터 후), v 는 휠 속도.
        # 둘 다 이 노드가 이미 들고 있으므로 별도 노드/토픽이 필요 없고,
        # 그 덕에 **이미 녹화된 bag 에도 같은 보정이 적용**된다.
        pts = _scan_xy(msg, self.max_scan_m,
                       w=(self.wz_f if self.deskew and self.phase == RUN else 0.0),
                       v=(self.v_wheel if self.deskew else 0.0))

        if self._scan_prev is None or self._scan_prev_t is None:
            self._scan_prev, self._scan_prev_t = pts, now
            return

        # ★ 기준 스캔은 ICP 를 실제로 돌린 시점에만 갱신한다 (2026-08-07).
        #   예전엔 매 스캔 갱신해서, icp_hz 를 낮춰도 정합 베이스라인은 항상
        #   스캔 간격(40Hz → 25ms)이었다. v_y = 0.2㎧ 면 그 사이 횡변위가 5mm
        #   — LiDAR 거리 노이즈와 같은 자릿수라 슬립각이 노이즈에 묻혔다.
        #   이제 icp_hz 가 곧 베이스라인이다 (10Hz → 100ms → 같은 조건 20mm).
        dt = now - self._scan_prev_t
        if dt < self.icp_dt_min:
            return

        if 0.005 < dt < 0.5 and len(pts) >= 20 and len(self._scan_prev) >= 20:
            try:
                # ── 다중 시작 + 잔차 선택 ────────────────────────────
                # 초기값 하나로는 0 에 락인된다(위 _icp docstring). 후보를
                # 여러 개 돌리고 정합 잔차가 가장 작은 것을 고른다.
                #
                # ★ 휠·자이로 후보를 넣어도 순환논법이 아니다 — 그 값을
                #   **믿는** 게 아니라 출발점으로만 쓰고, 채택 여부는 스캔
                #   정합 잔차가 정하기 때문이다. 휠이 미끄러지고 있으면 그
                #   후보는 잔차가 커져서 탈락하거나, ICP 가 실제 운동으로
                #   끌려가 수렴한다. 어느 쪽이든 답은 스캔이 정한다.
                cands = [(0.0, 0.0, 0.0),
                         (self.v_wheel * dt, 0.0, self.wz_f * dt)]
                if self._icp_ok:
                    cands.append((self.vx_l * dt, self.vy_l * dt, self.wz_l * dt))

                best = None
                for c in cands:
                    res = _icp(self._scan_prev, pts, init=c, n_src=self.icp_pts)
                    if res[3] and (best is None or res[4] < best[4]):
                        best = res

                ok = best is not None
                if ok:
                    tx, ty, dth = best[0], best[1], best[2]
                    self.icp_rms = best[4]
                    # LPF — alpha 는 **실제 갱신 간격** dt 로 계산된다. 예전엔
                    # 스캔 간격(25ms)으로 계산해 놓고 100ms 마다 적용해서,
                    # 설정한 컷오프와 실효 컷오프가 4배 어긋나 있었다.
                    a = dt / (dt + 1.0 / (2 * math.pi * max(self.lidar_lpf_hz, 0.1)))
                    self.vx_l  += a * (tx / dt - self.vx_l)
                    self.vy_l  += a * (ty / dt - self.vy_l)
                    self.wz_l  += a * (dth / dt - self.wz_l)
                    self._icp_ok  = True
                    self._icp_age = 0.0
                else:
                    self._icp_ok = False
            except Exception:
                self._icp_ok = False

        self._scan_prev, self._scan_prev_t = pts, now

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
        # 종방향은 관측 불가라 기본 OFF — 위 enable_lin_slip 주석 참고.
        self.slip_lin = (self.en_lin and icp_fresh
                         and abs(self.v_wheel - self.vx_l) > self.d_lin)
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
            round(self.vy_base(), 5),
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
            # vy_base = vy_lidar − ω·lidar_x (레버암 제거). α 계산엔 이걸 쓸 것.
            # vx_lidar 는 진단용으로만 남긴다 — 종방향은 관측 불가 (아래 참고).
            w.writerow(['t', 'v_wheel', 'delta', 'wz_imu',
                        'vx_lidar', 'vy_lidar', 'vy_base', 'wz_lidar',
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
            vyb  = self.vy_base()
            beta = math.degrees(math.atan2(vyb, max(abs(self.v_wheel), 0.3)))
            A(_row(f'  v_x={self.v_wheel:5.2f}㎧(휠)  '
                   f'v̂_y={vyb:+5.2f}㎧(차체)  '
                   f'β={beta:+5.1f}°  (레버암 {self.lidar_x:.2f}m 보정)'))
            ang_err = abs(kin_wz - self.wz_l)
            lin_txt = (f'Δv_lin={abs(self.v_wheel - self.vx_l):.2f}(임계{self.d_lin:.2f})'
                       if self.en_lin else 'Δv_lin=OFF(종방향 관측불가)')
            A(_row(f'  {lin_txt}  '
                   f'Δω={math.degrees(ang_err):.1f}°/s(임계{math.degrees(self.d_ang):.1f})'))
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
