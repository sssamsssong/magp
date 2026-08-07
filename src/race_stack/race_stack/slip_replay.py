#!/usr/bin/env python3
"""slip_replay — 녹화한 bag 으로 slip_friction 추정을 오프라인 재생한다.

왜 필요한가
──────────
ICP 파라미터(max_scan_m, icp_hz, icp_points, lidar_x)를 CSV 로는 못 고친다.
CSV 에는 이미 추정이 끝난 결과만 남기 때문이다. 원시 /scan 을 다시 돌려야
하고, 그건 차를 굴리지 않고 노트북에서 할 수 있어야 한다.

사용
────
  ros2 run race_stack slip_replay -- <bag_경로> [--icp-hz 10] [--max-scan 10]
                                     [--points 400] [--lidar-x 0.27]

출력: 요약 통계 + <bag>_replay.csv (slip_friction 과 같은 컬럼)
"""
import argparse, csv, math, os, sys
import numpy as np

from race_stack.slip_friction import _icp, _scan_xy, G


def read_bag(path):
    """bag 에서 (t, LaserScan) · (t, odom v) · (t, servo) · (t, imu) 를 뽑는다."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    rdr = rosbag2_py.SequentialReader()
    rdr.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
             rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in rdr.get_all_topics_and_types()}
    out = {k: [] for k in ('scan', 'odom', 'servo', 'imu')}
    key = {'/scan': 'scan', '/odom': 'odom',
           '/commands/servo/position': 'servo', '/sensors/imu/raw': 'imu'}
    while rdr.has_next():
        topic, data, ts = rdr.read_next()
        k = key.get(topic)
        if k is None:
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        out[k].append((ts * 1e-9, msg))
    return out


def _calib_lever(vw, vy, wz, max_g):
    """저속 정상원 주행에서 lidar_x 를 역산한다.

    원리: 횡가속이 충분히 작으면 타이어 슬립이 무시할 만해서 후축의 횡속도가
    0 이다. 그러면 LiDAR 에서 재는 횡속도는 순수 레버암 성분뿐이다:
        v_y_laser = ω · x_laser      →  x_laser = v_y_laser / ω

    ★ 반드시 저속이어야 한다. 미끄러지기 시작하면 횡속도 0 인 점이 후축 뒤로
      밀려 레버암이 실제보다 크게 나온다 — 2026-08-07 일반 주행 로그에서
      0.39m 로 나온 게 그 경우다(TF 선언값은 0.27m).
    """
    g = 9.80665
    alat = np.abs(vw * wz) / g                    # 구심가속도 [g]
    m = (alat < max_g) & (np.abs(wz) > math.radians(15)) & (vw > 0.5)
    print(f'\n── 레버암 캘리브레이션 (횡가속 < {max_g}g, |ω| > 15°/s) ──')
    print(f'  해당 구간 {m.sum()}행 / {len(vw)}'
          f'   횡가속 중앙 {np.median(alat[m]) if m.sum() else float("nan"):.3f}g')
    if m.sum() < 50:
        print('  ★ 표본 부족 — 더 크고 느린 원으로, 더 오래 돌아야 한다.')
        print('    (속도 1.0~1.5㎧ / 반경 2~3m 로 2~3바퀴, 좌·우 각각)')
        return

    for nm, mm in (('좌회전 ω>0', m & (wz > 0)),
                   ('우회전 ω<0', m & (wz < 0)),
                   ('전체',       m)):
        if mm.sum() < 20:
            print(f'  {nm:12s} 표본 부족 ({mm.sum()}행)')
            continue
        ratio = vy[mm] / wz[mm]
        slope = np.polyfit(wz[mm], vy[mm], 1)[0]
        print(f'  {nm:12s} n={mm.sum():4d}  비율중앙 {np.median(ratio):+.3f}m  '
              f'회귀기울기 {slope:+.3f}m  (표준편차 {np.std(ratio):.3f})')
    print('  → 좌/우 값이 비슷하면 신뢰할 만하다. 크게 갈리면 편향이 남은 것.')
    print(f'  → race_script.yaml 의 slip.lidar_x 에 넣을 것 (TF 선언값 0.27m)')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--icp-hz',   type=float, default=10.0)
    ap.add_argument('--max-scan', type=float, default=10.0)
    ap.add_argument('--points',   type=int,   default=400)
    ap.add_argument('--lidar-x',  type=float, default=0.27)
    ap.add_argument('--lpf-hz',   type=float, default=2.0)
    ap.add_argument('--wheelbase',type=float, default=0.33)
    ap.add_argument('--servo-gain',   type=float, default=-0.761)
    ap.add_argument('--servo-offset', type=float, default=0.425)
    ap.add_argument('--servo-min',type=float, default=0.15)
    ap.add_argument('--servo-max',type=float, default=0.85)
    ap.add_argument('--calib-lever', action='store_true',
                    help='저속 원 주행 bag 에서 lidar_x 를 역산한다')
    ap.add_argument('--calib-max-g', type=float, default=0.15,
                    help='캘리브레이션에 쓸 최대 횡가속 [g]. 이보다 크면 슬립이 '
                         '섞여 레버암이 부풀어 보인다')
    a = ap.parse_args([x for x in (argv or sys.argv[1:]) if x != '--'])

    b = read_bag(a.bag)
    if not b['scan']:
        print('★ /scan 이 bag 에 없다'); return 1
    print(f'스캔 {len(b["scan"])}개  odom {len(b["odom"])}  servo {len(b["servo"])}')

    # 최근접 시각으로 보간할 준비
    def sampler(seq, fn):
        ts = np.array([t for t, _ in seq]);  vs = np.array([fn(m) for _, m in seq])
        return lambda q: float(vs[np.searchsorted(ts, q).clip(0, len(vs) - 1)])
    v_of = sampler(b['odom'],  lambda m: m.twist.twist.linear.x)
    s_of = sampler(b['servo'], lambda m: m.data)
    g_of = sampler(b['imu'],   lambda m: m.angular_velocity.z)

    dt_min = 1.0 / max(a.icp_hz, 1.0)
    prev = prev_t = None
    vx = vy = wz = 0.0
    icp_ok = False
    rows = []

    for t, msg in b['scan']:
        pts = _scan_xy(msg, a.max_scan)
        if prev is None:
            prev, prev_t = pts, t;  continue
        dt = t - prev_t
        if dt < dt_min:
            continue
        if not (0.005 < dt < 0.5 and len(pts) >= 20 and len(prev) >= 20):
            prev, prev_t = pts, t;  continue

        vw = v_of(t)
        sv = min(max(s_of(t), a.servo_min), a.servo_max)
        dl = (sv - a.servo_offset) / a.servo_gain
        gy = math.radians(g_of(t))          # IMU 자이로 (deg/s 가정)

        cands = [(0.0, 0.0, 0.0), (vw * dt, 0.0, gy * dt)]
        if icp_ok:
            cands.append((vx * dt, vy * dt, wz * dt))
        best = None
        for c in cands:
            r = _icp(prev, pts, init=c, n_src=a.points)
            if r[3] and (best is None or r[4] < best[4]):
                best = r
        if best:
            icp_ok = True
            al = dt / (dt + 1.0 / (2 * math.pi * max(a.lpf_hz, 0.1)))
            vx += al * (best[0] / dt - vx)
            vy += al * (best[1] / dt - vy)
            wz += al * (best[2] / dt - wz)

        vyb = vy - a.lidar_x * wz
        rows.append((t, vw, dl, gy, vx, vy, vyb, wz,
                     vw * math.tan(dl) / a.wheelbase))
        prev, prev_t = pts, t

    d = np.array(rows)
    if len(d) < 10:
        print('★ 유효 행이 너무 적다'); return 1
    t, vw, dl, gy, vx, vy, vyb, wz, wk = d.T
    m = vw > 1.0
    beta_raw = np.degrees(np.arctan2(vy[m],  vw[m]))
    beta     = np.degrees(np.arctan2(vyb[m], vw[m]))
    lo = m & (np.abs(wz) < math.radians(10))

    print(f'\n재생 {len(d)}행  ({a.icp_hz}Hz, max_scan {a.max_scan}m, '
          f'{a.points}pt, lidar_x {a.lidar_x}m)')
    print(f'  ω_lidar vs ω_imu   corr={np.corrcoef(wz, gy)[0,1]:+.3f}  '
          f'기울기={np.polyfit(wz, gy, 1)[0]:+.3f}')
    print(f'  |β| 중앙값  레버암 보정 전 {np.median(abs(beta_raw)):5.1f}°  '
          f'→ 보정 후 {np.median(abs(beta)):5.1f}°')
    if lo.sum() > 30:
        bl = np.degrees(np.arctan2(vyb[lo], vw[lo]))
        print(f'  저요레이트(|ω|<10°/s) n={lo.sum()}  |β| 중앙값 {np.median(abs(bl)):.1f}°  '
              f'95%p {np.percentile(abs(bl),95):.1f}°   ← 0 에 가까워야')

    if a.calib_lever:
        _calib_lever(vw, vy, wz, a.calib_max_g)

    out = os.path.join(os.path.dirname(os.path.abspath(a.bag)),
                       os.path.basename(a.bag.rstrip('/')) + '_replay.csv')
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'v_wheel', 'delta', 'wz_imu', 'vx_lidar',
                    'vy_lidar', 'vy_base', 'wz_lidar', 'wz_kin'])
        w.writerows(np.round(d, 5))
    print(f'  저장: {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
