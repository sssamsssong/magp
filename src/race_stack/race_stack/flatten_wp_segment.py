 #!/usr/bin/env python3
"""
flatten_wp_segment.py — 이미 만든 웨이포인트의 한 구간만 "최대한 평평하게" 다시 깎는다.

gen_raceline.py 와의 차이
─────────────────────────
gen_raceline.py 는 트랙 **전체**에 대해 곡률 최소화 QP를 푼다. 시작점(seed)이
"양쪽 벽에서 가장 먼 능선(max-clearance ridge)"이라, 장애물이 여러 번 튀어나와
능선 자체가 S자로 휘는 구간에서는 --safety/--apex-extra 를 아무리 밀어붙여도
그 능선 근처에서만 맴돈다 (실측: 0824-2_clean 에서 --safety 0.10 --apex-extra 1.2
--rmin 0.90 까지 줘도 형태가 거의 그대로였다).

이 스크립트는 그 능선을 따라가지 않는다. 지정한 구간에서 목적함수 자체를
"곡률 최소화"가 아니라 **"차 반폭+여유를 지키는 한 가장 평평하게(y 변화 최소)"**
로 바꾼 별도 QP를 그 구간에만 푼다. 나머지 구간은 그대로 두고, 경계에서
원래 라인과 위치·기울기가 매끄럽게 맞물리도록 코사인 램프로 블렌딩한다.

★ x 가 그 구간에서 단조 증가/감소해야 한다 (y = f(x) 로 풀기 때문).
  헤어핀처럼 되짚어오는 구간엔 못 쓴다 — 그런 구간은 gen_raceline.py 의
  전역 최적화가 이미 맞는 도구다. 이건 "한쪽 벽을 여러 번 스치며 구불거리는
  직선형 통로"에만 쓰는 국소 보정 도구다.

사용 예:
  python3 -m race_stack.flatten_wp_segment --map 0824-2_clean \
    --in 0824-2_clean_defaultgeo_wp --out 0824-2_clean_flat_wp \
    --x0 -3.68 --x1 8.70 --margin 0.32
"""
import argparse
import csv
import os
import sys

import cv2
import numpy as np
import osqp
import scipy.sparse as sp
import yaml
from PIL import Image

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAPS_DIR = os.path.join(_PKG_DIR, 'maps')
WP_DIR = os.path.join(_PKG_DIR, 'waypoints')

from race_stack.gen_raceline import calc_vel_profile, save_overlay  # noqa: E402


def _load_map(map_name):
    cfg = yaml.safe_load(open(os.path.join(MAPS_DIR, map_name + '.yaml')))
    res = float(cfg['resolution'])
    ox, oy = float(cfg['origin'][0]), float(cfg['origin'][1])
    img = np.asarray(Image.open(os.path.join(MAPS_DIR, cfg['image'])).convert('L'))
    H, W = img.shape
    free = img > 128
    dist = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 5) * res
    return dist, res, ox, oy, H, W


def _load_wp(path):
    pts = []
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            pts.append([float(row[0]), float(row[1]), float(row[2])])
    return np.array(pts)


def _band_at(dist, res, ox, oy, H, W, x, y_ref, margin):
    """이 x 에서 벽 간격이 margin 이상인 y 구간들 중 y_ref 에 가장 가까운 것."""
    px = int(np.clip(round((x - ox) / res), 0, W - 1))
    col = dist[:, px]
    ok = col >= margin
    runs, run = [], None
    for i, v in enumerate(ok):
        if v and run is None:
            run = i
        if not v and run is not None:
            runs.append((run, i - 1))
            run = None
    if run is not None:
        runs.append((run, len(ok) - 1))
    if not runs:
        return None
    best, bd = None, 1e9
    for a, b in runs:
        y_hi, y_lo = oy + (H - a) * res, oy + (H - b) * res
        if y_lo <= y_ref <= y_hi:
            return (y_lo, y_hi)
        c = (y_lo + y_hi) / 2
        if abs(c - y_ref) < bd:
            bd, best = abs(c - y_ref), (y_lo, y_hi)
    return best


def flatten_segment(map_name, in_wp, out_wp, x0, x1, margin, blend_frac,
                    n_grid, mu, pac_d, vmax, vmin, acc, brk, y_near=None):
    dist, res, ox, oy, H, W = _load_map(map_name)
    pts = _load_wp(os.path.join(WP_DIR, in_wp + '.csv'))

    x_all = pts[:, 0]
    lo, hi = min(x0, x1), max(x0, x1)
    in_range = (x_all >= lo) & (x_all <= hi)
    idxs = np.where(in_range)[0]
    if len(idxs) < 5:
        sys.exit(f'구간에 점이 {len(idxs)}개뿐 — x0/x1 범위를 확인하세요')
    # ★ 폐루프라 x 범위가 겹치는 구간(예: 위쪽 직선 + 아래쪽 구불 구간)이
    #   동시에 걸릴 수 있다. 전부 이어붙었다고 가정하지 않고, 가장 긴
    #   "연속 인덱스 구간"만 골라서 쓴다 — 그게 사용자가 가리킨 구간이다.
    gaps = np.where(np.diff(idxs) != 1)[0]
    runs = np.split(idxs, gaps + 1)
    if len(runs) > 1:
        print(f'⚠ x 범위 [{lo:.2f},{hi:.2f}] 가 서로 떨어진 {len(runs)}개 구간과 겹칩니다 '
             f'(폐루프에서 흔함) — 그중 가장 긴 구간만 씁니다')
    if y_near is not None and len(runs) > 1:
        # 지정한 y 에 평균이 가장 가까운 구간 (외벽/안쪽 직선 구분용)
        run = min(runs, key=lambda r: abs(float(np.mean(pts[r, 1])) - y_near))
        print(f'  --y-near {y_near:.2f} → 평균 y {float(np.mean(pts[run,1])):.2f} 구간 선택 (idx {int(run[0])}~{int(run[-1])})')
    else:
        run = max(runs, key=len)
    i0, i1 = int(run[0]), int(run[-1])
    seg = pts[i0:i1 + 1]
    x_seg, y_seg = seg[:, 0], seg[:, 1]
    if not (np.all(np.diff(x_seg) > 0) or np.all(np.diff(x_seg) < 0)):
        sys.exit('이 구간에서 x 가 단조가 아닙니다 (헤어핀 포함?) — '
                 '이 도구는 못 씁니다. gen_raceline.py 전역 최적화를 쓰세요')
    if x_seg[0] > x_seg[-1]:
        x_seg, y_seg = x_seg[::-1], y_seg[::-1]
        flipped = True
    else:
        flipped = False

    xg = np.linspace(x_seg[0], x_seg[-1], n_grid)
    y_ref = np.interp(xg, x_seg, y_seg)

    lb = np.full(n_grid, -1e3)
    ub = np.full(n_grid, 1e3)
    for i, x in enumerate(xg):
        band = _band_at(dist, res, ox, oy, H, W, x, y_ref[i], margin)
        if band is None:
            sys.exit(f'x={x:.2f} 에서 margin={margin}m 을 만족하는 자유공간이 없습니다 '
                     f'— margin 을 줄이거나 구간을 좁히세요')
        lb[i], ub[i] = band
    lb[0] = ub[0] = y_ref[0]
    lb[-1] = ub[-1] = y_ref[-1]

    # 2차 차분(곡률 근사) 최소화 — "가장 평평한" y(x) 를 만든다.
    rows, cols, data = [], [], []
    for i in range(1, n_grid - 1):
        rows += [i - 1, i - 1, i - 1]
        cols += [i - 1, i, i + 1]
        data += [1, -2, 1]
    D = sp.csc_matrix((data, (rows, cols)), shape=(n_grid - 2, n_grid))
    P = (D.T @ D).tocsc()
    prob = osqp.OSQP()
    prob.setup(P, np.zeros(n_grid), sp.eye(n_grid, format='csc'), lb, ub,
              verbose=False, eps_abs=1e-6, eps_rel=1e-6, max_iter=20000)
    result = prob.solve()
    if result.info.status != 'solved':
        sys.exit(f'QP 실패: {result.info.status}')
    y_flat = result.x

    # 경계에서 원래 라인과 매끄럽게 맞물리도록 코사인 램프로 블렌딩
    k = max(2, int(n_grid * blend_frac))
    w = np.ones(n_grid)
    ramp = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, k))
    w[:k] = ramp
    w[-k:] = ramp[::-1]
    y_blend = w * y_flat + (1 - w) * y_ref

    y_new = np.interp(x_seg, xg, y_blend)
    if flipped:
        y_new = y_new[::-1]

    new_pts = pts.copy()
    new_pts[i0:i1 + 1, 1] = y_new

    raceline = new_pts[:, :2]
    el = np.linalg.norm(np.diff(np.vstack([raceline, raceline[0]]), axis=0), axis=1)
    dx = np.gradient(raceline[:, 0])
    dy = np.gradient(raceline[:, 1])
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    kappa = np.abs(dx * ddy - dy * ddx) / np.maximum((dx**2 + dy**2) ** 1.5, 1e-9)

    print(f'평탄화 구간: idx {i0}~{i1} ({len(seg)}pts)  '
         f'swing {y_seg.max()-y_seg.min():.3f}m → {y_new.max()-y_new.min():.3f}m')
    print(f'전체 곡률 최대 {kappa.max():.3f} 1/m  (R_min={1/max(kappa.max(),1e-9):.3f}m)')

    v = calc_vel_profile(kappa, el, mu=mu, v_max=vmax, v_min=vmin,
                         a_lon_acc=acc, a_lon_brk=brk)   # car2 calc_vel_profile: pac_d 없음(mu·g)

    out_path = os.path.join(WP_DIR, out_wp + '.csv')
    with open(out_path, 'w', newline='') as f:
        w_ = csv.writer(f)
        w_.writerow(['x', 'y', 'speed'])
        for (x, y), vi in zip(raceline, v):
            w_.writerow([f'{x:.4f}', f'{y:.4f}', f'{vi:.4f}'])
    print(f'저장: {out_path}')
    save_overlay(map_name, raceline, v, out_path, None)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--map', required=True, help='맵 이름 (예: 0824-2_clean)')
    p.add_argument('--in', dest='in_wp', required=True, help='입력 웨이포인트 이름 (확장자 없이)')
    p.add_argument('--out', dest='out_wp', required=True, help='출력 웨이포인트 이름')
    p.add_argument('--x0', type=float, required=True, help='평탄화할 구간 시작 x (맵 좌표, m)')
    p.add_argument('--x1', type=float, required=True, help='평탄화할 구간 끝 x (맵 좌표, m)')
    p.add_argument('--margin', type=float, default=0.32,
                   help='벽까지 최소 간격 — 차 반폭+여유 (기본 0.32 = 0.155+0.165)')
    p.add_argument('--blend-frac', type=float, default=0.18,
                   help='구간 양끝에서 원래 라인과 섞는 비율 (기본 0.18)')
    p.add_argument('--n-grid', type=int, default=260, help='내부 계산 격자 점 수')
    p.add_argument('--mu', type=float, default=0.9)
    p.add_argument('--pac-d', dest='pac_d', type=float, default=1.15)
    p.add_argument('--vmax', type=float, default=12.0)
    p.add_argument('--vmin', type=float, default=1.0)
    p.add_argument('--acc', type=float, default=5.0)
    p.add_argument('--brk', type=float, default=8.0)
    p.add_argument('--y-near', dest='y_near', type=float, default=None,
                   help='폐루프에서 x 범위에 직선이 2개 겹칠 때, 이 y 에 가장 가까운 '
                        '구간을 고른다 (없으면 가장 긴 구간). 예: 외벽쪽 직선이면 --y-near 4.2')
    args = p.parse_args()
    flatten_segment(args.map, args.in_wp, args.out_wp, args.x0, args.x1,
                    args.margin, args.blend_frac, args.n_grid,
                    args.mu, args.pac_d, args.vmax, args.vmin, args.acc, args.brk,
                    y_near=args.y_near)


if __name__ == '__main__':
    main()
