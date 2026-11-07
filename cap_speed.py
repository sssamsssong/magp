#!/usr/bin/env python3
"""cap_speed.py — 웨이포인트 csv 의 속도를 구간별로 낮춰 새 파일로 쓴다.

왜 필요한가
──────────
속도 노브가 모드별 grip_util 하나뿐이라, 위험한 코너 하나 때문에 랩 전체를 늦춰야
했다. 그런데 solve_profile 은 csv 속도를 **상한으로** 쓰고 그 위에서 가감속 패스를
돌린다. 그래서 그 구간의 csv 값만 낮추면 **컨트롤러를 고치지 않고** 구간별 제한이
된다. 감속 시작점도 프로파일이 알아서 앞당긴다.

  낮추기만 한다. 물리 한계 위로는 못 올린다 (프로파일이 다시 자른다).

사용
────
  # 저장 없이 효과만 본다 (s 는 lap_report.py 가 찍어 주는 값)
  ./cap_speed.py 0824-2_clean_map_wp --zone 8:12:3.0 --mode q1 --dry-run

  # <이름>_cap.csv 로 저장. 원본은 안 건드린다
  ./cap_speed.py 0824-2_clean_map_wp --zone 8:12:3.0 --zone 40:2:2.5 --mode q1

  --zone 시작s:끝s:최고속[㎧]. 시작>끝 이면 출발선을 넘는 구간이다.
  저장 후 race_script.yaml 의 waypoints: 를 새 이름으로 바꿔 기동한다 (커밋 금지).
  되돌리려면 그 한 줄만 원래 이름으로 돌리면 된다.

주의
────
  · 표의 프로파일은 컨트롤러와 같은 함수·곡률 계산으로 푼 **추정**이다. 실제 랩타임은
    구동계 지연·회피·슬립으로 더 길다. 전후 **차이**를 보는 용도로 쓸 것.
  · --mode 를 안 주면 csv 만 고친다 (효과 확인 없음).
"""
import argparse
import importlib.util
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
WP_DIR = os.path.join(ROOT, 'src', 'race_stack', 'waypoints')
MIN_CAP = 0.5        # 이보다 낮으면 오타로 본다 (speed.min_speed 근처면 코너에서 선다)


def parse_zone(text):
    try:
        a, b, v = (float(q) for q in text.split(':'))
    except ValueError:
        raise argparse.ArgumentTypeError(f'{text!r}: 시작s:끝s:최고속 형식이어야 한다')
    if v < MIN_CAP:
        raise argparse.ArgumentTypeError(f'{text!r}: 최고속 {v} 은 {MIN_CAP}㎧ 미만이다')
    return a, b, v


def load_csv(path):
    rows = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            cells = line.strip().split(',')
            try:
                rows.append([float(q) for q in cells[:3]])
            except ValueError:
                continue        # 헤더
    arr = np.array(rows)
    if arr.ndim != 2 or arr.shape[1] != 3 or len(arr) < 3:
        sys.exit(f'✗ x,y,speed 3열 csv 가 아니다: {path}')
    return arr


def zone_mask(s, length, a, b):
    if not (0.0 <= a <= length and 0.0 <= b <= length):
        sys.exit(f'✗ 구간 {a}~{b}m 가 둘레 {length:.1f}m 밖이다')
    return (s >= a) & (s < b) if a < b else (s >= a) | (s < b)


def curvature(pts):
    """map_controller._compute_curvature 와 같은 주기 스플라인."""
    from scipy.interpolate import CubicSpline
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    s_ext = np.append(s, s[-1] + float(np.linalg.norm(pts[0] - pts[-1])))
    cx = CubicSpline(s_ext, np.append(pts[:, 0], pts[0, 0]), bc_type='periodic')
    cy = CubicSpline(s_ext, np.append(pts[:, 1], pts[0, 1]), bc_type='periodic')
    dx, dy, ddx, ddy = cx(s, 1), cy(s, 1), cx(s, 2), cy(s, 2)
    return np.abs(dx * ddy - dy * ddx) / np.maximum((dx ** 2 + dy ** 2) ** 1.5, 1e-6)


def _deep_get(d, dotted, default=None):
    for key in dotted.split('.'):
        if not isinstance(d, dict) or key not in d:
            return default
        d = d[key]
    return d


def vehicle_params(car, mode, grip):
    """race.launch.py 와 같은 출처에서 프로파일 인자를 모은다."""
    import yaml
    if not car:
        path = os.path.join(ROOT, '.car')
        if os.path.exists(path):
            car = open(path, encoding='utf-8').read().strip()
    car = car or 'car1'
    with open(os.path.join(ROOT, 'config', f'vehicle_{car}.yaml'), encoding='utf-8') as f:
        veh = yaml.safe_load(f)
    with open(os.path.join(ROOT, 'race_script.yaml'), encoding='utf-8') as f:
        race = yaml.safe_load(f)
    if grip is None:
        table = _deep_get(veh, 'tuning.grip_util')
        grip = table.get(mode) if isinstance(table, dict) else table
        if grip is None:
            sys.exit(f'✗ {car} 의 grip_util 에 {mode} 가 없다 — --grip 으로 지정할 것')
    return car, float(grip), dict(
        mu_eff=float(_deep_get(veh, 'tire.mu')) * float(_deep_get(veh, 'tire.pac_d')),
        lf=float(_deep_get(veh, 'geometry.lf')),
        lr=float(_deep_get(veh, 'geometry.lr')),
        v_min=float(_deep_get(race, 'speed.min_speed', 0.4)),
        a_lon_acc=float(_deep_get(veh, 'tuning.a_lon_acc', 0.0)),
        a_lon_brk=float(_deep_get(veh, 'tuning.a_lon_brk', 0.0)),
        hard_cap=float(_deep_get(race, 'speed.hard_cap', 0.0)))


def load_solver():
    # 패키지를 통째로 import 하면 rclpy 가 필요하다. 이 모듈은 numpy 만 쓴다
    path = os.path.join(ROOT, 'src', 'race_stack', 'race_stack', 'speed_profile.py')
    spec = importlib.util.spec_from_file_location('speed_profile', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.solve_profile


def show_effect(before, after, zones, s, length, args):
    car, grip, params = vehicle_params(args.car, args.mode, args.grip)
    solve = load_solver()
    kap = curvature(before[:, :2])
    v0, i0 = solve(before[:, 0], before[:, 1], before[:, 2], grip_util=grip,
                   curvature=kap, **params)
    v1, i1 = solve(after[:, 0], after[:, 1], after[:, 2], grip_util=grip,
                   curvature=kap, **params)
    print(f'\n프로파일 추정 ({car}, {args.mode or "지정값"}, η={grip:.2f}, '
          f'μ_eff={params["mu_eff"]:.3f})')
    print(f'  랩타임 {i0["lap_time"]:.2f}s → {i1["lap_time"]:.2f}s '
          f'({i1["lap_time"] - i0["lap_time"]:+.2f}s)')
    for a, b, cap in zones:
        m = zone_mask(s, length, a, b)
        print(f'  s {a:g}~{b:g}m 상한 {cap:g}㎧: 최고속 {v0[m].max():.2f} → '
              f'{v1[m].max():.2f}㎧')
        if v0[m].max() <= cap:
            print('    ⚠ 이미 프로파일이 상한보다 낮다 — 이 구간 제한은 효과가 없다')
    # 제동이 앞당겨진 거리 — 구간 앞에서 속도가 줄기 시작하는 곳까지
    slowed = v1 < v0 - 0.01
    print(f'  속도가 줄어든 구간 길이 {slowed.mean() * length:.1f}m / 둘레 {length:.1f}m')


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('waypoints', help='웨이포인트 이름 또는 csv 경로')
    ap.add_argument('--zone', action='append', required=True, type=parse_zone,
                    metavar='시작s:끝s:최고속')
    ap.add_argument('--out', help='출력 이름 (기본 <이름>_cap)')
    ap.add_argument('--mode', help='프로파일 효과를 볼 모드 (q1, q3, race ...)')
    ap.add_argument('--grip', type=float, help='grip_util 을 직접 지정')
    ap.add_argument('--car', help='차량 (기본 .car, 없으면 car1)')
    ap.add_argument('--dry-run', action='store_true', help='저장하지 않는다')
    args = ap.parse_args()

    src = args.waypoints if args.waypoints.endswith('.csv') else \
        os.path.join(WP_DIR, args.waypoints + '.csv')
    if not os.path.exists(src):
        sys.exit(f'✗ 웨이포인트 없음: {src}')
    before = load_csv(src)
    if not np.all(before[:, 2] > 0):
        # solve_profile 은 0 이하가 하나라도 있으면 csv 속도 전체를 조용히 버린다
        sys.exit('✗ 원본에 0 이하 속도가 있어 컨트롤러가 csv 상한을 통째로 무시한다')

    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(before[:, :2], axis=0).T))]
    length = float(s[-1])
    after = before.copy()
    print(f'{src}\n  점 {len(before)}개, 둘레 {length:.1f}m')
    for a, b, cap in args.zone:
        m = zone_mask(s, length, a, b)
        if not m.any():
            sys.exit(f'✗ 구간 {a}~{b}m 에 웨이포인트가 없다')
        after[m, 2] = np.minimum(after[m, 2], cap)
        print(f'  s {a:g}~{b:g}m: 점 {int(m.sum())}개, csv 최고 '
              f'{before[m, 2].max():.2f} → {after[m, 2].max():.2f}㎧')

    if args.mode or args.grip is not None:
        show_effect(before, after, args.zone, s, length, args)

    if args.dry_run:
        print('\n--dry-run 이라 저장하지 않았다.')
        return
    base = os.path.splitext(os.path.basename(src))[0]
    name = args.out or (base + '_cap')
    dst = os.path.join(os.path.dirname(src), name + '.csv')
    if os.path.abspath(dst) == os.path.abspath(src):
        sys.exit('✗ 원본에 덮어쓰지 않는다 — --out 을 다른 이름으로')
    np.savetxt(dst, after, fmt='%.4f', delimiter=',', header='x,y,speed', comments='')
    print(f'\n저장: {dst}')
    print(f'race_script.yaml 의 waypoints: 를 {name} 로 바꿔 기동한다 (커밋 금지).')


if __name__ == '__main__':
    main()
