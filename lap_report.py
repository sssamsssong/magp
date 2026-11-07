#!/usr/bin/env python3
"""lap_report.py — 주행 로그를 랩 단위로 나눠 "이 값이 안정적인가" 를 숫자로 본다.

왜 필요한가
──────────
대회 현장에서 grip_util 을 올릴지 정할 때 최고 랩 하나만 보면 속는다. 빠른데
랩마다 들쭉날쭉하면 다음 랩에 벽을 받는다. 평균과 **산포**를 같이 봐야 한다.
횡오차가 몰리는 구간은 s 로 찍어 주므로 cap_speed.py 의 --zone 에 그대로 쓴다.

사용
────
  ./lap_report.py logs/20260825_012715_race_map            # 한 주행
  ./lap_report.py logs/<주행A> logs/<주행B>                 # 값 바꾸기 전후 비교
  ./lap_report.py --last 3                                 # 최근 3 주행
  ./lap_report.py logs/<주행> --wp <경로.csv>               # 웨이포인트 직접 지정

읽는 법
───────
  · 랩시간 산포가 크면 그 값은 아직 불안정하다 (평균이 빨라도)
  · 횡오차는 회피 중(|selected_d|>5㎝)과 위치 품질이 나쁜 프레임(잔차>40㎝)을 뺀다
  · 차가 끊기거나 위치가 튄 랩은 '불연속' 으로 표시하고 요약에서 뺀다

주의
────
  · driven_path.csv 는 **마지막 3000점만** 남는다 (약 3분). 그보다 긴 주행은 앞이 잘린다.
  · 웨이포인트는 같은 이름으로 덮어쓰인 적이 있다 (0821_clean). 그래서 지금 파일과
    주행 시각의 커밋을 둘 다 맞춰 보고 잘 맞는 쪽을 쓴다.
  · 맵 없이 재므로 벽까지의 거리는 모른다.
"""
import argparse
import csv
import glob
import os
import re
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
AVOID_M = 0.05       # 이보다 옆으로 나가 있으면 회피 중으로 본다
LOC_BAD_M = 0.40     # 이보다 잔차가 크면 위치 품질이 나쁜 프레임이다
GAP_S = 1.0          # 기록이 이만큼 끊기면 그 랩은 믿지 않는다
JUMP_FRAC = 0.05     # 한 프레임에 트랙 길이의 이 비율을 넘게 튀면 불연속이다
SECTION_M = 2.0      # 구간 분석 격자
MIN_FRAMES = 20      # 이보다 적은 칸은 통계를 내지 않는다


def read_log(run_dir):
    path = os.path.join(run_dir, 'driven_path.csv')
    with open(path, encoding='utf-8', errors='replace') as f:
        rows = list(csv.DictReader(f))

    def col(key):
        out = []
        for r in rows:
            try:
                out.append(float(r.get(key) or 'nan'))
            except ValueError:
                out.append(float('nan'))
        return np.array(out)
    return {k: col(k) for k in ('t', 'x', 'y', 'speed', 'steer', 'near',
                                'loc_residual', 'selected_d')}


def read_gains(run_dir):
    """gains.yaml 에서 필요한 값만. 차량 덮어쓰기가 뒤에 오므로 마지막 값이 이긴다."""
    out = {}
    path = os.path.join(run_dir, 'gains.yaml')
    if not os.path.exists(path):
        return out
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = re.match(r'^\s*(waypoints|max_steer)\s*:\s*([^#\s]+)', line)
            if m:
                out[m.group(1)] = m.group(2)
    return out


def parse_waypoints(text):
    rows = []
    for line in text.splitlines():
        cells = line.split(',')
        try:
            rows.append((float(cells[0]), float(cells[1])))
        except (ValueError, IndexError):
            continue        # 헤더
    return np.array(rows) if rows else None


def run_stamp(run_dir):
    m = re.match(r'(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})',
                 os.path.basename(os.path.normpath(run_dir)))
    if not m:
        return None
    y, mo, d, h, mi, s = m.groups()
    return f'{y}-{mo}-{d} {h}:{mi}:{s} +0900'


def waypoints_at_run(name, run_dir):
    """주행 시각의 커밋에 있던 웨이포인트. git 이나 기록이 없으면 None."""
    stamp = run_stamp(run_dir)
    if not stamp:
        return None
    rel = f'src/race_stack/waypoints/{name}.csv'
    try:
        commit = subprocess.run(
            ['git', 'log', '-1', '--format=%H', f'--before={stamp}', '--', rel],
            cwd=ROOT, capture_output=True, text=True).stdout.strip()
        if not commit:
            return None
        res = subprocess.run(['git', 'show', f'{commit}:{rel}'], cwd=ROOT,
                             capture_output=True)
        return parse_waypoints(res.stdout.decode('utf-8', 'replace'))
    except OSError:
        return None


def nearest(points, wp):
    """점마다 가장 가까운 웨이포인트의 (번호, 거리)."""
    idx = np.zeros(len(points), int)
    dist = np.zeros(len(points))
    for i in range(0, len(points), 500):        # 3000×수천 점이라 나눠서 잰다
        d2 = ((points[i:i + 500, None, :] - wp[None, :, :]) ** 2).sum(-1)
        idx[i:i + 500] = d2.argmin(1)
        dist[i:i + 500] = np.sqrt(d2.min(1))
    return idx, dist


def choose_waypoints(log, gains, run_dir, override):
    """(웨이포인트, 설명). 로그와 번호가 맞고 횡오차가 작은 쪽을 고른다."""
    if override:
        with open(override, encoding='utf-8', errors='replace') as f:
            return parse_waypoints(f.read()), override
    name = gains.get('waypoints')
    if not name:
        return None, 'gains.yaml 에 waypoints 없음'
    cands = []
    path = os.path.join(ROOT, 'src', 'race_stack', 'waypoints', name + '.csv')
    if os.path.exists(path):
        with open(path, encoding='utf-8', errors='replace') as f:
            cands.append((parse_waypoints(f.read()), f'{name} (현재 파일)'))
    old = waypoints_at_run(name, run_dir)
    if old is not None:
        cands.append((old, f'{name} (주행 시각 커밋)'))
    near_max = np.nanmax(log['near']) if np.isfinite(log['near']).any() else -1
    points = np.c_[log['x'], log['y']]
    points = points[np.isfinite(points).all(1)]
    best, best_e = (None, f'{name}: 파일을 찾지 못함'), float('inf')
    for wp, label in cands:
        if wp is None or near_max >= len(wp):
            continue
        err = float(np.median(nearest(points, wp)[1]))
        if err < best_e:
            best, best_e = (wp, label), err
    return best


def split_laps(log, n_wp):
    """near 가 한 바퀴 돌 때마다 자른다. [(시작, 끝, 불연속 여부)]."""
    ok = np.isfinite(log['near']) & np.isfinite(log['t'])
    frames = np.flatnonzero(ok)
    if len(frames) < 2:
        return []
    near, t = log['near'][frames].astype(int), log['t'][frames]
    # 진행량을 (-N/2, N/2] 로 접는다. 뒤집힌 웨이포인트(reverse_check)도 near 는 늘어난다
    step = (np.diff(near) + n_wp // 2) % n_wp - n_wp // 2
    broken = (np.abs(step) > JUMP_FRAC * n_wp) | (np.diff(t) > GAP_S)
    wraps = np.flatnonzero((near[1:] < near[:-1]) & (step > 0)) + 1
    laps = []
    for a, b in zip(wraps[:-1], wraps[1:]):
        laps.append((frames[a], frames[b], bool(broken[a:b].any())))
    return laps


def lap_stats(log, a, b, cte, max_steer):
    span = np.arange(a, b)
    moving = log['speed'][span] > 0.5
    d = np.nan_to_num(log['selected_d'][span])
    res = log['loc_residual'][span]
    clean = (np.abs(d) < AVOID_M) & ~(res > LOC_BAD_M)
    e = cte[span][clean]
    return {
        'time': float(log['t'][b] - log['t'][a]),
        'vmax': float(np.nanmax(log['speed'][span])),
        'cte_mean': float(e.mean()) if len(e) else float('nan'),
        'cte_max': float(e.max()) if len(e) else float('nan'),
        'sat': float(np.mean(np.abs(log['steer'][span][moving]) >= 0.95 * max_steer))
               if moving.any() and max_steer else float('nan'),
        'avoid': float(np.mean(np.abs(d) >= AVOID_M)),
    }


def worst_sections(log, cte, s, top):
    """파일 순서 s 기준 격자로 횡오차가 큰 구간을 찾는다 (cap_speed --zone 과 같은 s)."""
    d = np.nan_to_num(log['selected_d'])
    good = (log['speed'] > 0.5) & (np.abs(d) < AVOID_M) & ~(log['loc_residual'] > LOC_BAD_M)
    good &= np.isfinite(cte)
    rows = []
    for lo in np.arange(0.0, s.max() + SECTION_M, SECTION_M):
        m = good & (s >= lo) & (s < lo + SECTION_M)
        if m.sum() >= MIN_FRAMES:
            rows.append((float(np.percentile(cte[m], 95)), float(lo),
                         float(np.median(log['speed'][m])), int(m.sum()),
                         float(np.median(log['x'][m])), float(np.median(log['y'][m]))))
    return sorted(rows, reverse=True)[:top]


def report(run_dir, override, top):
    name = os.path.basename(os.path.normpath(run_dir))
    print(f'\n=== {name}')
    if not os.path.exists(os.path.join(run_dir, 'driven_path.csv')):
        print('  driven_path.csv 없음')
        return None
    log = read_log(run_dir)
    gains = read_gains(run_dir)
    wp, label = choose_waypoints(log, gains, run_dir, override)
    if wp is None:
        print(f'  웨이포인트를 못 정했다: {label} — --wp 로 지정할 것')
        return None
    if np.nanmax(log['near']) >= len(wp):
        print(f'  ⚠ {label} 점 {len(wp)}개는 이 로그와 안 맞다 (near 최대 '
              f'{int(np.nanmax(log["near"]))}) — 주행 당시 파일을 --wp 로 지정할 것')
        return None
    pts = np.c_[log['x'], log['y']]
    ok = np.isfinite(pts).all(1)
    idx, cte = np.zeros(len(pts), int), np.full(len(pts), np.nan)
    idx[ok], cte[ok] = nearest(pts[ok], wp)
    cum = np.r_[0.0, np.cumsum(np.hypot(*np.diff(wp, axis=0).T))]
    length = cum[-1]
    print(f'  웨이포인트 {label}, 둘레 {length:.1f}m, 기록 '
          f'{log["t"][-1] - log["t"][0]:.0f}s ({len(pts)}점)')
    if len(pts) >= 3000:
        print('  ※ 3000점 상한에 닿았다 — 이 주행의 앞부분은 잘렸다')

    max_steer = float(gains.get('max_steer', 0) or 0)
    stats = []
    print('  랩    시간     최고속  횡오차평균/최대  조향포화  회피')
    for i, (a, b, broken) in enumerate(split_laps(log, len(wp)), start=1):
        q = lap_stats(log, a, b, cte, max_steer)
        mark = '  불연속' if broken else ''
        print(f"  {i:2d}  {q['time']:6.2f}s  {q['vmax']:5.2f}㎧  "
              f"{q['cte_mean'] * 100:5.1f} / {q['cte_max'] * 100:5.1f}㎝  "
              f"{q['sat'] * 100:6.1f}%  {q['avoid'] * 100:3.0f}%{mark}")
        if not broken:
            stats.append(q)

    summary = None
    if stats:
        times = np.array([q['time'] for q in stats])
        ctes = np.array([q['cte_mean'] for q in stats])
        std = float(times.std(ddof=1)) if len(times) > 1 else float('nan')
        summary = dict(name=name, laps=len(stats), best=float(times.min()),
                       mean=float(times.mean()), std=std,
                       cte=float(np.nanmean(ctes)),
                       cte_max=float(np.nanmax([q['cte_max'] for q in stats])))
        print(f"  요약: 온전한 랩 {summary['laps']}개  최고 {summary['best']:.2f}s  "
              f"평균 {summary['mean']:.2f}s ± {std:.2f}s  횡오차 평균 "
              f"{summary['cte'] * 100:.1f}㎝, 최대 {summary['cte_max'] * 100:.1f}㎝")
    else:
        print('  요약: 온전한 랩이 없다')

    if top:
        rows = worst_sections(log, cte, cum[idx], top)
        if rows:
            print('  횡오차 큰 구간 (--zone 에 쓰는 s):')
            for p95, lo, v, n, x, y in rows:
                print(f'    s {lo:5.1f}~{lo + SECTION_M:5.1f}m  95% {p95 * 100:5.1f}㎝  '
                      f'속도 중앙 {v:4.2f}㎧  ({x:+6.2f},{y:+6.2f})  표본 {n}')
    return summary


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='*', help='주행 폴더 (logs/<주행>)')
    ap.add_argument('--last', type=int, default=0, help='최근 N 주행')
    ap.add_argument('--wp', help='웨이포인트 csv 를 직접 지정')
    ap.add_argument('--top', type=int, default=5, help='횡오차 큰 구간 개수 (0=끔)')
    args = ap.parse_args()

    runs = list(args.runs)
    if args.last:
        found = [d for d in sorted(glob.glob(os.path.join(ROOT, 'logs', '20*')))
                 if os.path.exists(os.path.join(d, 'driven_path.csv'))]
        runs += found[-args.last:]
    if not runs:
        ap.error('주행 폴더나 --last 를 주세요')

    summaries = [s for s in (report(r, args.wp, args.top) for r in runs) if s]
    if len(summaries) > 1:
        print('\n=== 비교 (시간은 온전한 랩만)')
        print('  주행                         랩  최고     평균 ± 산포        횡오차평균/최대')
        for s in summaries:
            print(f"  {s['name']:<28} {s['laps']:2d}  {s['best']:6.2f}s  "
                  f"{s['mean']:6.2f}s ± {s['std']:5.2f}s  "
                  f"{s['cte'] * 100:5.1f} / {s['cte_max'] * 100:5.1f}㎝")


if __name__ == '__main__':
    main()
