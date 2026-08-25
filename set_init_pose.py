#!/usr/bin/env python3
"""
set_init_pose.py — 맵 이미지 위에서 AMCL 초기 포즈(위치+yaw)를 직접 골라
                   race_script.yaml 의 localization.init_x/init_y/init_yaw 에 써넣는다.

초기 포즈는 맵 yaml 이 아니라 race_script.yaml 의 [localization] 에 있다.
(race.launch.py 가 이 값을 AMCL 의 initial_pose.x/y/yaw 로 주입한다)

★ 2026-08-24 — 여기서 고른 맵/웨이포인트로 race_script.yaml 의 map_name:/
  waypoints: 도 같이 갱신한다(--write 시). 예전엔 init_pose 만 쓰고 이
  둘은 손으로 고쳐야 했다 — 이 도구가 이미 맵을 고르는 과정을 거치므로
  (인자 없이 실행하면 맵 목록에서 고름, 또는 --map/--wp-file) 그 선택을
  트랙 설정에도 그대로 반영한다.
  ※ 새 맵을 딴 뒤 이 스크립트를 노트북에서 돌리려면, 그 전에 젯슨에서
  savemap/clean 한 결과(맵 파일)를 커밋·푸시하고 노트북에서 git pull 해야
  한다 — 맵 파일 자체는 두 차가 공유하지만(git), race_script.yaml 의
  map_name:/waypoints: 값 자체는 커밋하지 말 것(CLAUDE.md — 두 차가 다른
  트랙을 볼 수 있어 로컬 전용으로 둔다).

──────────────────────────────────────────────────────────────
사용법 3가지 — 상황에 맞게 하나만 쓰면 된다
──────────────────────────────────────────────────────────────
1) 미리보기 이미지로 좌표 읽기 (디스플레이 없이, SSH 에서)

       python3 set_init_pose.py                    # 격자 미리보기 PNG 저장
       → /tmp/init_pose_<map>.png 를 열어 축(미터)에서 원하는 지점을 읽는다
       → 현재 설정된 초기 포즈가 빨간 화살표로 같이 그려진다

2) 좌표를 알았으면 바로 지정 (--write 없으면 미리보기만, 파일 안 건드림)

       python3 set_init_pose.py --x 12.2 --y 3.5 --yaw -152 --deg
       python3 set_init_pose.py --x 12.2 --y 3.5 --yaw -152 --deg --write

   yaw 를 각도로 재기 귀찮으면 "바라볼 지점"으로 줘도 된다:

       python3 set_init_pose.py --x 12.2 --y 3.5 --to-x 10.0 --to-y 2.4 --write

   픽셀 좌표로 주고 싶으면 (--px/--py, 이미지 왼쪽위가 0,0):

       python3 set_init_pose.py --px 340 --py 40 --to-px 300 --to-py 60 --write

3) 웨이포인트에 그냥 붙이기 (가장 안전한 기본값)

       python3 set_init_pose.py --wp 0 --write     # 0번 웨이포인트 + 진행방향 yaw

──────────────────────────────────────────────────────────────
4) 클릭으로 찍기 (젯슨에 모니터가 붙어 있거나 X11 forwarding 될 때만)

       python3 set_init_pose.py --gui
       왼쪽 클릭 1회 = 위치,  2회째 = 바라볼 방향
       w = race_script.yaml 에 쓰기,  r = 다시,  q = 취소

좌표 규약: map 프레임 (m). yaw 는 +x 축 기준 반시계(rad).
        pixel → world :  x = ox + px*res,  y = oy + (H - py)*res
"""
import argparse
import csv
import math
import os
import re
import sys

import numpy as np
import yaml
from PIL import Image

REPO_DIR   = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR   = os.path.join(REPO_DIR, 'src', 'race_stack', 'maps')
WPS_DIR    = os.path.join(REPO_DIR, 'src', 'race_stack', 'waypoints')
SCRIPT_YML = os.path.join(REPO_DIR, 'race_script.yaml')


# ── 로드 ────────────────────────────────────────────────────────
def load_cfg():
    try:
        return yaml.safe_load(open(SCRIPT_YML)) or {}
    except OSError:
        return {}


def load_map(map_name):
    """맵 yaml + 이미지 → (img, res, ox, oy, H, W)"""
    map_yaml = os.path.join(MAPS_DIR, map_name + '.yaml')
    if not os.path.exists(map_yaml):
        sys.exit(f'맵 yaml 없음: {map_yaml}')
    mcfg = yaml.safe_load(open(map_yaml))
    res = float(mcfg['resolution'])
    ox, oy = float(mcfg['origin'][0]), float(mcfg['origin'][1])
    # map_server 와 같은 파일을 본다 — 같은 이름의 png 가 있어도 추측하지 않는다
    img_path = os.path.join(os.path.dirname(map_yaml), mcfg['image'])
    if not os.path.exists(img_path):
        sys.exit(f'맵 이미지 없음: {img_path} (yaml image: 필드 확인)')
    img = np.asarray(Image.open(img_path).convert('L'))
    H, W = img.shape
    return img, res, ox, oy, H, W


def load_waypoints(name):
    """CSV(x,y[,speed]) → (N,2). 없으면 None."""
    path = os.path.join(WPS_DIR, name + '.csv')
    if not name or not os.path.exists(path):
        return None
    pts = []
    with open(path) as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith('#'):
                continue
            try:
                pts.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue
    return np.array(pts) if pts else None


# ── 좌표 변환 ───────────────────────────────────────────────────
def px_to_world(px, py, res, ox, oy, H):
    return ox + px * res, oy + (H - py) * res


def world_to_px(x, y, res, ox, oy, H):
    return (x - ox) / res, H - (y - oy) / res


def cell_value(img, x, y, res, ox, oy, H, W):
    """해당 world 좌표의 픽셀 밝기 (없으면 None). 밝을수록 자유공간."""
    px, py = world_to_px(x, y, res, ox, oy, H)
    ix, iy = int(round(px)), int(round(py))
    if 0 <= ix < W and 0 <= iy < H:
        return int(img[iy, ix])
    return None


# ── race_script.yaml 쓰기 (주석 보존을 위해 줄 단위 치환) ────────
def write_values(pairs, dry=False):
    """pairs: [(key, value_str), ...] — 'key:' 로 시작하는 첫 줄의 값만 바꾼다."""
    with open(SCRIPT_YML) as f:
        lines = f.readlines()

    done = {}
    for key, val in pairs:
        pat = re.compile(rf'^(\s*{re.escape(key)}:\s*)(\[[^\]]*\]|[^\s#]+)(.*)$')
        for i, ln in enumerate(lines):
            m = pat.match(ln)
            if m:
                lines[i] = f'{m.group(1)}{val}{m.group(3)}\n'
                done[key] = (m.group(2), val, i + 1)
                break
        else:
            print(f'⚠ race_script.yaml 에서 "{key}:" 를 못 찾음 — 건너뜀')

    if not dry:
        with open(SCRIPT_YML, 'w') as f:
            f.writelines(lines)
    for key, (old, new, ln) in done.items():
        print(f'  {SCRIPT_YML}:{ln}  {key}: {old} → {new}')
    return done



# ── 대화형 맵 선택 ──────────────────────────────────────────────
def pick_map_interactive(cur_map, cur_wp):
    """인자 없이 실행했을 때 맵 목록에서 고르게 한다.

    gen_waypoints_kappa.select_map_interactive 와 같은 규약이다 — 번호나 이름을
    받고, 그냥 Enter 면 race_script.yaml 의 현재 값을 쓴다.
    """
    maps = sorted({f.rsplit('.', 1)[0] for f in os.listdir(MAPS_DIR)
                   if f.endswith(('.pgm', '.png')) and not f.endswith('_preview.png')})
    if not maps:
        sys.exit(f'맵이 없습니다: {MAPS_DIR}\n  ./race.sh savemap <이름> 으로 먼저 저장하세요.')

    # ★ .csv 만 센다. 웨이포인트는 .csv 와 같은 이름의 .png(미리보기)가 쌍으로
    #   있어서, 확장자만 떼면 같은 이름이 두 번 잡혀 개수가 두 배로 보인다.
    wps = sorted(f[:-4] for f in os.listdir(WPS_DIR)
                 if f.endswith('.csv')) if os.path.isdir(WPS_DIR) else []

    print('\n' + '=' * 56)
    print('  맵 선택   (Enter = 현재 설정 그대로)')
    print('=' * 56)
    for i, m in enumerate(maps, 1):
        mark = ' ←현재' if m == cur_map else ''
        n = sum(1 for w in wps if w.startswith(m))
        print(f'  {i:3d}. {m:<28s} 웨이포인트 {n}개{mark}')
    print('=' * 56)

    while True:
        v = input(f'맵 번호 또는 이름 [{cur_map or "없음"}]: ').strip()
        if not v:
            return cur_map, cur_wp
        if v.isdigit() and 1 <= int(v) <= len(maps):
            name = maps[int(v) - 1]
        elif v in maps:
            name = v
        else:
            print('  그런 맵이 없습니다. 다시.')
            continue
        # 그 맵으로 시작하는 웨이포인트를 찾아 준다 — 하나면 자동, 여럿이면 고르게
        cand = [w for w in wps if w.startswith(name)]
        if not cand:
            print(f'  ⚠ {name} 로 시작하는 웨이포인트가 없습니다 (라인 없이 맵만 봅니다)')
            return name, ''
        if len(cand) == 1:
            print(f'  웨이포인트: {cand[0]}')
            return name, cand[0]
        print('\n  웨이포인트 선택:')
        for i, w in enumerate(cand, 1):
            print(f'    {i:3d}. {w}')
        while True:
            v2 = input(f'  번호 [1]: ').strip() or '1'
            if v2.isdigit() and 1 <= int(v2) <= len(cand):
                return name, cand[int(v2) - 1]
            print('    다시.')


# ── 그리기 ──────────────────────────────────────────────────────
def draw(img, res, ox, oy, H, W, wps, poses, title, out, show=False):
    """poses: [(x, y, yaw, color, label), ...] — extent 를 미터로 줘서 축에서 좌표를 바로 읽는다."""
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    else:
        # ★ SSH 로 들어오면 DISPLAY 가 비어 있어 창을 못 연다. 젯슨에 모니터가
        #   붙어 있으면 로컬 X 세션이 /tmp/.X11-unix/X<N> 으로 살아 있으므로
        #   그걸 잡아준다 (rviz.sh 와 같은 방식). 창은 **젯슨 화면**에 뜬다.
        #   ※ 반드시 pyplot import 전에 해야 한다 — 백엔드가 import 시점에 정해진다.
        if not os.environ.get('DISPLAY'):
            import glob
            for sock in sorted(glob.glob('/tmp/.X11-unix/X*')):
                os.environ['DISPLAY'] = ':' + os.path.basename(sock)[1:]
                break
        if not os.environ.get('DISPLAY'):
            sys.exit('✗ 젯슨에 그래픽 세션이 없습니다 (DISPLAY 없음, /tmp/.X11-unix 비어 있음).\n'
                     '  모니터를 연결하거나, --gui 없이 --x/--y/--yaw 로 값을 직접 주세요.')
        matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    x0, x1 = ox, ox + W * res
    y0, y1 = oy, oy + H * res

    fig, ax = plt.subplots(figsize=(12, max(4.0, 12 * (y1 - y0) / (x1 - x0))))
    ax.imshow(np.flipud(img), cmap='gray', origin='lower',
              extent=[x0, x1, y0, y1], interpolation='nearest')

    if wps is not None:
        ax.plot(np.append(wps[:, 0], wps[0, 0]), np.append(wps[:, 1], wps[0, 1]),
                color='deepskyblue', lw=1.0, alpha=0.7, zorder=2, label='waypoints')
        ax.scatter([wps[0, 0]], [wps[0, 1]], marker='*', s=180, color='deepskyblue',
                   edgecolor='black', zorder=3)

    arrow_len = 0.9
    for (x, y, yaw, color, label) in poses:
        ax.scatter([x], [y], s=90, color=color, edgecolor='black', zorder=6, label=label)
        ax.annotate('', xy=(x + arrow_len * math.cos(yaw), y + arrow_len * math.sin(yaw)),
                    xytext=(x, y), zorder=6,
                    arrowprops=dict(arrowstyle='-|>', color=color, lw=2.5))

    # 1m 격자 — 이 눈금에서 좌표를 읽는다
    ax.set_xticks(np.arange(math.ceil(x0), x1, 1.0))
    ax.set_yticks(np.arange(math.ceil(y0), y1, 1.0))
    ax.grid(True, color='orange', lw=0.4, alpha=0.5)
    ax.tick_params(labelsize=7)
    ax.set_xlabel('map x (m)'); ax.set_ylabel('map y (m)')
    ax.set_title(title, fontsize=10)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_aspect('equal')

    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'미리보기 저장: {out}')
    return fig, ax, plt


# ── GUI 클릭 픽커 ───────────────────────────────────────────────
def run_gui(img, res, ox, oy, H, W, wps, cur, map_name, out, auto_write, wp_name=None):
    PRESET_KEYS = {'up', 'down', 'left', 'right', 'a', 'b', 'x', 'y'}
    PRESET_COLORS = {'up': 'red', 'down': 'orange', 'left': 'gold', 'right': 'lime',
                     'a': 'cyan', 'b': 'deepskyblue', 'x': 'magenta', 'y': 'white'}
    state = {'pos': None, 'yaw': None, 'artists': [], 'sel': None}

    poses = [cur + ('red', 'current init_pose')] if cur else []
    fig, ax, plt = draw(img, res, ox, oy, H, W, wps, poses,
                        f'{map_name} | ①키(↑↓←→·a/b/x/y)로 프리셋 선택 → ②위치·방향 클릭 '
                        '| w=init/start  r=취소  q=종료',
                        out, show=True)

    def clear_transient():
        """진행중인(아직 저장 안 한) 클릭 마커만 지운다. 저장된 프리셋 점은 남는다."""
        for a in state['artists']:
            a.remove()
        state['artists'] = []
        state['pos'] = state['yaw'] = None
        fig.canvas.draw_idle()

    def on_click(ev):
        if ev.inaxes is not ax or ev.xdata is None:
            return
        c = PRESET_COLORS.get(state['sel'], 'lime')
        if state['pos'] is None:
            state['pos'] = (ev.xdata, ev.ydata)
            state['artists'] += ax.plot([ev.xdata], [ev.ydata], 'o', ms=9,
                                        color=c, mec='black', zorder=7)
            tag = f' [프리셋 {state["sel"]}]' if state['sel'] else ''
            print(f'위치 찍음{tag} → 이제 방향 클릭')
        else:
            x, y = state['pos']
            state['yaw'] = math.atan2(ev.ydata - y, ev.xdata - x)
            ann = ax.annotate('', xy=(x + 0.9 * math.cos(state['yaw']),
                                      y + 0.9 * math.sin(state['yaw'])),
                              xytext=(x, y), zorder=7,
                              arrowprops=dict(arrowstyle='-|>', color=c, lw=2.5))
            state['artists'].append(ann)
            if state['sel']:
                # 선택된 프리셋에 저장을 **시도**한다. 실제로 써졌을 때만
                # 점을 영구로 남기고 성공 메시지를 찍는다 — 예전엔 write
                # 가 조용히 실패해도(줄이 없어서) 무조건 "저장됨" 을 찍어서
                # 안 써진 걸 써진 것처럼 오인했다 (2026-08-26 실제로 겪음).
                _, ok = apply_pose(x, y, state['yaw'], img, res, ox, oy, H, W,
                                   True, {'preset': state['sel']})
                if ok:
                    ax.annotate(state['sel'], (x, y), color=c, fontsize=11,
                                fontweight='bold', zorder=8,
                                xytext=(6, 6), textcoords='offset points')
                    state['artists'] = []   # 참조를 버려서 clear 때 안 지워지게 = 영구
                    state['pos'] = state['yaw'] = None
                    print(f'★ 프리셋 [{state["sel"]}] 저장됨 — 다음 프리셋 키 누르고 클릭')
                else:
                    print(f'✗ 프리셋 [{state["sel"]}] 저장 실패 — race_script.yaml 에 '
                          f'preset_{state["sel"]}: 줄이 없습니다. 점은 아직 임시(r=취소, '
                          f'다시 클릭)이고, w 키로도 안 써집니다 — 파일에 그 줄을 먼저 추가하세요.')
                state['sel'] = None
            elif auto_write:
                apply_pose(x, y, state['yaw'], img, res, ox, oy, H, W, True, {},
                           map_name=map_name, wp_name=wp_name)  # (yaw, ok) — 여기선 항상 성공
            else:
                print(f'방향 찍음 ({math.degrees(state["yaw"]):.0f}°) → '
                      'w=init/start, 또는 프리셋 키 먼저 눌렀어야 함')
        fig.canvas.draw_idle()

    def on_key(ev):
        # ★ Caps Lock/Shift 가 켜져 있으면 matplotlib 이 대문자('A' 등)를
        #   준다. PRESET_KEYS 는 소문자만 담고 있어서 대문자는 그냥 씹혀
        #   조용히 아무 반응이 없었다(에러도 없음) — 2026-08-26 실제로 겪음.
        #   화살표는 'up'/'down' 처럼 이름 자체가 대소문자 개념이 없어서
        #   멀쩡히 됐고, a/b/x/y 네 개만 전부 막혀 있었다.
        key = (ev.key or '').lower()
        if key == 'r':
            clear_transient(); state['sel'] = None; print('취소 — 다시')
        elif key == 'q':
            plt.close(fig)
        elif key == 'w':
            if state['pos'] is None or state['yaw'] is None:
                print('위치+방향을 먼저 클릭하세요')
                return
            x, y = state['pos']
            apply_pose(x, y, state['yaw'], img, res, ox, oy, H, W, True, {},
                       map_name=map_name, wp_name=wp_name)  # (yaw, ok) — init/start 는 항상 성공
            plt.close(fig)
        elif key in PRESET_KEYS:
            state['sel'] = key
            clear_transient()   # 진행중 클릭만 리셋 (저장된 프리셋 점은 유지)
            print(f'프리셋 [{key}] 선택 (색 {PRESET_COLORS[key]}) — 위치·방향 클릭하세요')

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()


# ── 적용 ────────────────────────────────────────────────────────
def apply_pose(x, y, yaw, img, res, ox, oy, H, W, do_write, extra,
               map_name=None, wp_name=None):
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))     # -π~π 로 정규화
    v = cell_value(img, x, y, res, ox, oy, H, W)
    print(f'\n초기 포즈:  init_x={x:.4f}  init_y={y:.4f}  init_yaw={yaw:.4f} rad '
          f'({math.degrees(yaw):.1f}°)')
    if v is None:
        print('⚠ 이 좌표는 맵 밖입니다 — AMCL 이 수렴 못 합니다')
    elif v < 128:
        print(f'⚠ 이 좌표의 픽셀값이 {v} (어두움 = 벽/미탐색). 트랙 위인지 확인하세요')

    preset = extra.get('preset')
    if preset:
        # 프리셋 전용: init/start/map 은 건드리지 않고 preset_<key> 만 쓴다.
        pairs = [(f'preset_{preset}', f'[{x:.4f}, {y:.4f}, {yaw:.4f}]')]
        ok = True
        if do_write:
            print(f'race_script.yaml 갱신 (프리셋 {preset}):')
            # ★ write_values 는 **기존 줄만** 값을 바꾼다. race_script.yaml 에
            #   preset_<key>: 줄 자체가 없으면 경고만 찍고 조용히 건너뛰는데,
            #   예전엔 호출자가 그 결과를 안 보고 무조건 "저장됨" 을 찍어서
            #   실제로는 저장 안 됐는데 성공한 것처럼 보였다 (2026-08-26 실제로
            #   겪음 — preset_* 줄이 race_script.yaml 에 아예 없었다).
            done = write_values(pairs)
            ok = f'preset_{preset}' in done
        else:
            print(f'(미리보기) preset_{preset}: [{x:.4f}, {y:.4f}, {yaw:.4f}]')
        return yaw, ok

    pairs = [('init_x', f'{x:.4f}'), ('init_y', f'{y:.4f}'), ('init_yaw', f'{yaw:.4f}')]
    if extra.get('start'):
        pairs += [('start_x', f'{x:.4f}'), ('start_y', f'{y:.4f}')]
    if extra.get('sim'):
        pairs += [('sx', f'{x:.4f}'), ('sy', f'{y:.4f}'), ('stheta', f'{yaw:.4f}')]
    # ★ 2026-08-24 — 여기서 고른 맵/웨이포인트를 race_script.yaml 의
    #   map_name:/waypoints: 에도 같이 쓴다. 예전엔 init_pose 만 갱신하고
    #   이 둘은 안 건드려서, 맵을 새로 딴 뒤엔 결국 에디터로 수동으로
    #   고쳐야 했다(젯슨에서 이러던 것). 이 스크립트가 이미 맵을 고르는
    #   과정을 다 거치므로(pick_map_interactive 또는 --map/--wp-file),
    #   그 선택을 그대로 트랙 설정에도 반영한다.
    #   ※ CLAUDE.md 규칙대로 이 값은 커밋하지 말 것 — 두 차가 공유하는
    #     race_script.yaml 의 로컬 전용 필드다.
    if map_name:
        pairs.append(('map_name', map_name))
    if wp_name:
        pairs.append(('waypoints', wp_name))

    if do_write:
        print('race_script.yaml 갱신:')
        write_values(pairs)
    else:
        print('(--write 를 붙이면 race_script.yaml 에 씁니다. 지금은 미리보기만)')
    return yaw, True


def main():
    cfg = load_cfg()
    def_map = cfg.get('map_name', '')
    def_wp = cfg.get('waypoints', '')

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default=def_map, help=f'맵 이름 (기본: {def_map or "없음"})')
    ap.add_argument('--wp-file', default=def_wp, help=f'웨이포인트 이름 (기본: {def_wp or "없음"})')
    ap.add_argument('--x', type=float); ap.add_argument('--y', type=float)
    ap.add_argument('--px', type=float, help='이미지 픽셀 x (왼쪽위 0,0)')
    ap.add_argument('--py', type=float, help='이미지 픽셀 y')
    ap.add_argument('--yaw', type=float, help='방향 (기본 rad, --deg 면 도)')
    ap.add_argument('--to-x', type=float); ap.add_argument('--to-y', type=float)
    ap.add_argument('--to-px', type=float); ap.add_argument('--to-py', type=float)
    ap.add_argument('--deg', action='store_true', help='--yaw 를 도(°) 로 해석')
    ap.add_argument('--wp', type=int, help='N번 웨이포인트 위치 + 진행방향 yaw 를 사용')
    ap.add_argument('--gui', action='store_true', help='클릭으로 찍기 (디스플레이 필요)')
    ap.add_argument('--write', action='store_true', help='race_script.yaml 에 실제로 쓰기')
    ap.add_argument('--also-start', action='store_true',
                    help='mission.start_x/start_y 도 같은 좌표로 갱신')
    ap.add_argument('--also-sim', action='store_true',
                    help='sim.sx/sy/stheta 도 같은 포즈로 갱신')
    ap.add_argument('--preset', choices=['up', 'down', 'left', 'right',
                                         'a', 'b', 'x', 'y'], default=None,
                    help='이 좌표를 복구 프리셋 joystick.preset_<키> 에 쓴다 '
                         '(init/start 대신). GUI 에선 ↑↓←→·a/b/x/y 키로도 가능')
    ap.add_argument('--out', default='', help='미리보기 PNG 경로')
    args = ap.parse_args()

    # 인자를 하나도 안 줬으면 맵부터 고르게 한다 (--gui 도 자동으로 켠다).
    # "python3 set_init_pose.py" 만 쳤을 때 바로 쓸 수 있어야 한다.
    _given = any(v is not None for v in (args.x, args.y, args.px, args.py,
                                         args.yaw, args.to_x, args.to_y,
                                         args.to_px, args.to_py, args.wp))
    if not _given and not args.gui:
        args.map, args.wp_file = pick_map_interactive(args.map, args.wp_file)
        args.gui = True

    if not args.map:
        sys.exit('맵 이름이 없습니다. --map 을 지정하세요.')

    img, res, ox, oy, H, W = load_map(args.map)
    wps = load_waypoints(args.wp_file)
    out = args.out or f'/tmp/init_pose_{args.map}.png'
    extra = {'start': args.also_start, 'sim': args.also_sim, 'preset': args.preset}

    loc = cfg.get('localization') or {}
    cur = None
    if {'init_x', 'init_y'} <= set(loc):
        cur = (float(loc['init_x']), float(loc['init_y']), float(loc.get('init_yaw', 0.0)))

    print(f'맵 {args.map}: {W}x{H}px @ {res}m/px, origin=({ox}, {oy})')
    print(f'  map 좌표 범위: x {ox:.2f}~{ox + W * res:.2f} m,  y {oy:.2f}~{oy + H * res:.2f} m')
    if cur:
        v = cell_value(img, cur[0], cur[1], res, ox, oy, H, W)
        tag = '맵 밖' if v is None else ('벽/미탐색' if v < 128 else '자유공간')
        print(f'  현재 init_pose: ({cur[0]:.3f}, {cur[1]:.3f}, {cur[2]:.3f} rad) — {tag}')

    if args.gui:
        run_gui(img, res, ox, oy, H, W, wps, cur, args.map, out, args.write,
               wp_name=args.wp_file)
        return

    # ── 위치 결정 ──
    x = y = yaw = None
    if args.wp is not None:
        if wps is None:
            sys.exit(f'웨이포인트를 못 읽었습니다: {args.wp_file}')
        i = args.wp % len(wps)
        j = (i + 3) % len(wps)              # 3점 앞을 보고 진행방향(접선) 계산
        x, y = wps[i]
        yaw = math.atan2(wps[j][1] - y, wps[j][0] - x)
        print(f'  웨이포인트 {i}번 사용 (진행방향은 {j}번 기준)')
    else:
        if args.px is not None and args.py is not None:
            x, y = px_to_world(args.px, args.py, res, ox, oy, H)
        elif args.x is not None and args.y is not None:
            x, y = args.x, args.y

        if args.to_px is not None and args.to_py is not None:
            tx, ty = px_to_world(args.to_px, args.to_py, res, ox, oy, H)
            if x is None:
                sys.exit('--to-px/--to-py 는 위치(--x/--y 또는 --px/--py)와 함께 쓰세요')
            yaw = math.atan2(ty - y, tx - x)
        elif args.to_x is not None and args.to_y is not None:
            if x is None:
                sys.exit('--to-x/--to-y 는 위치(--x/--y 또는 --px/--py)와 함께 쓰세요')
            yaw = math.atan2(args.to_y - y, args.to_x - x)
        elif args.yaw is not None:
            yaw = math.radians(args.yaw) if args.deg else args.yaw

    # ── 좌표가 안 주어지면 미리보기만 ──
    if x is None or yaw is None:
        poses = [cur + ('red', 'current init_pose')] if cur else []
        draw(img, res, ox, oy, H, W, wps, poses,
             f'{args.map} - grid 1m : read (x, y) of the spot you want from the axes', out)
        print('\n이미지에서 좌표를 읽은 뒤:')
        print(f'  python3 set_init_pose.py --x <X> --y <Y> --yaw <각도> --deg --write')
        print(f'  python3 set_init_pose.py --x <X> --y <Y> --to-x <앞쪽X> --to-y <앞쪽Y> --write')
        if wps is not None:
            print(f'  python3 set_init_pose.py --wp 0 --write     # 0번 웨이포인트에 붙이기')
        if x is not None and yaw is None:
            print('\n※ 위치는 받았지만 방향이 없습니다 (--yaw 또는 --to-x/--to-y 필요)')
        return

    yaw, _ok = apply_pose(x, y, yaw, img, res, ox, oy, H, W, args.write, extra,
                          map_name=args.map, wp_name=args.wp_file)

    poses = []
    if cur:
        poses.append(cur + ('red', 'old init_pose'))
    poses.append((x, y, yaw, 'lime', 'new init_pose'))
    draw(img, res, ox, oy, H, W, wps, poses,
         f'{args.map} - init_pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg)', out)
    if not args.write:
        print('확인 후 같은 명령에 --write 를 붙여 실행하세요.')


if __name__ == '__main__':
    main()
