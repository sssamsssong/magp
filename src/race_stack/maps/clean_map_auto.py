#!/usr/bin/env python3
"""
clean_map_auto.py — SLAM 맵을 **화면 없이** 정제한다 (clean_map.py 의 비대화형 판).

왜 있나
───────
clean_map.py 는 마우스로 폴리곤을 찍는 GUI 라 디스플레이가 필요하다. 맥북에서
SSH 로만 붙어 작업할 때는 못 쓴다. 그런데 우리 맵에서 지워야 할 것은 대부분
**트랙 경계 틈으로 빔이 새어 나가 만든 가는 빛살**이고, 이건 모양이 뚜렷해서
자동으로 걸러진다:

    빛살   폭 1~3px 의 가는 선   → opening 으로 사라진다
    트랙   폭 1m(=20px) 이상     → opening 후에도 남는다

그래서 opening 으로 가는 것을 지우고, 남은 것 중 **가장 큰 연결 성분**만
트랙으로 채택한다. 트랙은 닫힌 복도라 하나의 성분으로 이어져 있다.

출력 형식
─────────
기존 `_clean` 맵과 같게 맞춘다 — **미지(205)를 남기지 않는다.**
    자유(254)  채택된 트랙 복도
    벽(0)      그 밖 전부
clean_map.py 의 "외부 폴리곤 밖은 전부 벽" 과 같은 규약이고, 이렇게 해야
경계에 틈이 없어 gen_raceline 이 트랙을 추출할 수 있다.

★★ 위치추정용 맵 `<출력>_loc` 도 같이 쓴다 (2026-08-21)
──────────────────────────────────────────────────────
"밖은 전부 벽" 규약은 gen_raceline 에는 필요하지만 **AMCL 을 퇴화시킨다.**
likelihood field 는 빔 끝점에서 가장 가까운 점유 셀까지의 거리로 점수를
매기는데, 외부가 통째로 점유라서 **끝점이 트랙 밖 어디에 떨어져도 만점**이다.
경계 틈으로 빔이 새는 코너(맵핑 때 빛살이 생긴 바로 그 자리)에서는 실제
스캔도 틈을 통과하므로, 자세를 밖으로 밀어낸 가설이 진짜 자세와 같은 점수를
받는다 — 08/21 왼쪽 위 코너에서 매 바퀴 추정이 맵 밖으로 튀어나간 원인.
그래서 위치추정 전용 맵을 하나 더 낸다:
    자유(254)  트랙 복도 (동일)
    벽(0)      **원본 맵의 실측 벽만**
    미지(205)  그 밖 전부  ← likelihood field 의 거리 계산에서 빠진다
race.launch.py 가 `<map_name>_loc.yaml` 이 있으면 AMCL·auto_localize·
localization_monitor 에만 이걸 물린다. 컨트롤러·회피·raceline 은 계속
`_clean` 을 본다 (틈이 자유공간이 되면 그쪽으로 회피 후보가 샌다).

사용법
──────
    python3 src/race_stack/maps/clean_map_auto.py --map our_second
    python3 src/race_stack/maps/clean_map_auto.py --map our_second --radius 5

    --radius N    opening 반경 [px]. 클수록 과감히 지운다 (기본 4 = 20cm)
    --pad N       밴드 여유 [px]. 벽을 포함할 만큼 (기본 6 = 30cm)
    --hole-min N  이보다 큰 구멍은 인필드로 보고 남긴다 [px] (기본 400)
    --out NAME    출력 이름 (기본 <맵>_clean)
    --preview     결과 PNG 만 만들고 맵은 안 쓴다 (검수용)

동작 (clean_map.py 의 폴리곤 방식을 자동화한 것)
────────────────────────────────────────────────
  ① opening 으로 빛살을 지워 **씨앗** 을 얻는다
     — 이 마스크를 그대로 쓰면 가장자리가 opening 자국으로 울퉁불퉁하다
  ② 씨앗에서 외부/내부 경계를 만들어 **밴드** 를 정의한다
     (clean_map.py 의 외부·내부 폴리곤에 해당)
  ③ 밴드 안에서는 **원본 픽셀** 을 쓴다 → 벽이 선명하다
  ④ 복도 안 점 노이즈만 메우고, 인필드 같은 큰 구멍은 남긴다
     — 인필드가 없으면 gen_raceline 이 내곽 윤곽을 못 찾는다

★ 결과 PNG(<출력>_preview.png)를 **반드시 눈으로 확인할 것.** 트랙이 잘려
  나갔으면 --radius 를 줄이고, 빛살이 남았으면 키운다.
"""
import argparse
import os
import shutil
import sys

import numpy as np
import scipy.ndimage as ndi
from PIL import Image

VAL_FREE = 254
VAL_WALL = 0

_DIR = os.path.dirname(os.path.abspath(__file__))


def disk(r):
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    return (yy * yy + xx * xx) <= r * r


def write_loc_map(a, track, src_name, out_name):
    """위치추정용 맵 <out_name>_loc.pgm/.yaml 을 쓴다 (모듈 주석 참조).

    a      원본 맵 배열 (0=벽, 205=미지, 254=자유)
    track  채택된 트랙 복도 마스크
    """
    loc = np.full(a.shape, 205, np.uint8)   # 기본: 미지
    loc[a == VAL_WALL] = VAL_WALL           # 실측 벽만 점유
    loc[track] = VAL_FREE                   # 트랙 복도 (복도 안 순간 장애물 제거)
    loc_name = out_name + '_loc'
    loc_pgm = os.path.join(_DIR, loc_name + '.pgm')
    Image.fromarray(loc).save(loc_pgm)

    src_yaml = os.path.join(_DIR, src_name + '.yaml')
    if not os.path.exists(src_yaml):
        print(f'저장: {loc_pgm}   (yaml 원본이 없어 복사 못 함)')
        return
    with open(src_yaml) as f:
        txt = f.read()
    txt = txt.replace(src_name + '.pgm', loc_name + '.pgm')
    # free_thresh 가 0.196(=미지 205 의 점유확률)보다 크면 map_server 가 205 를
    # "자유" 로 분류해 RViz 에서 미지가 안 보인다. AMCL 점수에는 무관하지만
    # (likelihood field 는 점유 셀만 본다) 눈으로 검수할 수 있게 맞춘다.
    import re
    txt, n = re.subn(r'free_thresh:\s*[\d.]+', 'free_thresh: 0.196', txt)
    if n == 0:
        txt += '\nfree_thresh: 0.196\n'
    loc_yaml = os.path.join(_DIR, loc_name + '.yaml')
    with open(loc_yaml, 'w') as f:
        f.write(txt)
    print(f'저장: {loc_pgm}\n      {loc_yaml}   (위치추정용 — AMCL 만 본다)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='맵 이름 (확장자 없이)')
    ap.add_argument('--out', default=None, help='출력 이름 (기본 <맵>_clean)')
    ap.add_argument('--radius', type=int, default=4, help='opening 반경 px (기본 4)')
    ap.add_argument('--pad', type=int, default=6,
                    help='밴드 여유 px — 벽을 포함할 만큼 (기본 6 = 30cm)')
    ap.add_argument('--hole-min', type=int, default=400, dest='hole_min',
                    help='이보다 큰 구멍은 인필드로 보고 남긴다 [px] (기본 400)')
    ap.add_argument('--preview', action='store_true', help='PNG 만 만들고 맵은 안 쓴다')
    args = ap.parse_args()

    src_pgm = os.path.join(_DIR, args.map + '.pgm')
    src_yaml = os.path.join(_DIR, args.map + '.yaml')
    if not os.path.exists(src_pgm):
        sys.exit(f'✗ 맵 없음: {src_pgm}')
    out_name = args.out or (args.map + '_clean')

    a = np.array(Image.open(src_pgm))
    free = (a == VAL_FREE)
    print(f'원본 {a.shape[1]}x{a.shape[0]}px  자유 {free.sum()}  벽 {(a == VAL_WALL).sum()}')

    # ① 가는 빛살 제거 → 트랙 복도의 **씨앗**
    #    여기서 나온 마스크를 그대로 쓰면 가장자리가 opening 자국으로 울퉁불퉁하다.
    #    씨앗은 "트랙이 어디냐" 를 정하는 데만 쓰고, 실제 픽셀은 ③에서 원본을 쓴다.
    op = ndi.binary_opening(free, structure=disk(args.radius))
    lbl, n = ndi.label(op)
    if n == 0:
        sys.exit('✗ opening 후 남은 자유공간이 없습니다. --radius 를 줄이세요.')
    sizes = ndi.sum(op, lbl, range(1, n + 1))
    seed = (lbl == (1 + int(np.argmax(sizes))))
    print(f'  ① opening(r={args.radius}) → 성분 {n}개, 최대 성분 채택 {seed.sum()}px')

    # ② clean_map.py 의 외부/내부 폴리곤에 해당하는 밴드를 자동으로 만든다
    #    outer = 트랙 ∪ 인필드,  infield = 그중 안쪽 섬
    outer = ndi.binary_fill_holes(seed)
    infield = outer & ~seed
    band = (ndi.binary_dilation(outer, disk(args.pad))
            & ~ndi.binary_erosion(infield, disk(args.pad)))
    print(f'  ② 밴드 생성 (pad={args.pad}px) — 인필드 {infield.sum()}px')

    # ③ 밴드 안에서는 **원본 픽셀** 을 쓴다 → 벽이 선명하게 남는다
    keep = band & free
    l2, m = ndi.label(keep)
    if m == 0:
        sys.exit('✗ 밴드 안에 자유공간이 없습니다.')
    s2 = ndi.sum(keep, l2, range(1, m + 1))
    keep = (l2 == (1 + int(np.argmax(s2))))
    print(f'  ③ 원본 픽셀 채택 {keep.sum()}px ({100 * keep.sum() / free.sum():.1f}% 유지)')

    # ④ 복도 안 점 노이즈만 메운다. 인필드 같은 **큰 구멍은 남긴다** —
    #    그게 없으면 gen_raceline 이 내곽 윤곽을 못 찾아 트랙 추출에 실패한다.
    filled = ndi.binary_fill_holes(keep)
    holes = filled & ~keep
    hl, hn = ndi.label(holes)
    track = filled
    n_big = 0
    if hn:
        hs = ndi.sum(holes, hl, range(1, hn + 1))
        for i, sz in enumerate(hs, start=1):
            if sz >= args.hole_min:
                track = track & ~(hl == i)
                n_big += 1
    print(f'  ④ 구멍 {hn}개 중 {n_big}개 유지(인필드), 나머지 메움')

    # ⑤ 출력: 트랙만 자유, 나머지 전부 벽 (미지 없음 — 기존 _clean 규약)
    out = np.full(a.shape, VAL_WALL, np.uint8)
    out[track] = VAL_FREE
    print(f'결과  자유 {100 * track.sum() / out.size:.1f}%  '
          f'벽 {100 * (out == VAL_WALL).sum() / out.size:.1f}%  (미지 0%)')

    # 검수용 PNG — 빨강이 지워진 영역
    prev = np.full(a.shape + (3,), 128, np.uint8)
    prev[free & ~track] = (255, 80, 80)
    prev[track] = (255, 255, 255)
    prev[a == VAL_WALL] = (0, 0, 0)
    prev_path = os.path.join(_DIR, out_name + '_preview.png')
    Image.fromarray(prev).save(prev_path)
    print(f'검수용 PNG: {prev_path}   (빨강 = 지워진 영역)')

    if args.preview:
        print('--preview 라 맵은 쓰지 않았습니다.')
        return

    dst_pgm = os.path.join(_DIR, out_name + '.pgm')
    Image.fromarray(out).save(dst_pgm)
    if os.path.exists(src_yaml):
        dst_yaml = os.path.join(_DIR, out_name + '.yaml')
        shutil.copy(src_yaml, dst_yaml)
        with open(dst_yaml) as f:
            txt = f.read()
        with open(dst_yaml, 'w') as f:
            f.write(txt.replace(args.map + '.pgm', out_name + '.pgm'))
        print(f'저장: {dst_pgm}\n      {dst_yaml}')
    else:
        print(f'저장: {dst_pgm}   (yaml 원본이 없어 복사 못 함)')

    # ⑤b 위치추정용 맵 (<출력>_loc) — 모듈 주석 "위치추정용 맵" 참조.
    #    실측 벽(원본 0)만 점유로 남기고 그 밖은 미지(205). 트랙 자유가
    #    마지막에 덮으므로 맵핑 때 복도 안에 찍힌 순간 장애물(사람 등)은
    #    지워진다 — 실제 경기 때는 없는 물체다.
    write_loc_map(a, track, args.map, out_name)

    print('\n다음 단계:')
    print(f'  1) {out_name}_preview.png 를 눈으로 확인 '
          f'(트랙이 잘렸으면 --radius 를 줄일 것)')
    # ※ ./race.sh waypoints-tumftm 은 2026-08-10 현재 main 에서 ImportError 로
    #   죽는다 (gen_raceline 재작성에서 apply_accel_brake_sweep → calc_vel_profile).
    #   고쳐지면 그쪽이 더 낫다.
    print(f'  2) 레이싱라인:  cd src/race_stack && '
          f'python3 -m race_stack.gen_raceline --map {out_name}')


if __name__ == '__main__':
    main()
