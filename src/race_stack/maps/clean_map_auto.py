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

사용법
──────
    python3 src/race_stack/maps/clean_map_auto.py --map our_second
    python3 src/race_stack/maps/clean_map_auto.py --map our_second --radius 5

    --radius N   opening 반경 [px]. 클수록 과감히 지운다 (기본 4 = 20cm)
    --fill N     복도 안 점 노이즈를 메우는 closing 반경 [px] (기본 2, 0=끔)
    --out NAME   출력 이름 (기본 <맵>_clean)
    --preview    결과 PNG 만 만들고 맵은 안 쓴다 (검수용)

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='맵 이름 (확장자 없이)')
    ap.add_argument('--out', default=None, help='출력 이름 (기본 <맵>_clean)')
    ap.add_argument('--radius', type=int, default=4, help='opening 반경 px (기본 4)')
    ap.add_argument('--fill', type=int, default=2, help='복도 내 점노이즈 메우기 반경 px')
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

    # ① 가는 빛살 제거
    op = ndi.binary_opening(free, structure=disk(args.radius))

    # ② 가장 큰 연결 성분 = 트랙 복도 (닫힌 복도라 하나로 이어진다)
    lbl, n = ndi.label(op)
    if n == 0:
        sys.exit('✗ opening 후 남은 자유공간이 없습니다. --radius 를 줄이세요.')
    sizes = ndi.sum(op, lbl, range(1, n + 1))
    track = (lbl == (1 + int(np.argmax(sizes))))
    print(f'  opening(r={args.radius}) → 성분 {n}개, 최대 성분 채택 '
          f'{track.sum()} ({100 * track.sum() / free.sum():.1f}% 유지)')

    # ③ 복도 안 점 노이즈 메우기 (SLAM 잔점이 벽으로 남으면 라인 추출을 방해한다)
    if args.fill > 0:
        before = track.sum()
        track = ndi.binary_closing(track, structure=disk(args.fill))
        track = ndi.binary_fill_holes(track) & ndi.binary_dilation(track, disk(1))
        print(f'  fill(r={args.fill}) → +{track.sum() - before}px')

    # ④ 출력: 트랙만 자유, 나머지 전부 벽 (미지 없음 — 기존 _clean 규약)
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

    print('\n다음 단계:')
    print(f'  1) {out_name}_preview.png 를 눈으로 확인 '
          f'(트랙이 잘렸으면 --radius 를 줄일 것)')
    print(f'  2) 레이싱라인:  ./race.sh waypoints-tumftm {out_name}')


if __name__ == '__main__':
    main()
