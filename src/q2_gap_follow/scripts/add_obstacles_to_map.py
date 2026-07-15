#!/usr/bin/env python3
"""
add_obstacles_to_map.py
=======================
원본 맵 PNG에 0.5m × 0.5m 가상 장애물 N개를 무작위로 박아서
새 맵 PNG/YAML로 저장. 시뮬에서 실제 LiDAR가 이 픽셀을 진짜 벽처럼 보게 됨.

사용법:
    python3 add_obstacles_to_map.py \
        --map ../maps/levine \
        --out ../maps/levine_q2 \
        --n 2 --seed 42

원리:
    - YAML의 resolution (m/픽셀)과 origin (실세계 → 픽셀) 변환
    - PNG의 자유공간(밝은 픽셀) 중에서 벽으로부터 충분히 떨어진 위치 샘플링
    - 0.5m × 0.5m 영역을 0(검정=점유)으로 채움
    - 새 PNG + 새 YAML 생성
"""

import argparse
import os
import random
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


def world_to_pixel(x_w, y_w, origin, resolution, img_height):
    """월드 좌표 (m) → 이미지 픽셀 (col, row).
    nav2 map_server 컨벤션: origin은 이미지 좌하단의 월드 좌표.
    """
    col = int((x_w - origin[0]) / resolution)
    row = img_height - 1 - int((y_w - origin[1]) / resolution)
    return col, row


def pixel_to_world(col, row, origin, resolution, img_height):
    x_w = col * resolution + origin[0]
    y_w = (img_height - 1 - row) * resolution + origin[1]
    return x_w, y_w


def is_safe_position(arr, col, row, half_px, wall_clearance_px,
                     free_threshold=250):
    """주변에 자유공간(밝은 픽셀)만 있는지 검사.

    nav2 map_server 규약:
      - 0~50 정도 어두움 = 벽/점유
      - 100~200 중간 그레이 = 미지(unknown) — 트랙 밖 영역
      - 250 이상 밝음 = 자유(free)
    트랙 안에서만 장애물을 박으려면 free_threshold를 250 정도로 빡빡하게.
    """
    H, W = arr.shape
    pad = half_px + wall_clearance_px
    if col - pad < 0 or col + pad >= W:
        return False
    if row - pad < 0 or row + pad >= H:
        return False
    region = arr[row - pad:row + pad + 1, col - pad:col + pad + 1]
    # 전체 영역이 자유공간(흰색)이어야 함
    return region.min() >= free_threshold


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='입력 맵 prefix (확장자 제외)')
    ap.add_argument('--out', required=True, help='출력 맵 prefix (확장자 제외)')
    ap.add_argument('--n', type=int, default=2, help='장애물 개수')
    ap.add_argument('--size', type=float, default=0.5, help='장애물 한 변 (m)')
    ap.add_argument('--clearance', type=float, default=0.6,
                    help='벽과의 최소 거리 (m)')
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--positions', type=str, default=None,
                    help='직접 지정 시 "x1,y1;x2,y2" 형식 (월드 좌표 m)')
    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)
        np.random.seed(args.seed)

    yaml_in = Path(args.map + '.yaml')
    with open(yaml_in) as f:
        meta = yaml.safe_load(f)
    resolution = meta['resolution']
    origin = meta['origin']
    img_name = meta['image']

    png_in = yaml_in.parent / img_name
    img = Image.open(png_in).convert('L')
    arr = np.array(img)
    H, W = arr.shape

    half_px = int((args.size / 2.0) / resolution)
    clear_px = int(args.clearance / resolution)

    chosen = []

    if args.positions:
        # 사용자 지정 위치
        for p in args.positions.split(';'):
            x_s, y_s = p.split(',')
            xw, yw = float(x_s), float(y_s)
            col, row = world_to_pixel(xw, yw, origin, resolution, H)
            chosen.append((col, row, xw, yw))
    else:
        # 무작위로 자유공간 샘플
        max_tries = 5000
        attempts = 0
        while len(chosen) < args.n and attempts < max_tries:
            attempts += 1
            col = random.randint(half_px + clear_px, W - half_px - clear_px - 1)
            row = random.randint(half_px + clear_px, H - half_px - clear_px - 1)
            if not is_safe_position(arr, col, row, half_px, clear_px):
                continue
            # 기존 장애물과도 떨어뜨림
            ok = True
            min_sep_px = int(2.0 / resolution)  # 2m
            for c0, r0, _, _ in chosen:
                if (col - c0)**2 + (row - r0)**2 < min_sep_px**2:
                    ok = False
                    break
            if not ok:
                continue
            xw, yw = pixel_to_world(col, row, origin, resolution, H)
            chosen.append((col, row, xw, yw))

        if len(chosen) < args.n:
            print(f'경고: {args.n}개 중 {len(chosen)}개만 배치 성공 '
                  f'(자유공간 부족 또는 clearance 너무 큼)')

    # 장애물 칠하기
    arr_out = arr.copy()
    for col, row, xw, yw in chosen:
        arr_out[row - half_px:row + half_px + 1,
                col - half_px:col + half_px + 1] = 0
        print(f'  장애물 추가: 월드 ({xw:+.2f}, {yw:+.2f}) m  '
              f'/ 픽셀 ({col}, {row})')

    out_png = Path(args.out + '.png')
    out_yaml = Path(args.out + '.yaml')
    Image.fromarray(arr_out).save(out_png)

    meta_out = dict(meta)
    meta_out['image'] = out_png.name
    with open(out_yaml, 'w') as f:
        yaml.safe_dump(meta_out, f, default_flow_style=False)

    print(f'\n저장 완료:\n  {out_png}\n  {out_yaml}')
    print(f'장애물 위치 (m): {[(round(c[2],2), round(c[3],2)) for c in chosen]}')


if __name__ == '__main__':
    main()
