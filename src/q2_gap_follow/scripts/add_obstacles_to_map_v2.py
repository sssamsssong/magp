#!/usr/bin/env python3
"""
v2: floodfill로 트랙 안 자유공간만 골라서 장애물 배치.
출발점 (월드 0,0 또는 --start_x/y) 에서 도달 가능한 흰색 영역만 자유로 인정.
"""
import argparse
from pathlib import Path
import random
import numpy as np
from PIL import Image
import yaml
from collections import deque


def world_to_pixel(x_w, y_w, origin, resolution, img_height):
    col = int((x_w - origin[0]) / resolution)
    row = img_height - 1 - int((y_w - origin[1]) / resolution)
    return col, row


def pixel_to_world(col, row, origin, resolution, img_height):
    x_w = col * resolution + origin[0]
    y_w = (img_height - 1 - row) * resolution + origin[1]
    return x_w, y_w


def floodfill_track(arr, seed_col, seed_row, free_threshold=250):
    """seed에서 시작해서 자유공간만 따라 floodfill, 도달 가능한 픽셀만 True."""
    H, W = arr.shape
    visited = np.zeros((H, W), dtype=bool)
    if not (0 <= seed_col < W and 0 <= seed_row < H):
        return visited
    if arr[seed_row, seed_col] < free_threshold:
        # seed가 자유공간이 아니면 근처 자유공간 찾기
        for dr in range(-50, 51, 5):
            for dc in range(-50, 51, 5):
                r, c = seed_row + dr, seed_col + dc
                if 0 <= r < H and 0 <= c < W and arr[r, c] >= free_threshold:
                    seed_row, seed_col = r, c
                    break
            else:
                continue
            break
    if arr[seed_row, seed_col] < free_threshold:
        return visited

    q = deque([(seed_row, seed_col)])
    visited[seed_row, seed_col] = True
    while q:
        r, c = q.popleft()
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and not visited[nr, nc]:
                if arr[nr, nc] >= free_threshold:
                    visited[nr, nc] = True
                    q.append((nr, nc))
    return visited


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--n', type=int, default=2)
    ap.add_argument('--size', type=float, default=0.5)
    ap.add_argument('--clearance', type=float, default=0.6)
    ap.add_argument('--start_x', type=float, default=0.0)
    ap.add_argument('--start_y', type=float, default=0.0)
    ap.add_argument('--seed', type=int, default=None)
    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)
        np.random.seed(args.seed)

    yaml_in = Path(args.map + '.yaml')
    with open(yaml_in) as f:
        meta = yaml.safe_load(f)
    resolution = meta['resolution']
    origin = meta['origin']
    png_in = yaml_in.parent / meta['image']

    img = Image.open(png_in).convert('L')
    arr = np.array(img)
    H, W = arr.shape

    # 1) 트랙 안 영역 floodfill
    seed_col, seed_row = world_to_pixel(
        args.start_x, args.start_y, origin, resolution, H)
    print(f'출발점 픽셀: ({seed_col}, {seed_row})')
    track = floodfill_track(arr, seed_col, seed_row)
    n_track = track.sum()
    print(f'트랙 안 자유공간 픽셀: {n_track} ({100*n_track/(H*W):.1f}%)')

    if n_track == 0:
        print('ERROR: 트랙 영역을 못 찾음. start_x/start_y를 트랙 안으로 지정하세요.')
        return

    half_px = int((args.size / 2.0) / resolution)
    clear_px = int(args.clearance / resolution)

    # 2) 트랙 안 + 벽에서 충분히 떨어진 픽셀 찾기
    # 벽까지 거리 = 트랙 영역의 erosion
    from scipy.ndimage import binary_erosion
    se = np.ones((2*(half_px+clear_px)+1, 2*(half_px+clear_px)+1))
    safe_zone = binary_erosion(track, structure=se)
    n_safe = safe_zone.sum()
    print(f'안전 배치 가능 픽셀: {n_safe}')

    if n_safe == 0:
        print('ERROR: clearance 너무 큼. 줄이거나 size를 줄이세요.')
        return

    safe_rows, safe_cols = np.where(safe_zone)

    # 3) 무작위 샘플링 + 장애물끼리 떨어뜨림
    chosen = []
    min_sep_px = int(3.0 / resolution)  # 장애물끼리 3m 떨어뜨림
    max_tries = 5000
    for _ in range(max_tries):
        if len(chosen) >= args.n:
            break
        idx = random.randint(0, len(safe_rows) - 1)
        row, col = safe_rows[idx], safe_cols[idx]
        ok = True
        for c0, r0, _, _ in chosen:
            if (col - c0)**2 + (row - r0)**2 < min_sep_px**2:
                ok = False
                break
        if ok:
            xw, yw = pixel_to_world(col, row, origin, resolution, H)
            chosen.append((col, row, xw, yw))

    if len(chosen) < args.n:
        print(f'경고: {args.n}개 중 {len(chosen)}개만 배치')

    # 4) 그리기
    arr_out = arr.copy()
    for col, row, xw, yw in chosen:
        arr_out[row - half_px:row + half_px + 1,
                col - half_px:col + half_px + 1] = 0
        print(f'  장애물: 월드 ({xw:+.2f}, {yw:+.2f}) m / 픽셀 ({col}, {row})')

    out_png = Path(args.out + '.png')
    out_yaml = Path(args.out + '.yaml')
    Image.fromarray(arr_out).save(out_png)

    meta_out = dict(meta)
    meta_out['image'] = out_png.name
    with open(out_yaml, 'w') as f:
        yaml.safe_dump(meta_out, f, default_flow_style=False)

    print(f'\n저장: {out_png}, {out_yaml}')


if __name__ == '__main__':
    main()
