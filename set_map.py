#!/usr/bin/env python3
"""set_map.py — 맵 목록에서 골라 race_script.yaml 의 map_name:/waypoints: 만 바꾼다.

init_pose 는 안 건드린다. joy2(--bag, 마찰계수 측정용 조이스틱 주행)처럼
자율주행이 필요 없는 용도 전용 — 출발 위치를 몰라도(AMCL을 안 쓰므로)
상관없다. 디스플레이가 전혀 필요 없어 순수 SSH 에서 바로 된다
(set_init_pose.py --gui 와 달리 matplotlib 창을 안 띄운다).

자율주행(q1/q2/q3/race/quali)에 쓸 맵을 바꾸는 거라면 이 스크립트가 아니라
set_init_pose.py(= ./race.sh start_pos)를 쓸 것 — 그쪽은 맵을 바꾸면서
init_pose 도 같이 잡아준다. 이걸 자율주행 전에 쓰면 AMCL 초기 위치가
이전 맵 기준으로 남아 수렴을 못 할 수 있다.

사용법:
    python3 set_map.py                       맵 목록에서 번호/이름으로 고르기
    python3 set_map.py --map <이름>           바로 지정 (웨이포인트는 자동/목록)
    python3 set_map.py --map <이름> --wp-file <이름>   둘 다 직접 지정
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import set_init_pose as sip  # noqa: E402  (경로 삽입 뒤에 임포트해야 한다)


def pick_wp_for_map(name, cur_wp):
    """--map 은 줬지만 --wp-file 은 안 줬을 때 — pick_map_interactive 의
    "맵 하나에 웨이포인트 여러 개면 고르게, 하나면 자동" 규칙을 그대로 쓴다."""
    wps = sorted(f[:-4] for f in os.listdir(sip.WPS_DIR)
                 if f.endswith('.csv')) if os.path.isdir(sip.WPS_DIR) else []
    cand = [w for w in wps if w.startswith(name)]
    if not cand:
        print(f'  ⚠ {name} 로 시작하는 웨이포인트가 없습니다 — waypoints: 는 안 건드립니다')
        return cur_wp if cur_wp.startswith(name) else ''
    if len(cand) == 1:
        return cand[0]
    print('\n  웨이포인트 선택:')
    for i, w in enumerate(cand, 1):
        print(f'    {i:3d}. {w}')
    while True:
        v = input('  번호 [1]: ').strip() or '1'
        if v.isdigit() and 1 <= int(v) <= len(cand):
            return cand[int(v) - 1]
        print('    다시.')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default='', help='맵 이름 (생략하면 목록에서 고름)')
    ap.add_argument('--wp-file', default='', help='웨이포인트 이름 (생략하면 자동/목록)')
    args = ap.parse_args()

    cfg = sip.load_cfg()
    cur_map = cfg.get('map_name', '')
    cur_wp = cfg.get('waypoints', '')

    if args.map:
        name = args.map
        wp = args.wp_file or pick_wp_for_map(name, cur_wp)
    else:
        name, wp = sip.pick_map_interactive(cur_map, cur_wp)

    if name == cur_map and wp == cur_wp:
        print(f'변경 없음 — 이미 map_name={name}, waypoints={wp or "(없음)"}')
        return

    print('\nrace_script.yaml 갱신:')
    pairs = [('map_name', name)]
    if wp:
        pairs.append(('waypoints', wp))
    sip.write_values(pairs)
    print('\n※ init_pose 는 안 건드렸습니다. 자율주행에 쓸 거면 ./race.sh start_pos 로')
    print('  출발 위치도 다시 잡으세요 — joy2 처럼 AMCL 안 쓰는 용도면 필요 없습니다.')


if __name__ == '__main__':
    main()
