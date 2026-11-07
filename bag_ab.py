#!/usr/bin/env python3
"""bag_ab.py — 같은 bag 을 스위치만 바꿔 재생하고 결과를 **같은 잣대**로 비교한다.

왜 필요한가
──────────
driven_path.csv 는 주행마다 조건이 달라(코드·맵·상대 차) 스위치 하나의 효과를
못 가른다. 같은 bag 을 두 번 돌리면 입력이 같으니 차이는 스위치에서만 나온다.

★ 2026-08-10 deskew A/B 의 함정을 피한다
  그때는 원시 스캔에서 나온 AMCL 자세를 기준으로 판정해 결과가 편향됐다.
  그래서 여기서는:
    · arm 마다 AMCL 을 **새로 돌린다** (녹화된 /amcl_pose 를 쓰지 않는다)
    · 모든 arm 을 **/scan 과 /scan_deskewed 양쪽**으로 채점한다. 각자 자기 입력으로
      채점하면 deskew arm 이 자기 스캔에 맞춘 자세를 자기 스캔으로 재게 된다.
    · AMCL 은 난수를 쓰므로 arm 마다 반복해서, 반복 간 산포보다 큰 차이만 인정한다.

사용 (ROS 2 Humble + race_stack 빌드된 곳. 차 없이 된다)
──────────────────────────────────────────────────────
  # ① AMCL 입력 A/B — raw(/scan) · deskew(/scan_deskewed) · mask(/scan_amcl)
  ./bag_ab.py loc logs/bags/<주행> --gains logs/<주행>/gains.yaml \\
      --map src/race_stack/maps/<맵>.yaml --arms raw deskew --repeat 3 --out /tmp/ab1

  # ② 노드 파라미터 A/B — frenet_planner / map_controller 를 bag 자세로 open-loop 재생
  ros2 param dump /frenet_planner > /tmp/frenet.yaml      # 스택 띄운 채로 1회
  ./bag_ab.py node logs/bags/<주행> --exe frenet_planner --params /tmp/frenet.yaml \\
      --label skip0.10 --out /tmp/ab2
  ./bag_ab.py node logs/bags/<주행> --exe frenet_planner --params /tmp/frenet.yaml \\
      --set raycast_wall_skip=0.0 --label skip0 --out /tmp/ab2

  # ③ 결과
  ./bag_ab.py report /tmp/ab1 --map src/race_stack/maps/<맵>.yaml
  ./bag_ab.py report /tmp/ab2

주의
────
  · **맵은 그 주행 당시 것이어야 한다.** 0821_clean 처럼 같은 이름으로 덮어쓴 맵이
    있다. report 가 녹화된 자세의 잔차를 먼저 찍으니 그 값이 40㎝ 를 넘으면 맵부터 의심.
  · node 모드는 open-loop 다. 지령은 바뀌어도 차가 그 지령대로 움직이지는 않으므로
    "회피 판정·조향 지령이 어떻게 달라지나" 까지만 말할 수 있다.
  · 실차와 섞이지 않게 자식 프로세스는 ROS_DOMAIN_ID 99 + localhost 에 가둔다.
"""
import argparse
import glob
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import time

import numpy as np

DOMAIN = '99'
MARKER = '차량 덮어쓰기'
LOST_M = 0.40        # 스캔 평균 잔차가 이보다 크면 위치를 잃은 것으로 센다
SPEED_BINS = (('v<2', 0.5, 2.0), ('2~3.5', 2.0, 3.5), ('v>=3.5', 3.5, 99.0))
MAP_HINT_M = 0.40


# ── 설정 ───────────────────────────────────────────────────────────────
def _merge(a, b):
    for k, v in b.items():
        if isinstance(v, dict) and isinstance(a.get(k), dict):
            _merge(a[k], v)
        else:
            a[k] = v
    return a


def load_gains(path):
    """gains.yaml = race_script.yaml + 차량 덮어쓰기. 그냥 읽으면 중복 키가 절을
    통째로 바꿔치기하므로(localization 이 speed_sign 하나만 남는다) 나눠서 합친다."""
    import yaml
    text = open(path, encoding='utf-8').read()
    i = text.find(MARKER)
    if i < 0:
        return yaml.safe_load(text)
    head = text.rfind('\n', 0, i)
    return _merge(yaml.safe_load(text[:head]) or {},
                  yaml.safe_load(text[head:]) or {})


# ── 채점 (ROS 없이 돈다) ───────────────────────────────────────────────
def load_map(map_yaml):
    """localization_monitor._load_occ 와 같은 규약 — 행 0 이 위(최대 y)."""
    import yaml
    from PIL import Image
    cfg = yaml.safe_load(open(map_yaml, encoding='utf-8'))
    img = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), cfg['image'])
    arr = np.asarray(Image.open(img).convert('L'), dtype=np.uint8)
    occ = (arr < 128) if not cfg.get('negate', 0) else (arr > 128)
    h, w = occ.shape
    return occ, (h, w, float(cfg['resolution']),
                 float(cfg['origin'][0]), float(cfg['origin'][1]))


def expected_ranges(occ, meta, lx, ly, angs, rmax):
    """localization_monitor._expected 와 같은 레이캐스팅."""
    h, w, res, ox, oy = meta
    step = res * 0.5
    t = np.arange(0.0, rmax + step, step)
    xs = lx + np.cos(angs)[:, None] * t[None, :]
    ys = ly + np.sin(angs)[:, None] * t[None, :]
    ix = ((xs - ox) / res).astype(np.int32)
    iy = (h - (ys - oy) / res).astype(np.int32)
    inb = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    hit = np.zeros(ix.shape, bool)
    hit[inb] = occ[iy[inb], ix[inb]]
    hit |= ~inb
    free = ~hit
    escaped = free.any(axis=1)
    first_free = np.argmax(free, axis=1)
    hit = hit & (np.arange(hit.shape[1])[None, :] > first_free[:, None])
    exp = np.where(hit.any(axis=1), t[np.argmax(hit, axis=1)], np.nan)
    return np.where(escaped, exp, np.nan)


def score_scan(occ, meta, pose, ranges, angle_min, angle_inc, range_min, range_max,
               laser_dx=0.27, fov=math.pi, beam=math.radians(2.0), rmax=6.0,
               reject=1.0):
    """스캔 하나의 평균 잔차 [m]. 쓸 빔이 모자라면 None."""
    n = len(ranges)
    stride = max(1, int(round(beam / max(angle_inc, 1e-6))))
    idx = np.arange(0, n, stride)
    a = angle_min + idx * angle_inc
    keep = np.abs(a) <= fov * 0.5
    idx, a = idx[keep], a[keep]
    if not len(idx):
        return None
    meas = np.asarray(ranges, dtype=float)[idx]
    px, py, yaw = pose
    exp = expected_ranges(occ, meta, px + math.cos(yaw) * laser_dx,
                          py + math.sin(yaw) * laser_dx, yaw + a, rmax)
    ok = (np.isfinite(meas) & np.isfinite(exp)
          & (meas > range_min) & (meas < min(range_max, rmax)))
    if ok.sum() < 10:
        return None
    d = np.abs(meas[ok] - exp[ok])
    d = d[d < reject]
    return float(d.mean()) if len(d) >= 10 else None


def score_run(occ, meta, scans, poses, odoms, max_age=0.2):
    """scans: (t, ranges, amin, ainc, rmin, rmax) / poses: (t, x, y, yaw) /
    odoms: (t, v, w). t 는 header.stamp 라 재생 속도와 무관하다.
    반환: (v, |w|, 잔차, 채점 못한 스캔 수)"""
    if not poses or not scans:
        return np.zeros((0, 3)), len(scans)
    pt = np.array([p[0] for p in poses])
    ot = np.array([o[0] for o in odoms]) if odoms else np.zeros(0)
    rows, skipped = [], 0
    for s in scans:
        i = int(np.searchsorted(pt, s[0], side='right')) - 1
        if i < 0 or s[0] - pt[i] > max_age:
            skipped += 1
            continue
        r = score_scan(occ, meta, poses[i][1:], s[1], s[2], s[3], s[4], s[5])
        if r is None:
            skipped += 1
            continue
        v = w = 0.0
        if len(ot):
            j = min(int(np.searchsorted(ot, s[0])), len(ot) - 1)
            v, w = abs(odoms[j][1]), abs(odoms[j][2])
        rows.append((v, w, r))
    return (np.array(rows) if rows else np.zeros((0, 3))), skipped


def summarize(rows, skipped=0):
    """잔차 표의 한 줄. 표본이 적은 칸은 뺀다."""
    out = {}
    v, w, r = rows[:, 0], rows[:, 1], rows[:, 2]
    bins = [(n, (v >= a) & (v < b)) for n, a, b in SPEED_BINS]
    bins.append(('corner', (v >= 2.0) & (w > 1.0)))
    for name, m in bins:
        if m.sum() >= 30:
            out[name] = float(np.median(r[m]))
    moving = v >= 0.5
    out['p90'] = float(np.percentile(r[moving], 90)) if moving.sum() >= 30 else None
    out['lost'] = float(np.mean(r > LOST_M)) if len(r) else 1.0
    out['n'] = int(len(r))
    out['skipped'] = int(skipped)
    return out


def verdict(base, other, key='v>=3.5', min_cm=1.0):
    """base·other = 반복별 요약 리스트. 반복 산포보다 큰 차이만 차이로 친다."""
    a = [q[key] for q in base if key in q]
    b = [q[key] for q in other if key in q]
    if not a or not b:
        return '판정 불가 (표본 부족)', None
    diff = float(np.mean(b) - np.mean(a)) * 100
    spread = max(np.ptp(a), np.ptp(b)) * 100
    need = max(min_cm, 2.0 * spread)
    if len(a) < 2 or len(b) < 2:
        return f'{diff:+.1f}㎝ — 반복이 없어 산포를 모른다', diff
    if abs(diff) < need:
        return f'{diff:+.1f}㎝ — 차이 없음 (문턱 ±{need:.1f}㎝)', diff
    return f'{diff:+.1f}㎝ — {"개선" if diff < 0 else "악화"} (문턱 ±{need:.1f}㎝)', diff


# ── bag 읽기 ───────────────────────────────────────────────────────────
def _quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_topics(path, topics):
    """{토픽: [msg, ...]}. 없는 토픽은 빈 리스트."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rdr = rosbag2_py.SequentialReader()
    rdr.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
             rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in rdr.get_all_topics_and_types()
             if t.name in topics}
    out = {t: [] for t in topics}
    while rdr.has_next():
        topic, data, _ = rdr.read_next()
        if topic in types:
            out[topic].append(deserialize_message(data, types[topic]))
    return out


def _stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def to_scans(msgs):
    return [(_stamp(m), np.asarray(m.ranges, dtype=np.float32), m.angle_min,
             m.angle_increment, m.range_min, m.range_max) for m in msgs]


def to_poses(msgs):
    return [(_stamp(m), m.pose.pose.position.x, m.pose.pose.position.y,
             _quat_yaw(m.pose.pose.orientation)) for m in msgs]


def to_odoms(msgs):
    return [(_stamp(m), m.twist.twist.linear.x, m.twist.twist.angular.z)
            for m in msgs]


# ── 프로세스 ───────────────────────────────────────────────────────────
class Procs:
    def __init__(self, log_dir):
        self.env = dict(os.environ, ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY='1')
        self.env.pop('CYCLONEDDS_URI', None)     # dds_env.sh 의 IP 고정을 물려받지 않는다
        self.log_dir = log_dir
        self.procs = []
        os.makedirs(log_dir, exist_ok=True)

    def start(self, name, cmd):
        log = open(os.path.join(self.log_dir, f'{name}.log'), 'w')
        log.write(shlex.join(cmd) + '\n')
        log.flush()
        p = subprocess.Popen(cmd, env=self.env, stdout=log, stderr=log,
                             start_new_session=True)
        self.procs.append((name, p))
        return p

    def run_out(self, cmd):
        return subprocess.run(cmd, env=self.env, capture_output=True, text=True).stdout

    def stop_all(self):
        # rosbag2 는 SIGINT 를 받아야 메타데이터를 쓴다 (race.sh 와 같은 이유)
        for _, p in reversed(self.procs):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        deadline = time.time() + 8.0
        for _, p in self.procs:
            try:
                p.wait(timeout=max(0.1, deadline - time.time()))
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        self.procs = []


def ros_run(pkg, exe, params=None, files=()):
    cmd = ['ros2', 'run', pkg, exe, '--ros-args', '-r', f'__node:={exe}']
    for f in files:
        cmd += ['--params-file', f]
    for k, v in dict(params or {}, use_sim_time=True).items():
        cmd += ['-p', f'{k}:={_ros_value(v)}']
    return cmd


def _ros_value(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    return json.dumps(v) if isinstance(v, (list, str)) else str(v)


def play(procs, bag, topics, rate):
    p = procs.start('play', ['ros2', 'bag', 'play', bag, '--clock', '100',
                             '--rate', str(rate), '--topics', *topics])
    p.wait()
    time.sleep(2.0)     # 마지막 스캔이 처리될 시간


def wait_active(procs, node, timeout=45.0):
    end = time.time() + timeout
    while time.time() < end:
        if 'active' in procs.run_out(['ros2', 'lifecycle', 'get', f'/{node}']):
            return True
        time.sleep(1.0)
    return False


# ── loc: AMCL 입력 A/B ─────────────────────────────────────────────────
def run_loc_arm(args, arm, rep, cfg, first_pose, laser_frame):
    loc = cfg.get('localization', {})
    out = os.path.join(args.out, f'{arm}_{rep}')
    procs = Procs(os.path.join(args.out, 'logs', f'{arm}_{rep}'))
    scan_in = {'raw': '/scan', 'deskew': '/scan_deskewed', 'mask': '/scan_amcl'}[arm]
    amcl_yaml = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'src', 'race_stack', 'config', 'amcl.yaml')
    try:
        procs.start('static_tf', [
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0.27', '--y', '0', '--z', '0.11',
            '--frame-id', 'base_link', '--child-frame-id', laser_frame])
        # race.launch.py 의 imu_odom 파라미터와 같은 값이어야 한다
        procs.start('imu_odom', ros_run('race_stack', 'imu_odom', {
            'imu_yaw':             bool(loc.get('imu_yaw', True)),
            'imu_topic':           str(loc.get('imu_topic', '/sensors/imu/raw')),
            'gyro_units':          str(loc.get('imu_gyro_units', 'deg')),
            'yaw_axis':            str(loc.get('imu_yaw_axis', 'z')),
            'yaw_sign':            float(loc.get('imu_yaw_sign', 1.0)),
            'speed_sign':          float(loc.get('speed_sign', 1.0)),
            'bias_warmup_sec':     float(loc.get('imu_bias_warmup_sec', 3.0)),
            'bias_alpha':          float(loc.get('imu_bias_alpha', 0.002)),
            'stationary_speed':    float(loc.get('imu_stationary_speed', 0.05)),
            'stationary_rate_deg': float(loc.get('imu_stationary_rate_deg', 2.0)),
        }))
        # raw arm 도 deskew 를 띄운다 — AMCL 은 안 보고 채점 잣대로만 녹화한다
        procs.start('scan_deskew', ros_run('race_stack', 'scan_deskew', {
            'scan_in': '/scan', 'scan_out': '/scan_deskewed',
            'odom_topic': str(loc.get('odom_topic', '/odom_imu')),
            'stamp_at': str(loc.get('deskew_stamp_at', 'start')),
            'min_yaw_rate': float(loc.get('deskew_min_yaw_rate', 0.05)),
            'min_speed': float(loc.get('deskew_min_speed', 0.10)),
        }))
        if arm == 'mask':
            sets = {'amcl_mask': True, 'amcl_mask_max': args.mask_max,
                    'amcl_scan_topic': '/scan_amcl'}
            procs.start('frenet_planner', ros_run(
                'race_stack', 'frenet_planner', sets, [args.frenet_params]))
        procs.start('map_server', ros_run('nav2_map_server', 'map_server', {
            'yaml_filename': os.path.abspath(args.map),
            'topic_name': 'map', 'frame_id': 'map'}))
        procs.start('amcl', ros_run('nav2_amcl', 'amcl', {
            'scan_topic': scan_in.lstrip('/'),
            'initial_pose.x': first_pose[1], 'initial_pose.y': first_pose[2],
            'initial_pose.yaw': first_pose[3], 'set_initial_pose': True,
        }, [amcl_yaml]))
        procs.start('lifecycle', ros_run(
            'nav2_lifecycle_manager', 'lifecycle_manager',
            {'autostart': True, 'bond_timeout': 0.0,
             'node_names': ['map_server', 'amcl']}))
        if not wait_active(procs, 'amcl'):
            sys.exit(f'AMCL 이 active 가 안 됐다 — {procs.log_dir}/amcl.log 확인')
        procs.start('record', ['ros2', 'bag', 'record', '-o', out,
                               '/amcl_pose', '/scan', '/scan_deskewed', '/odom_imu'])
        time.sleep(2.0)
        play(procs, args.bag, ['/scan', '/odom', '/sensors/imu/raw'], args.rate)
    finally:
        procs.stop_all()


def cmd_loc(args):
    if 'mask' in args.arms and not args.frenet_params:
        sys.exit('mask arm 은 --frenet-params 가 필요하다 (ros2 param dump /frenet_planner)')
    cfg = load_gains(args.gains)
    src = read_topics(args.bag, ['/amcl_pose', '/scan'])
    if not src['/amcl_pose'] or not src['/scan']:
        sys.exit('bag 에 /amcl_pose 또는 /scan 이 없다')
    first = to_poses(src['/amcl_pose'][:1])[0]
    frame = src['/scan'][0].header.frame_id
    os.makedirs(args.out, exist_ok=True)
    json.dump({'source_bag': os.path.abspath(args.bag), 'kind': 'loc'},
              open(os.path.join(args.out, 'meta.json'), 'w'))
    for arm in args.arms:
        for rep in range(args.repeat):
            print(f'[bag_ab] {arm} #{rep + 1}/{args.repeat}', flush=True)
            run_loc_arm(args, arm, rep, cfg, first, frame)
    print(f'[bag_ab] 끝. ./bag_ab.py report {args.out} --map {args.map}')


def report_loc(args, meta):
    occ, m = load_map(args.map)
    src = read_topics(meta['source_bag'], ['/amcl_pose', '/scan', '/odom_imu'])
    odoms = to_odoms(src['/odom_imu'])
    rows, skipped = score_run(occ, m, to_scans(src['/scan']),
                              to_poses(src['/amcl_pose']), odoms)
    ref = summarize(rows, skipped)
    print(f"녹화된 /amcl_pose 잔차 — {_line(ref)}")
    if ref.get('v<2', ref.get('2~3.5', 0)) > MAP_HINT_M:
        print('⚠ 녹화 자세의 잔차가 크다. 맵이 그 주행 당시 것인지 먼저 확인할 것.')

    res = {}
    for d in sorted(glob.glob(os.path.join(args.out, '*_[0-9]*'))):
        if not os.path.isdir(d) or not os.path.exists(os.path.join(d, 'metadata.yaml')):
            continue
        arm = os.path.basename(d).rsplit('_', 1)[0]
        t = read_topics(d, ['/amcl_pose', '/scan', '/scan_deskewed', '/odom_imu'])
        poses, od = to_poses(t['/amcl_pose']), to_odoms(t['/odom_imu']) or odoms
        for yard, topic in (('/scan', '/scan'), ('/scan_deskewed', '/scan_deskewed')):
            rows, skipped = score_run(occ, m, to_scans(t[topic]), poses, od)
            res.setdefault(yard, {}).setdefault(arm, []).append(summarize(rows, skipped))
    for yard, arms in res.items():
        print(f'\n잣대 {yard} (스캔 평균 잔차의 중앙값 [㎝])')
        for arm, reps in arms.items():
            for i, q in enumerate(reps):
                print(f'  {arm:7s} #{i + 1}  {_line(q)}')
        base = arms.get('raw')
        for arm, reps in arms.items():
            if base and arm != 'raw':
                for key in ('v>=3.5', 'corner'):
                    print(f'  → {arm} vs raw [{key}]: {verdict(base, reps, key)[0]}')


def _line(q):
    cells = ' '.join(f"{k} {q[k] * 100:4.1f}" if k in q else f'{k} -' for k in
                     ('v<2', '2~3.5', 'v>=3.5', 'corner'))
    p90 = '-' if q.get('p90') is None else f"{q['p90'] * 100:.1f}"
    return (f"{cells} | p90 {p90} | 잃음 {q['lost'] * 100:.1f}% | "
            f"n {q['n']} (못 잼 {q['skipped']})")


# ── node: frenet_planner / map_controller open-loop A/B ────────────────
NODE_TOPICS = {
    # map_controller 는 bag 의 회피 결과를 입력으로 받는다
    'frenet_planner': (['/scan', '/odom_imu', '/amcl_pose', '/race/enabled'],
                       ['/frenet/diag', '/frenet/offset', '/frenet/status']),
    'map_controller': (['/odom_imu', '/amcl_pose', '/race/enabled', '/frenet/offset',
                        '/frenet/speed_scale', '/frenet/diag', '/loc/residual'],
                       ['/drive']),
}


def parse_sets(items):
    out = {}
    for s in items or []:
        k, _, v = s.partition('=')
        for conv in (int, float):
            try:
                out[k] = conv(v)
                break
            except ValueError:
                continue
        else:
            out[k] = {'true': True, 'false': False}.get(v.lower(), v)
    return out


def cmd_node(args):
    play_t, rec_t = NODE_TOPICS[args.exe]
    out = os.path.join(args.out, args.label)
    procs = Procs(os.path.join(args.out, 'logs', args.label))
    os.makedirs(args.out, exist_ok=True)
    json.dump({'source_bag': os.path.abspath(args.bag), 'kind': args.exe},
              open(os.path.join(args.out, 'meta.json'), 'w'))
    try:
        procs.start(args.exe, ros_run('race_stack', args.exe,
                                      parse_sets(args.set), [args.params]))
        time.sleep(5.0)
        procs.start('record', ['ros2', 'bag', 'record', '-o', out, *rec_t])
        time.sleep(2.0)
        play(procs, args.bag, play_t, args.rate)
    finally:
        procs.stop_all()
    print(f'[bag_ab] {args.label} 끝. ./bag_ab.py report {args.out}')


def report_node(args, meta):
    labels = sorted(d for d in os.listdir(args.out)
                    if os.path.exists(os.path.join(args.out, d, 'metadata.yaml')))
    if meta['kind'] == 'frenet_planner':
        from race_stack.obstacle_avoid import FRENET_DIAG_KEYS, unpack_frenet_diag
        print(f"{'label':14s} {'n_obs>0':>8s} {'|d|>5㎝':>8s} {'속도배율':>8s}  status%")
        for lb in labels:
            t = read_topics(os.path.join(args.out, lb), ['/frenet/diag'])
            rows = [unpack_frenet_diag(m.data) for m in t['/frenet/diag']]
            if not rows:
                print(f'{lb:14s} (출력 없음)')
                continue
            col = {k: np.array([r.get(k, np.nan) for r in rows], float)
                   for k in FRENET_DIAG_KEYS}
            st = col['planner_status'][np.isfinite(col['planner_status'])].astype(int)
            hist = ' '.join(f'{s}:{100 * np.mean(st == s):.0f}' for s in np.unique(st))
            print(f"{lb:14s} {100 * np.nanmean(col['n_obs'] > 0):7.1f}% "
                  f"{100 * np.nanmean(np.abs(col['selected_d']) > 0.05):7.1f}% "
                  f"{np.nanmean(col['speed_scale']):8.3f}  {hist}")
    else:
        print(f"{'label':14s} {'|δ|p50':>8s} {'|δ|p95':>8s} {'|δ|>0.34':>9s} {'지령v중앙':>9s}")
        for lb in labels:
            t = read_topics(os.path.join(args.out, lb), ['/drive'])
            d = np.array([m.drive.steering_angle for m in t['/drive']], float)
            v = np.array([m.drive.speed for m in t['/drive']], float)
            if not len(d):
                print(f'{lb:14s} (출력 없음)')
                continue
            print(f'{lb:14s} {np.median(np.abs(d)):8.3f} '
                  f'{np.percentile(np.abs(d), 95):8.3f} '
                  f'{100 * np.mean(np.abs(d) > 0.34):8.1f}% {np.median(v):9.2f}')


def cmd_report(args):
    meta = json.load(open(os.path.join(args.out, 'meta.json')))
    if meta['kind'] == 'loc':
        if not args.map:
            sys.exit('loc 결과는 --map 이 필요하다')
        report_loc(args, meta)
    else:
        report_node(args, meta)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('loc', help='AMCL 입력 A/B')
    p.add_argument('bag')
    p.add_argument('--gains', required=True, help='logs/<주행>/gains.yaml')
    p.add_argument('--map', required=True, help='그 주행 당시 맵 yaml')
    p.add_argument('--arms', nargs='+', default=['raw', 'deskew'],
                   choices=['raw', 'deskew', 'mask'])
    p.add_argument('--repeat', type=int, default=3)
    p.add_argument('--rate', type=float, default=1.0)
    p.add_argument('--frenet-params', help='mask arm 용 ros2 param dump 결과')
    p.add_argument('--mask-max', type=float, default=0.2)
    p.add_argument('--out', required=True)
    p.set_defaults(fn=cmd_loc)
    p = sub.add_parser('node', help='frenet_planner/map_controller 파라미터 A/B')
    p.add_argument('bag')
    p.add_argument('--exe', required=True, choices=list(NODE_TOPICS))
    p.add_argument('--params', required=True, help='ros2 param dump 결과')
    p.add_argument('--set', nargs='*', help='덮어쓸 값 k=v ...')
    p.add_argument('--label', required=True)
    p.add_argument('--rate', type=float, default=1.0)
    p.add_argument('--out', required=True)
    p.set_defaults(fn=cmd_node)
    p = sub.add_parser('report', help='결과 비교')
    p.add_argument('out')
    p.add_argument('--map')
    p.set_defaults(fn=cmd_report)
    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    main()
