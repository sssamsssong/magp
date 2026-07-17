"""race.launch.py — 자율주행 모드 (q1 / q2 / q3 / race).

RACE_SCRIPT 환경변수(기본: ~/race_script.yaml)의 설정을 읽어
전체 스택을 한 번에 기동한다:

  f1tenth_stack 브링업 (LiDAR + VESC + odom + mux)
  → map_server + AMCL (저장된 맵 위에서 위치 추정)
  → mission_manager (랩 카운트, 자동 정지)
  → safety_node (TTC 기반 AEB)
  → 컨트롤러 (모드에 따라 pure_pursuit 또는 gap_follower)
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            TimerAction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

DEFAULT_CONTROLLER = {'q1': 'pure_pursuit', 'q2': 'gap_follower',
                      'q3': 'pure_pursuit', 'race': 'gap_follower'}

VALID_CONTROLLERS = {'pure_pursuit', 'gap_follower', 'stanley'}


def generate_launch_description():
    # ---------- race_script.yaml 로드 ----------
    cfg_path = os.environ.get(
        'RACE_SCRIPT', os.path.expanduser('~/race_script.yaml'))
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    # mode:=/controller:= 오버라이드는 race.sh가 launch arg로 준다.
    # sys.argv를 직접 파싱한다 (LaunchConfiguration은 파이썬 로직에서 못 씀)
    import sys
    mode = cfg['mode']
    controller = cfg.get('controller', 'auto')
    for a in sys.argv:
        if a.startswith('mode:='):
            mode = a.split(':=')[1]
        elif a.startswith('controller:='):
            controller = a.split(':=')[1]
    if mode not in DEFAULT_CONTROLLER:
        raise RuntimeError(f'race.launch.py: 잘못된 mode "{mode}"')

    if controller == 'auto':
        controller = DEFAULT_CONTROLLER[mode]
    if controller not in VALID_CONTROLLERS:
        raise RuntimeError(
            f'race.launch.py: invalid controller "{controller}". '
            f'Valid: {sorted(VALID_CONTROLLERS)}')

    max_speed = float(cfg['speed'][f'{mode}_max'])
    min_speed = float(cfg['speed']['min_speed'])

    race_share = get_package_share_directory('race_stack')
    f1tenth_share = get_package_share_directory('f1tenth_stack')

    # 맵/웨이포인트/경계는 빌드 시점 스냅샷(share)이 아니라 repo src 에서 직접 읽는다.
    # RACE_WS 가 있고 그 src/race_stack/maps 가 실재하면 이를 우선 사용 → 새 맵·
    # 웨이포인트를 만들어도 colcon build 없이 즉시 인식된다 (gen_waypoints 와 동일 규약).
    # 없으면 기존처럼 share 로 폴백.
    _ws = os.environ.get('RACE_WS')
    _src = os.path.join(_ws, 'src', 'race_stack') if _ws else None
    race_data = _src if _src and os.path.isdir(os.path.join(_src, 'maps')) \
                else race_share

    map_yaml = os.path.join(race_data, 'maps', cfg['map_name'] + '.yaml')
    wp_csv = os.path.join(race_data, 'waypoints', cfg['waypoints'] + '.csv')
    amcl_yaml = os.path.join(race_share, 'config', 'amcl.yaml')

    if not os.path.exists(map_yaml):
        raise RuntimeError(
            f'맵 파일 없음: {map_yaml}\n'
            f'→ 주최측 맵을 src/race_stack/maps/에 넣고 colcon build 하거나,\n'
            f'  ./race.sh mapping 으로 직접 맵을 만든 뒤 ./race.sh savemap 하세요.')

    nodes = []

    # ---------- 1. 차량 브링업 ----------
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_share, 'launch', 'bringup_launch.py'))))

    # ---------- 2. 맵 서버 + AMCL + lifecycle ----------
    loc = cfg.get('localization', {})
    nodes.append(Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml,
                     'topic_name': 'map', 'frame_id': 'map'}]))
    nodes.append(Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[amcl_yaml,
                    {'initial_pose.x': float(loc.get('init_x', 0.0)),
                     'initial_pose.y': float(loc.get('init_y', 0.0)),
                     'initial_pose.yaw': float(loc.get('init_yaw', 0.0)),
                     'set_initial_pose': True}]))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['map_server', 'amcl']}]))

    # ---------- 3. 미션 매니저 ----------
    m = cfg['mission']
    time_limit = int(m.get('time_limit_sec', 0))
    if mode == 'q3' and time_limit == 0:
        time_limit = 120   # Q3 규정: 2분
    nodes.append(Node(
        package='race_stack', executable='mission_manager',
        name='mission_manager', output='screen',
        parameters=[{'laps': int(m.get('laps', 0)),
                     'time_limit_sec': time_limit,
                     'start_x': float(m.get('start_x', 0.0)),
                     'start_y': float(m.get('start_y', 0.0)),
                     'start_radius': float(m.get('start_radius', 1.5))}]))

    # ---------- 4. 안전장치 (AEB) ----------
    s = cfg.get('safety', {})
    if s.get('enabled', True):
        nodes.append(Node(
            package='race_stack', executable='safety_node',
            name='safety_node', output='screen',
            parameters=[{'ttc_threshold': float(s.get('ttc_threshold', 0.6)),
                         'hold_sec': float(s.get('hold_sec', 1.0))}]))

    # ---------- 5. 컨트롤러 (AMCL 수렴 시간 확보를 위해 3초 지연 기동) ----------
    if controller == 'pure_pursuit':
        pp = cfg['pure_pursuit']
        ctrl = Node(
            package='race_stack', executable='pure_pursuit',
            name='pure_pursuit', output='screen',
            parameters=[{'waypoint_file': wp_csv,
                         'max_speed': max_speed, 'min_speed': min_speed,
                         'lookahead': float(pp['lookahead']),
                         'lookahead_speed_gain': float(pp['lookahead_speed_gain']),
                         'wheelbase': float(pp['wheelbase']),
                         'max_steer': float(pp['max_steer']),
                         'curvature_slowdown': float(pp['curvature_slowdown'])}])

    elif controller == 'stanley':
        sc = cfg.get('stanley', {})
        pp = cfg.get('pure_pursuit', {})

        # Q2: boundary files live in  maps/<map_name>/  subdirectory
        bound_dir = sc.get('bound_dir', '') or \
            os.path.join(race_data, 'maps', cfg['map_name'])

        ctrl = Node(
            package='race_stack', executable='stanley_controller',
            name='stanley_controller', output='screen',
            parameters=[{
                'mode':            mode,
                'waypoint_file':   wp_csv,
                'max_speed':       max_speed,
                'min_speed':       min_speed,
                'max_steer':       float(pp.get('max_steer', 0.36)),
                'wheelbase':       float(pp.get('wheelbase',  0.33)),
                'stanley_gain':    float(sc.get('gain',           0.8)),
                'stanley_k':       float(sc.get('k',              1.8)),
                'stanley_ks':      float(sc.get('ks',             1.3)),
                'vel_scale':       float(sc.get('vel_scale',      1.0)),
                'obstacle_decel':  float(sc.get('obstacle_decel', 0.85)),
                'obs2lane_dist':   float(sc.get('obs2lane_dist',  0.25)),
                'interp_scale':    int(  sc.get('interp_scale',   20)),
                'lookahead_min':   float(sc.get('lookahead_min',  0.5)),
                'lookahead_max':   float(sc.get('lookahead_max',  2.0)),
                'lookahead_gain':  float(sc.get('lookahead_gain', 7.0)),
                'ff_gain':         float(sc.get('ff_gain',        1.0)),
                'ff_preview_sec':  float(sc.get('ff_preview_sec', 0.10)),
                'steer_lpf':       float(sc.get('steer_lpf',      1.0)),
                # Q2 obstacle detection
                'map_yaml':        map_yaml,
                'bound_dir':       bound_dir,
                'grid_xmin':       0.0,  'grid_xmax':      5.0,
                'grid_ymin':      -2.5,  'grid_ymax':      2.5,
                'grid_res':        0.04, 'grid_safe_dist':  0.3,
                'bound_ignore_px': 7.5,
            }])

    else:
        gf = cfg['gap_follower']
        ctrl = Node(
            package='race_stack', executable='gap_follower',
            name='gap_follower', output='screen',
            parameters=[{'max_speed': max_speed, 'min_speed': min_speed,
                         'car_width': float(gf['car_width']),
                         'disparity_threshold': float(gf['disparity_threshold']),
                         'max_range': float(gf['max_range']),
                         'fov_deg': float(gf['fov_deg']),
                         'steer_gain': float(gf['steer_gain']),
                         'speed_gain': float(gf['speed_gain']),
                         'max_steer': float(cfg['pure_pursuit']['max_steer'])}])

    nodes.append(TimerAction(period=3.0, actions=[ctrl]))

    nodes.append(ExecuteProcess(
        cmd=['echo', f'[race.launch] mode={mode} controller={controller} '
                     f'max_speed={max_speed} map={cfg["map_name"]}'],
        output='screen'))

    return LaunchDescription(
        [DeclareLaunchArgument('mode', default_value=mode),
         DeclareLaunchArgument('controller', default_value=controller)]
        + nodes)
