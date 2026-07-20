"""sim_race.launch.py — f1tenth_gym_ros + StanleyController 통합 런치.

race_script.yaml 의 stanley/speed/waypoints/map 설정을 그대로 사용해
시뮬레이터에서 실차 알고리즘을 구동한다.

실행:
  export RACE_WS=~/f1tenth_ws
  export RACE_SCRIPT=~/yax/race_script.yaml
  source ~/yax/f1TENTH-contest_full_stack/sim_ws/install/setup.bash
  source ~/f1tenth_ws/install/setup.bash
  ros2 launch race_stack sim_race.launch.py

  또는 race.sh 사용:
  ./race.sh sim
"""
import math
import os
import yaml

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── race_script.yaml 로드 ────────────────────────────────────────────
    cfg_path = os.environ.get('RACE_SCRIPT',
                              os.path.expanduser('~/race_script.yaml'))
    if not os.path.exists(cfg_path):
        raise RuntimeError(
            f'race_script.yaml 없음: {cfg_path}\n'
            'RACE_SCRIPT 환경변수를 설정하거나 ./race.sh sim 으로 실행하세요.')
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    # ── 맵·웨이포인트 경로 ────────────────────────────────────────────────
    _ws  = os.environ.get('RACE_WS')
    _src = os.path.join(_ws, 'src', 'race_stack') if _ws else None
    race_data = (_src if _src and os.path.isdir(os.path.join(_src, 'maps'))
                 else get_package_share_directory('race_stack'))

    map_name = cfg['map_name']
    map_base = os.path.join(race_data, 'maps', map_name)  # 확장자 없음
    map_yaml = map_base + '.yaml'
    wp_csv   = os.path.join(race_data, 'waypoints', cfg['waypoints'] + '.csv')

    if not os.path.exists(map_yaml):
        raise RuntimeError(f'맵 파일 없음: {map_yaml}')
    if not os.path.exists(wp_csv):
        raise RuntimeError(f'웨이포인트 없음: {wp_csv}')

    map_img_ext = '.png' if os.path.exists(map_base + '.png') else '.pgm'

    # ── 스폰 위치 (race_script.yaml [sim] 섹션, 없으면 기본값) ─────────
    sim_cfg = cfg.get('sim', {})
    sx      = float(sim_cfg.get('sx',     0.853))
    sy      = float(sim_cfg.get('sy',     4.111))
    stheta  = float(sim_cfg.get('stheta', math.pi))

    # ── 컨트롤러 파라미터 ─────────────────────────────────────────────────
    mode      = cfg.get('mode', 'q1')
    max_speed = float(cfg['speed'].get(f'{mode}_max', cfg['speed']['q1_max']))
    min_speed = float(cfg['speed']['min_speed'])
    sc        = cfg.get('stanley', {})
    pp        = cfg.get('pure_pursuit', {})
    bound_dir = sc.get('bound_dir', '') or os.path.join(race_data, 'maps', map_name)

    # ── f1tenth_gym_ros 패키지 공유 디렉토리 ──────────────────────────────
    sim_share = get_package_share_directory('f1tenth_gym_ros')
    sim_yaml  = os.path.join(sim_share, 'config', 'sim.yaml')

    # ── 노드 정의 ─────────────────────────────────────────────────────────

    # 1. gym 물리 시뮬레이터
    gym_bridge = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        output='screen',
        parameters=[sim_yaml,
                    {'map_path':    map_base,
                     'map_img_ext': map_img_ext,
                     'num_agent':   1,
                     'kb_teleop':   False,
                     'sx':     sx,
                     'sy':     sy,
                     'stheta': stheta}])

    # 2. RViz
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz',
        arguments=['-d', os.path.join(sim_share, 'launch', 'gym_bridge.rviz')])

    # 3. 차량 URDF 발행 (RViz 시각화용)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', os.path.join(sim_share, 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')])

    # 4. 맵 서버 (RViz 지도 표시)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml,
                     'topic_name': 'map', 'frame_id': 'map'}])
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_sim',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server']}])

    # 5. sim_bridge: /ego_racecar/odom → /amcl_pose + /odom + /race/enabled
    sim_bridge = Node(
        package='race_stack',
        executable='sim_bridge',
        name='sim_bridge',
        output='screen')

    # 6. StanleyController (시뮬 초기화 3초 대기 후 기동)
    stanley = Node(
        package='race_stack',
        executable='stanley_controller',
        name='stanley_controller',
        output='screen',
        parameters=[{
            'mode':            mode,
            'waypoint_file':   wp_csv,
            'max_speed':       max_speed,
            'min_speed':       min_speed,
            'max_steer':       float(pp.get('max_steer',       0.36)),
            'wheelbase':       float(pp.get('wheelbase',       0.33)),
            'stanley_gain':    float(sc.get('gain',            0.8)),
            'stanley_k':       float(sc.get('k',               1.0)),
            'stanley_ks':      float(sc.get('ks',              1.5)),
            'vel_scale':       float(sc.get('vel_scale',       1.0)),
            'obstacle_decel':  float(sc.get('obstacle_decel',  0.85)),
            'obs2lane_dist':   float(sc.get('obs2lane_dist',   0.25)),
            'interp_scale':    int(  sc.get('interp_scale',    20)),
            'lookahead_min':   float(sc.get('lookahead_min',   0.5)),
            'lookahead_max':   float(sc.get('lookahead_max',   2.0)),
            'lookahead_gain':  float(sc.get('lookahead_gain',  7.0)),
            'ff_gain':         float(sc.get('ff_gain',         1.0)),
            'ff_preview_sec':  float(sc.get('ff_preview_sec',  0.10)),
            'steer_lpf':       float(sc.get('steer_lpf',       1.0)),
            # Q2 장애물 감지 (맵·경계 파일)
            'map_yaml':        map_yaml,
            'bound_dir':       bound_dir,
        }])

    return LaunchDescription([
        gym_bridge,
        rviz,
        rsp,
        map_server,
        lifecycle,
        sim_bridge,
        TimerAction(period=3.0, actions=[stanley]),
    ])
