"""joystick.launch.py — 조이스틱 수동 조종 + 차량 브링업.

f1tenth_stack bringup (joy 드라이버 포함) + joystick_teleop 을 한 번에 기동.
race_script.yaml의 joystick 섹션 파라미터 사용.

조작법 (Logitech F710, Mode 버튼 OFF):
  LB              → 수동 ↔ 자율 모드 토글
  RT 당김          → 전진 (당긴 만큼 선형 비례, 놓으면 즉시 정지)
  RB 누른 채 RT    → 후진
  L스틱 좌/우      → 조향

★ joy 드라이버는 bringup_launch.py 가 이미 띄운다.
  여기서 joy_node 를 또 띄우면 /joy 퍼블리셔가 2개가 되어 deadzone 설정이
  뒤섞이므로, 대신 bringup 의 joy_config 인자를 덮어써서 설정만 주입한다.
"""
import os
import re
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _write_joy_config(joy_cfg: dict) -> str:
    """bringup 의 joy / joy_teleop 노드에 넘길 파라미터 파일을 생성."""
    dev = str(joy_cfg.get('device', '/dev/input/js0'))
    m = re.search(r'(\d+)\s*$', dev)
    device_id = int(m.group(1)) if m else 0

    params = {
        # joy 드라이버: deadzone 0 → 트리거 응답을 왜곡 없이 그대로 전달.
        # (데드존은 joystick_teleop 이 정규화된 0~1 스로틀 기준으로 처리한다.
        #  joy 의 deadzone 은 축의 '중앙'을 기준으로 잡히는데, 트리거는 중앙이
        #  절반쯤 당긴 지점이라 그대로 두면 스로틀 중간에 평평한 구간이 생긴다.)
        # autorepeat_rate 20Hz → RT를 고정한 채 있어도 /joy 가 계속 발행되어
        #  ackermann_mux 의 timeout(0.2s)에 걸려 차가 끊기지 않는다.
        'joy': {'ros__parameters': {
            'device_id':       device_id,
            'deadzone':        0.0,
            'autorepeat_rate': 20.0,
        }},
        # f1tenth_stack 기본 joy_teleop 도 /teleop 에 발행한다.
        # 원래 데드맨이 LB(4)인데 race_stack 은 LB 를 모드 토글로 쓰므로 충돌한다.
        # 데드맨을 RS(10)로 옮겨 충돌만 없애고, 수동 오버라이드 기능은 남겨둔다.
        'joy_teleop': {'ros__parameters': {
            'human_control': {
                'type':            'topic',
                'interface_type':  'ackermann_msgs/msg/AckermannDriveStamped',
                'topic_name':      'teleop',
                'deadman_buttons': [10],
                'axis_mappings': {
                    'drive-speed':          {'axis': 1, 'scale': 1.0,  'offset': 0.0},
                    'drive-steering_angle': {'axis': 3, 'scale': 0.34, 'offset': 0.0},
                },
            },
        }},
    }

    path = os.path.join(tempfile.gettempdir(), 'race_joy_params.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(params, f)
    return path


def generate_launch_description():
    cfg_path = os.environ.get(
        'RACE_SCRIPT', os.path.expanduser('~/race_script.yaml'))
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    joy_cfg = cfg.get('joystick', {})
    speed   = cfg.get('speed', {})

    f1tenth_share = get_package_share_directory('f1tenth_stack')
    joy_config    = _write_joy_config(joy_cfg)

    nodes = []

    # 차량 브링업 (LiDAR + VESC + odom + mux + joy 드라이버)
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_share, 'launch', 'bringup_launch.py')),
        launch_arguments={'joy_config': joy_config}.items()))

    # joystick_teleop (3초 지연: 브링업 후 mux 준비 시간 확보)
    nodes.append(TimerAction(period=3.0, actions=[Node(
        package='race_stack', executable='joystick_teleop',
        name='joystick_teleop', output='screen',
        parameters=[{
            'axis_steer':       int(  joy_cfg.get('axis_steer',       0)),
            'axis_speed':       int(  joy_cfg.get('axis_speed',       5)),
            'btn_reverse':      int(  joy_cfg.get('btn_reverse',      5)),
            'btn_toggle':       int(  joy_cfg.get('btn_toggle',       4)),
            'steer_invert':     bool( joy_cfg.get('steer_invert',     False)),
            'trigger_invert':   bool( joy_cfg.get('trigger_invert',   True)),
            'trigger_deadzone': float(joy_cfg.get('trigger_deadzone', 0.05)),
            'trigger_expo':     float(joy_cfg.get('trigger_expo',     1.0)),
            'steer_deadzone':   float(joy_cfg.get('steer_deadzone',   0.08)),
            'arm_on_neutral':   bool( joy_cfg.get('arm_on_neutral',   True)),
            'debug':            bool( joy_cfg.get('debug',            False)),
            'max_speed':        float(speed.get('mapping_max', 1.0)),
            'max_steer':        float(cfg.get('pure_pursuit', {}).get('max_steer', 0.36)),
        }])]))

    return LaunchDescription(nodes)
