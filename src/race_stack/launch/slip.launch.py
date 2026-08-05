"""slip.launch.py — 조이스틱 조종 + 슬립 테스트 리그를 한 번에.

`./race.sh slip` 하나로 끝난다. joystick.launch.py 를 quiet 모드로 포함해
차량 브링업과 조종을 올리고, 그 위에 slip_logger 를 얹는다.

★ 왜 quiet 인가
  slip_logger 는 매 0.3초 전체 화면을 지우고 패널을 다시 그린다. 같은 터미널에
  다른 노드가 stdout 으로 끼어들면 패널이 깨진다. 그래서 joystick_teleop 은
  로그 파일로 돌리고, 대신 그 상태(트리거 중립 대기 등)를 패널 안에 띄운다.
  브링업 로그는 기동 직후에만 나오고 이후 조용하다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    share = get_package_share_directory('race_stack')
    cfg_path = os.environ.get('RACE_SCRIPT', os.path.expanduser('~/race_script.yaml'))
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    sl = cfg.get('slip', {})
    vesc = os.path.join(os.path.dirname(os.path.dirname(share)), 'config', 'vesc.yaml')

    # 서보 캘리브레이션은 config/vesc.yaml 이 원본이다. 여기서 다시 적지 않는다 —
    # 두 곳에 적으면 반드시 갈라진다.
    gain, offset = -0.761, 0.425
    for p in (os.path.join(os.path.expanduser(os.environ.get('RACE_WS', '~/yax')),
                           'config', 'vesc.yaml'), vesc):
        try:
            v = yaml.safe_load(open(p))['/**']['ros__parameters']
            gain = float(v['steering_angle_to_servo_gain'])
            offset = float(v['steering_angle_to_servo_offset'])
            break
        except Exception:
            continue

    nodes = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'joystick.launch.py')),
        launch_arguments={'quiet': 'true'}.items())]

    # 브링업·teleop 이 자리를 잡은 뒤 패널을 띄운다 (그래야 첫 화면이 안 깨진다)
    nodes.append(TimerAction(period=5.0, actions=[Node(
        package='race_stack', executable='slip_logger',
        name='slip_logger', output='screen',
        parameters=[{
            'servo_gain':   gain,
            'servo_offset': offset,
            'wheelbase':    float(cfg.get('pure_pursuit', {}).get('wheelbase', 0.33)),
            'radii':        [float(x) for x in sl.get('radii', [1.0, 1.5, 2.0, 3.0])],
            'speeds':       [float(x) for x in sl.get('speeds',
                                                      [1.0, 1.5, 2.0, 2.5, 3.0])],
            'target_runs':  int(sl.get('target_runs', 3)),
            'lever_x':      float(sl.get('lever_x', 0.0)),
            'mu':           float(sl.get('mu', 0.7)),
        }])]))
    return LaunchDescription(nodes)
