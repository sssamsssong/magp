"""curvature_probe.launch.py — 조이스틱 조종 + 곡률-속도 캘리브레이션 리그.

`./race.sh curvprobe` 하나로 끝난다. joystick.launch.py 를 quiet 모드로
포함해 차량 브링업과 조종을 올리고, 그 위에 curvature_probe 를 얹는다.

slip.launch.py 와 같은 패턴이지만 완전히 독립된 토픽/설정을 쓴다
(race_script.yaml 의 curv_probe: 섹션, /curvprobe/* 토픽) — slip 쪽 파일은
건드리지 않는다.
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
    cp = cfg.get('curv_probe', {})
    vesc = os.path.join(os.path.dirname(os.path.dirname(share)), 'config', 'vesc.yaml')

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

    # joystick.launch.py 의 RACE_TELEOP_MAX_SPEED/STEER (범용 오버라이드) 를 쓴다.
    # RACE_SLIP_* 는 slip 리그 전용이라 여기서 안 건드린다.
    os.environ['RACE_QUIET'] = '1'
    os.environ['RACE_TELEOP_MAX_SPEED'] = str(float(cp.get('max_speed', 3.5)))
    os.environ['RACE_TELEOP_MAX_STEER'] = str(float(cp.get('full_lock_steer', 0.65)))

    nodes = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'joystick.launch.py')))]

    nodes.append(TimerAction(period=5.0, actions=[Node(
        package='race_stack', executable='curvature_probe',
        name='curvature_probe', output='screen',
        parameters=[{
            'servo_gain':      gain,
            'servo_offset':    offset,
            'wheelbase':       float(cfg.get('pure_pursuit', {}).get('wheelbase', 0.33)),
            'max_speed':       float(cp.get('max_speed', 3.5)),
            'full_lock_steer': float(cp.get('full_lock_steer', 0.65)),
            'kappa_max':       float(cp.get('kappa_max', 1.1406)),
            'mu':              float(cp.get('mu', 0.7)),
        }])]))
    return LaunchDescription(nodes)
