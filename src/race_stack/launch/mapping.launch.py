"""mapping.launch.py — SLAM 맵 생성 모드.

f1tenth_stack 브링업(센서+VESC+mux) + slam_toolbox(online async, mapping).
키보드 텔레옵은 tty가 필요해 race.sh가 포그라운드로 따로 띄운다.

★ imu_odom 도 함께 띄운다. vesc.yaml 에서 vesc_to_odom_node.publish_tf 를 껐기
  때문에 odom→base_link TF 를 발행하는 것이 이 노드뿐이고, slam_toolbox 는 odom
  TF 가 없으면 맵을 만들지 못한다. 맵 품질 자체도 자이로 yaw 쪽이 낫다.
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    f1tenth_share = get_package_share_directory('f1tenth_stack')
    slam_params = os.path.join(f1tenth_share, 'config',
                               'f1tenth_online_async.yaml')

    # race.launch.py 와 같은 경로 규약 (race.sh 가 RACE_SCRIPT 를 export 한다)
    cfg_path = os.environ.get(
        'RACE_SCRIPT', os.path.expanduser('~/race_script.yaml'))
    try:
        with open(cfg_path) as f:
            loc = (yaml.safe_load(f) or {}).get('localization', {}) or {}
    except OSError:
        loc = {}

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_share, 'launch', 'bringup_launch.py')))

    imu_odom = Node(
        package='race_stack', executable='imu_odom', name='imu_odom',
        output='screen',
        parameters=[{
            'imu_yaw':             bool( loc.get('imu_yaw',             True)),
            'imu_topic':           str(  loc.get('imu_topic',           '/sensors/imu/raw')),
            'gyro_units':          str(  loc.get('imu_gyro_units',      'deg')),
            'yaw_axis':            str(  loc.get('imu_yaw_axis',        'z')),
            'yaw_sign':            float(loc.get('imu_yaw_sign',        1.0)),
            'bias_warmup_sec':     float(loc.get('imu_bias_warmup_sec', 3.0)),
            'bias_alpha':          float(loc.get('imu_bias_alpha',      0.002)),
            'stationary_speed':    float(loc.get('imu_stationary_speed', 0.05)),
            'stationary_rate_deg': float(loc.get('imu_stationary_rate_deg', 2.0)),
        }])

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params])

    return LaunchDescription([bringup, imu_odom, slam])
