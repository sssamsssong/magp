"""mapping.launch.py — SLAM 맵 생성 모드.

f1tenth_stack 브링업(센서+VESC+mux) + slam_toolbox(online async, mapping).
키보드 텔레옵은 tty가 필요해 race.sh가 포그라운드로 따로 띄운다.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    f1tenth_share = get_package_share_directory('f1tenth_stack')
    slam_params = os.path.join(f1tenth_share, 'config',
                               'f1tenth_online_async.yaml')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_share, 'launch', 'bringup_launch.py')))

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params])

    return LaunchDescription([bringup, slam])
