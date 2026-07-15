"""Q2 launch — Q2 드라이버 + 파라미터 yaml 로드."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('q2_gap_follow'),
        'config', 'q2_params.yaml'
    ])

    q2_node = Node(
        package='q2_gap_follow',
        executable='q2_driver',
        name='q2_driver',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([q2_node])
