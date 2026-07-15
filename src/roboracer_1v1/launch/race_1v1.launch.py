from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('roboracer_1v1'), 'config', 'race_1v1_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file),
        LogInfo(msg='[race_1v1] Launching 1v1 race stack...'),

        Node(package='roboracer_1v1', executable='opponent_tracker_node',
             name='opponent_tracker_node', output='screen',
             parameters=[LaunchConfiguration('params_file')]),

        Node(package='roboracer_1v1', executable='gap_follow_node',
             name='gap_follow_node', output='screen',
             parameters=[LaunchConfiguration('params_file')]),

        Node(package='roboracer_1v1', executable='overtaking_node',
             name='overtaking_node', output='screen',
             parameters=[LaunchConfiguration('params_file')]),

        Node(package='roboracer_1v1', executable='fully_autonomous_manager',
             name='fully_autonomous_manager', output='screen',
             parameters=[LaunchConfiguration('params_file')]),

        Node(package='roboracer_1v1', executable='lap_counter_node',
             name='lap_counter_node', output='screen',
             parameters=[LaunchConfiguration('params_file')]),

        Node(package='roboracer_1v1', executable='emergency_stop_node',
             name='emergency_stop_node', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
    ])