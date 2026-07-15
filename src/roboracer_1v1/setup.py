from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roboracer_1v1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='orin@todo.todo',
    description='RoboRacer 1v1 race stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'opponent_tracker_node = roboracer_1v1.opponent_tracker_node:main',
            'gap_follow_node = roboracer_1v1.gap_follow_node:main',
            'overtaking_node = roboracer_1v1.overtaking_node:main',
            'fully_autonomous_manager = roboracer_1v1.fully_autonomous_manager:main',
            'lap_counter_node = roboracer_1v1.lap_counter_node:main',
            'emergency_stop_node = roboracer_1v1.emergency_stop_node:main',
            'pure_pursuit_node = roboracer_1v1.pure_pursuit_node:main',
            'waypoint_recorder_node = roboracer_1v1.waypoint_recorder_node:main',
        ],
    },
)
