from setuptools import setup
import os
from glob import glob

package_name = 'q2_gap_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BlueTwin',
    maintainer_email='blue@twin.local',
    description='Q2 — wall-follow backbone with gap-follow obstacle avoidance',
    license='MIT',
    entry_points={
        'console_scripts': [
            'q2_driver = q2_gap_follow.q2_driver_node:main',
        ],
    },
)
