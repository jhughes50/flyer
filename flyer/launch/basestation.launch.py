"""
    Jason Hughes
    January 2025

"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_dir = get_package_share_directory("flyer")

    viz_node = Node(package="flyer",
                    executable="flyer_viz",
                    name="flyer_viz",
                    output="screen",
                    emulate_tty=True,
                    parameters=[{"directory": share_dir,
                                 "ip_address": "127.0.0.1",
                                 "port": 5000}]])

    mocha_node = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('mocha_launch'),
                            'launch',
                            'basestation.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'robot_name': 'flyer-base',
                    }.items()
                )

    return LaunchDescription([Node(package="flyer",
                                   executable="flyer_viz",
                                   name="flyer_viz",
                                   output="screen",
                                   emulate_tty=True,
                                   parameters=[{"directory": share_dir,
                                                "ip_address": "127.0.0.1",
                                                "port": 5000}]])
