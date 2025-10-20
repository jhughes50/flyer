"""
    Jason Hughes
    January 2025

    Launch the factor graph node
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_dir = get_package_share_directory("flyer")

    return LaunchDescription([Node(package="flyer",
                                   executable="flyer_viz",
                                   name="flyer_viz",
                                   output="screen",
                                   emulate_tty=True,
                                   parameters=[{"directory": share_dir,
                                                "ip_address": "127.0.0.1",
                                                "port": 5000}],
                                   remappings=[("/gps", "/ublox_gps_node/fix"),
                                               ("/odom", "/glider/odom")])])
