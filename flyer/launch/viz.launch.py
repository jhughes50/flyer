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

    share_dir = get_package_share_directory("symbiote_ag")

    return LaunchDescription([Node(package="symbiote_ag",
                                   executable="symbiote_map",
                                   name="symbiote_map",
                                   output="screen",
                                   emulate_tty=True,
                                   #arguments=['--ros-args', '--log-level', 'DEBUG'],
                                   remappings=[("/gps", "/ublox/fix"),])])
