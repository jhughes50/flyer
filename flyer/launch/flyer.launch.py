"""
    Jason Hughes
    January 2025

    Launch the flyer node
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Find package share directory
    flyer_share = FindPackageShare('flyer')

    gdino_config_path = PathJoinSubstitution([flyer_share, 
                                             'config', 
                                             'grounding_dino.yaml'])

    sam2_config_path = PathJoinSubstitution([flyer_share,
                                            'config',
                                            'sam2.1_hiera_l.yaml'])

    calib_config_path = PathJoinSubstitution([flyer_share,
                                             'config',
                                             'blackfly-evmapper.yaml'])

    node = Node(package="flyer",
                executable="flyer_node",
                name="flyer_node",
                output="screen",
                emulate_tty=True,
                remappings=[("/image", "/flir/image_raw/compressed"),
                            ("/odom", "/glider/odom"),
                            ("/text", "/text")],
                parameters=[{"use_sim_time": use_sim_time,
                             "gdino_config_path": gdino_config_path,
                             "sam2_config_path": sam2_config_path,
                             "calib_config_path": calib_config_path,
                             "use_compressed": True,
                             "inference_rate": 1,
                             "downres_output": False}])

    return LaunchDescription([use_sim_time_arg, node])

