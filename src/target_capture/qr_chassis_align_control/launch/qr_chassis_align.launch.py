#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('qr_chassis_align_control')
    default_config = os.path.join(pkg_dir, 'config', 'qr_chassis_align.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
        ),

        Node(
            package='qr_chassis_align_control',
            executable='qr_chassis_align_node',
            name='qr_chassis_align_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])