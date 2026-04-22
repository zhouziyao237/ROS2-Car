#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_odom_pkg')
    default_params = os.path.join(pkg_share, 'config', 'lidar_odom.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to lidar odometry parameter file'
        ),
        Node(
            package='lidar_odom_pkg',
            executable='scan_match_odom.py',
            name='scan_match_odom',
            output='screen',
            parameters=[params_file]
        )
    ])