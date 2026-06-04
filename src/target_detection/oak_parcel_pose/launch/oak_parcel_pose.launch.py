#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('oak_parcel_pose')
    default_params = os.path.join(pkg_share, 'config', 'datamatrix_depth.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to ROS2 parameters YAML file.',
        ),
        Node(
            package='oak_parcel_pose',
            executable='oak_parcel_pose_node',
            name='oak_parcel_pose_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
