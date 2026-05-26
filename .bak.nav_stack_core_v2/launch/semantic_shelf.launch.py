#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    semantic_map_path = LaunchConfiguration('semantic_map_path')
    reset_semantic_map = LaunchConfiguration('reset_semantic_map')

    semantic_launch_path = os.path.join(
        get_package_share_directory('robot_logistics_semantic'),
        'launch',
        'real_semantic_only.launch.py'
    )

    semantic_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(semantic_launch_path),
        launch_arguments={
            'semantic_map_path': semantic_map_path,
            'reset_semantic_map': reset_semantic_map,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'semantic_map_path',
            default_value='/host-home/maps/semantic_library.json'
        ),
        DeclareLaunchArgument(
            'reset_semantic_map',
            default_value='false'
        ),
        semantic_system,
    ])
