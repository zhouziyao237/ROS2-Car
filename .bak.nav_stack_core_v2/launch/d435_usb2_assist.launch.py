#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    start_realsense = LaunchConfiguration('start_realsense')
    start_d435_scan = LaunchConfiguration('start_d435_scan')

    depth_profile = LaunchConfiguration('depth_profile')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    output_scan_topic = LaunchConfiguration('output_scan_topic')
    output_frame = LaunchConfiguration('output_frame')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        condition=IfCondition(start_realsense),
        launch_arguments={
            'enable_color': 'false',
            'enable_depth': 'true',

            'pointcloud.enable': 'false',
            'align_depth.enable': 'false',
            'enable_sync': 'false',

            'enable_infra': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',

            'depth_module.depth_profile': depth_profile,
        }.items()
    )

    d435_depth_to_scan = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='nav_stack_core_v2',
                executable='d435_depth_to_scan',
                name='d435_depth_to_scan_node',
                output='screen',
                condition=IfCondition(start_d435_scan),
                parameters=[
                    {
                        'depth_image_topic': depth_image_topic,
                        'camera_info_topic': camera_info_topic,
                        'output_scan_topic': output_scan_topic,
                        'output_frame': output_frame,
                        'scan_height_pixels': 8,
                        'range_min': 0.25,
                        'range_max': 2.5,
                        'angle_min_deg': -30.0,
                        'angle_max_deg': 30.0,
                    }
                ],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('start_d435_scan', default_value='true'),

        DeclareLaunchArgument('depth_profile', default_value='640x480x6'),

        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/camera/depth/image_rect_raw'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera/depth/camera_info'
        ),
        DeclareLaunchArgument(
            'output_scan_topic',
            default_value='/d435_scan'
        ),
        DeclareLaunchArgument(
            'output_frame',
            default_value='base_link'
        ),

        realsense_launch,
        d435_depth_to_scan,
    ])
