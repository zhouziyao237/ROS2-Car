#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('nav_stack_core')
    pickup_pkg = get_package_share_directory('target_pickup_bringup')
    align_pkg = get_package_share_directory('qr_chassis_align_control')
    decision_pkg = get_package_share_directory('pickup_decision_manager')

    target_frame = LaunchConfiguration('target_frame')
    initial_target_code = LaunchConfiguration('initial_target_code')

    base_nav_launch = os.path.join(
        nav_pkg,
        'launch',
        'mapping_nav_with_serial_wheel_heading.launch.py'
    )

    pickup_launch = os.path.join(
        pickup_pkg,
        'launch',
        'qr_pickup_pipeline.launch.py'
    )

    qr_align_config = os.path.join(
        align_pkg,
        'config',
        'qr_chassis_align_nav_mux.yaml'
    )

    decision_config = os.path.join(
        decision_pkg,
        'config',
        'pickup_decision.yaml'
    )

    base_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_nav_launch),
        launch_arguments={
            'serial_cmd_vel_topic': '/cmd_vel_selected',
        }.items()
    )

    pickup_pipeline = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pickup_launch),
                launch_arguments={
                    'target_code': initial_target_code,
                    'target_frame': target_frame,
                    'start_lift': 'true',
                    'start_chassis_align': 'false',
                }.items()
            )
        ]
    )

    qr_align = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='qr_chassis_align_control',
                executable='qr_chassis_align_node',
                name='qr_chassis_align_node',
                output='screen',
                parameters=[qr_align_config],
            )
        ]
    )

    decision_manager = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='pickup_decision_manager',
                executable='pickup_decision_manager_node',
                name='pickup_decision_manager_node',
                output='screen',
                parameters=[decision_config],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_frame', default_value='oak_camera_link'),
        DeclareLaunchArgument('initial_target_code', default_value='COL:1'),

        base_nav,
        pickup_pipeline,
        qr_align,
        decision_manager,
    ])