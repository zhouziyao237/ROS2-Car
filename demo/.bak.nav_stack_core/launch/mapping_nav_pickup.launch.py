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

    target_code = LaunchConfiguration('target_code')
    target_frame = LaunchConfiguration('target_frame')

    # 你的原始边建图边导航 launch
    # 如果你的文件名不是这个，请改这里
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

    base_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_nav_launch),
        launch_arguments={
            # 这里保持你原导航系统默认配置
            # serial 输出仍然是 /cmd_vel_selected
            'serial_cmd_vel_topic': '/cmd_vel_selected',
        }.items()
    )

    pickup_pipeline = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pickup_launch),
                launch_arguments={
                    'target_code': target_code,
                    'target_frame': target_frame,

                    # QR pipeline 内只启动识别、深度定位、升降
                    # 底盘对准由下面单独 qr_chassis_align_control 启动
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

    return LaunchDescription([
        DeclareLaunchArgument('target_code', default_value='4309'),
        DeclareLaunchArgument('target_frame', default_value='oak_camera_link'),

        base_nav,
        pickup_pipeline,
        qr_align,
    ])
