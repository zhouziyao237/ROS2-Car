#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('platform_target_type', default_value='auto_aim_interfaces/msg/PlatformTarget'),
        DeclareLaunchArgument('initial_lift_target_mm', default_value='0.0'),
        DeclareLaunchArgument('lift_direction', default_value='1.0'),

        Node(
            package='qr_platform_lift_control',
            executable='qr_lift_control_node',
            name='qr_lift_control_node',
            output='screen',
            parameters=[{
                'target_pose_topic': '/target_pose_base',
                'platform_target_topic': '/platform/target',

                'platform_target_type': LaunchConfiguration('platform_target_type'),
                'lift_field': 'target_lift_z_mm',
                'slide_field': 'target_slide_x_mm',
                'screw_field': 'target_screw_x_mm',

                'platform_min_height_mm': 0.0,
                'platform_max_height_mm': 280.0,
                'initial_lift_target_mm': ParameterValue(LaunchConfiguration('initial_lift_target_mm'), value_type=float),

                'z_target_mm': 0.0,
                'kp_z': 0.08,
                'max_step_mm': 1.0,
                'deadband_mm': 30.0,
                'release_deadband_mm': 45.0,
                'hold_required_frames': 5,
                'lift_direction': ParameterValue(LaunchConfiguration('lift_direction'), value_type=float),

                'valid_abs_z_max_mm': 900.0,
                'valid_abs_y_max_mm': 900.0,
                'max_jump_z_mm': 900.0,

                'filter_window': 7,
                'stable_required': 3,

                'lost_timeout_sec': 0.5,
                'return_home_step_mm': 4.0,

                'slide_hold_mm': 0.0,
                'screw_hold_mm': 0.0,
            }],
        ),
    ])