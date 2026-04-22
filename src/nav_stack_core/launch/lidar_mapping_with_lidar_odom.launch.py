#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_stack_core')
    default_xacro = os.path.join(pkg_share, 'description', 'minimal_lidar_robot.urdf.xacro')
    default_slam = os.path.join(pkg_share, 'config', 'slam_toolbox_online.yaml')

    xacro_path = LaunchConfiguration('xacro_path')
    slam_params_file = LaunchConfiguration('slam_params_file')

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('xacro_path', default_value=default_xacro),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
