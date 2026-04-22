#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_stack_core')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    default_xacro = os.path.join(pkg_share, 'description', 'minimal_lidar_robot.xacro')
    default_slam = os.path.join(pkg_share, 'config', 'slam_toolbox_online.yaml')
    default_nav2 = os.path.join(pkg_share, 'config', 'nav2_params_min.yaml')

    xacro_path = LaunchConfiguration('xacro_path')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    nav2_launch = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
    slam_launch = os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
    default_rviz = os.path.join(pkg_share, 'rviz', 'nav_stack.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('xacro_path', default_value=default_xacro),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': slam_params_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'params_file': nav2_params_file,
                'use_sim_time': 'false',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz],
        ),
    ])
