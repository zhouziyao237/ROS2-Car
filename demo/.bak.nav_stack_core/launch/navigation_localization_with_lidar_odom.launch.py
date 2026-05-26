#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_stack_core')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_xacro = os.path.join(pkg_share, 'description', 'minimal_lidar_robot.xacro')
    default_nav2 = os.path.join(pkg_share, 'config', 'nav2_params_localization.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'nav_stack.rviz')

    xacro_path = LaunchConfiguration('xacro_path')
    map_yaml = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    launch_robot_state_publisher = LaunchConfiguration('launch_robot_state_publisher')
    localization_start_delay = LaunchConfiguration('localization_start_delay')
    navigation_start_delay = LaunchConfiguration('navigation_start_delay')
    rviz_start_delay = LaunchConfiguration('rviz_start_delay')

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    localization_launch = os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
    navigation_launch = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('xacro_path', default_value=default_xacro),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('launch_robot_state_publisher', default_value='true'),
        DeclareLaunchArgument('localization_start_delay', default_value='3.0'),
        DeclareLaunchArgument('navigation_start_delay', default_value='4.0'),
        DeclareLaunchArgument('rviz_start_delay', default_value='5.0'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(launch_robot_state_publisher),
        ),
        TimerAction(
            period=localization_start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(localization_launch),
                    launch_arguments={
                        'map': map_yaml,
                        'params_file': nav2_params_file,
                        'use_sim_time': 'false',
                        'autostart': 'true',
                        'use_respawn': 'true',
                        'log_level': 'info',
                    }.items(),
                ),
            ],
        ),
        TimerAction(
            period=navigation_start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_launch),
                    launch_arguments={
                        'params_file': nav2_params_file,
                        'use_sim_time': 'false',
                        'autostart': 'true',
                        'use_respawn': 'true',
                        'log_level': 'info',
                    }.items(),
                ),
                Node(
                    package='nav_stack_core',
                    executable='goal_pose_bridge.py',
                    name='goal_pose_bridge',
                    output='screen',
                    parameters=[{
                        'goal_topic': '/goal_pose',
                        'action_name': '/navigate_to_pose',
                        'marker_topic': '/goal_markers',
                        'cancel_previous_goal': True,
                    }],
                ),
            ],
        ),
        TimerAction(
            period=rviz_start_delay,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config],
                ),
            ],
        ),
    ])
