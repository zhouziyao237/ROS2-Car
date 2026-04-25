#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
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
    default_rviz = os.path.join(pkg_share, 'rviz', 'nav_stack.rviz')

    xacro_path = LaunchConfiguration('xacro_path')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    launch_robot_state_publisher = LaunchConfiguration('launch_robot_state_publisher')
    nav2_start_delay = LaunchConfiguration('nav2_start_delay')
    rviz_start_delay = LaunchConfiguration('rviz_start_delay')

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    nav2_launch = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
    slam_launch = os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('xacro_path', default_value=default_xacro),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('launch_robot_state_publisher', default_value='true'),
        DeclareLaunchArgument('nav2_start_delay', default_value='4.0'),
        DeclareLaunchArgument('rviz_start_delay', default_value='5.0'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(launch_robot_state_publisher),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': slam_params_file,
            }.items(),
        ),
        TimerAction(
            period=nav2_start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
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
