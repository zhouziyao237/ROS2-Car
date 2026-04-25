#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('m10p_lidar_odom_bringup')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(pkg_share, 'worlds', 'm10p_odom_test.world')
    default_urdf = os.path.join(pkg_share, 'urdf', 'm10p_test_robot.urdf.xacro')
    default_params = os.path.join(pkg_share, 'config', 'm10p_lidar_odom_sim.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'm10p_lidar_odom.rviz')

    world = LaunchConfiguration('world')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_drive = LaunchConfiguration('auto_drive')
    gui = LaunchConfiguration('gui')
    spawn_z = LaunchConfiguration('spawn_z')

    # ROS2 Humble needs robot_description to be explicitly typed as string.
    # Otherwise Launch may try to parse the generated XML as YAML.
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', default_urdf,
            ' use_sim_time:=', use_sim_time,
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': 'false',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'm10p_test_robot',
            '-x', '0', '-y', '0', '-z', spawn_z,
        ],
        output='screen',
    )

    lidar_odom = Node(
        package='lidar_odom_pkg',
        executable='scan_match_odom',
        name='scan_match_odom',
        output='screen',
        parameters=[params_file],
    )

    lidar_path = Node(
        package='m10p_lidar_odom_bringup',
        executable='odom_to_path',
        name='lidar_path',
        output='screen',
        parameters=[params_file],
    )

    truth_path = Node(
        package='m10p_lidar_odom_bringup',
        executable='odom_to_path',
        name='truth_path',
        output='screen',
        parameters=[params_file],
    )

    error_monitor = Node(
        package='m10p_lidar_odom_bringup',
        executable='odom_error_monitor',
        name='odom_error_monitor',
        output='screen',
        parameters=[params_file],
    )

    odom_truth_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_odom_truth_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'odom_truth'],
        output='screen',
    )

    cmd_vel_demo = Node(
        package='m10p_lidar_odom_bringup',
        executable='cmd_vel_demo',
        name='cmd_vel_demo',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(auto_drive),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_drive', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('spawn_z', default_value='0.0'),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        odom_truth_static_tf,
        TimerAction(period=2.0, actions=[lidar_odom, lidar_path, truth_path, error_monitor]),
        TimerAction(period=3.0, actions=[cmd_vel_demo]),
        TimerAction(period=3.0, actions=[rviz]),
    ])
