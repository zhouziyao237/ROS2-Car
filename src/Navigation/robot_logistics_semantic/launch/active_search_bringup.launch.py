#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ws = Path('/host-home/robot_logistics_tb3_rf2o_ws')
    world = str(ws / 'src' / 'robot_logistics_sim' / 'worlds' / 'warehouse_semantic.world')
    urdf_xacro = str(ws / 'src' / 'robot_logistics_sim' / 'urdf' / 'logistics_robot.urdf.xacro')
    slam_params = str(ws / 'src' / 'robot_logistics_semantic' / 'config' / 'slam_toolbox_sim.yaml')
    nav2_params = str(ws / 'src' / 'robot_logistics_semantic' / 'config' / 'nav2_params.yaml')
    rviz_config = str(ws / 'src' / 'robot_logistics_semantic' / 'rviz' / 'nav_stack.rviz')
    semantic_map_path = LaunchConfiguration('semantic_map_path')
    reset_semantic_map = LaunchConfiguration('reset_semantic_map')
    run_id = LaunchConfiguration('run_id')
    robot_description = ParameterValue(Command(['xacro ', urdf_xacro]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('semantic_map_path', default_value='/host-home/maps/semantic_library.json'),
        DeclareLaunchArgument('reset_semantic_map', default_value='false'),
        DeclareLaunchArgument('run_id', default_value=''),

        ExecuteProcess(cmd=['gzserver', world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_state.so'], output='screen'),
        ExecuteProcess(cmd=['gzclient'], output='screen'),

        TimerAction(period=2.0, actions=[
            Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen', parameters=[{'use_sim_time': True, 'robot_description': robot_description}]),
            Node(package='joint_state_publisher', executable='joint_state_publisher', name='joint_state_publisher', output='screen', parameters=[{'use_sim_time': True}]),
            Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_logistics_robot', output='screen', arguments=['-topic', 'robot_description', '-entity', 'logistics_robot', '-x', '0.0', '-y', '0.0', '-z', '0.08', '-Y', '0.0']),
        ]),

        TimerAction(period=5.0, actions=[
            Node(package='robot_logistics_semantic', executable='sim_model_state_odom_node', name='rf2o_laser_odometry', output='screen', parameters=[{'use_sim_time': True, 'laser_scan_topic': '/scan', 'odom_topic': '/odom_laser', 'publish_tf': False, 'base_frame_id': 'd435-cam-Link', 'odom_frame_id': 'odom', 'init_pose_from_topic': '', 'freq': 12.0}]),
            Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen', parameters=[slam_params]),
        ]),

        TimerAction(period=9.0, actions=[
            Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[nav2_params]),
            Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[nav2_params]),
            Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[nav2_params]),
            Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[nav2_params]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen', parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0, 'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']}]),
        ]),

        TimerAction(period=13.0, actions=[
            Node(package='robot_logistics_semantic', executable='semantic_relocalizer_node', name='semantic_relocalizer_node', output='screen', parameters=[{'use_sim_time': True, 'semantic_map_path': semantic_map_path, 'run_id': run_id}]),
            Node(package='robot_logistics_semantic', executable='semantic_registrar_node', name='semantic_registrar_node', output='screen', parameters=[{'use_sim_time': True, 'semantic_map_path': semantic_map_path, 'map_frame': 'map', 'robot_frame': 'base_footprint', 'run_id': run_id, 'semantic_observation_topic': '/vision/semantic_observation', 'semantic_observation_json_topic': '/vision/semantic_observation_json', 'qr_info_topic': '/vision/qr_info', 'target_pose_topic': '/target_pose_base', 'default_rows': 3, 'default_cols': 3, 'infer_yaw_from_robot': True, 'single_anchor_relocalize': True}]),
            Node(package='robot_logistics_semantic', executable='semantic_marker_publisher_node', name='semantic_marker_publisher_node', output='screen', parameters=[{'use_sim_time': True, 'semantic_map_path': semantic_map_path, 'frame_id': 'map', 'publish_period_sec': 0.5}]),
            Node(package='robot_logistics_semantic', executable='nav_to_qr_server', name='nav_to_qr_server', output='screen', parameters=[{'use_sim_time': True, 'map_frame': 'map', 'default_stop_distance': 1.2}]),
            Node(package='robot_logistics_semantic', executable='rviz_goal_relay_node', name='rviz_goal_relay_node', output='screen', parameters=[{'use_sim_time': True, 'goal_pose_topic': '/goal_pose'}]),
            Node(package='robot_logistics_semantic', executable='frontier_explorer_node', name='frontier_explorer_node', output='screen', parameters=[{'use_sim_time': True, 'map_topic': '/map', 'enable_topic': '/explore/enable', 'status_topic': '/explore/status', 'map_frame': 'map', 'robot_frame': 'base_footprint', 'goal_period_sec': 3.0, 'goal_timeout_sec': 35.0, 'min_frontier_cluster_size': 3, 'min_frontier_distance': 0.45, 'max_frontier_distance': 8.0, 'frontier_blacklist_radius': 0.50, 'unknown_value': -1, 'free_threshold': 10, 'occupied_threshold': 65, 'info_gain_weight': 0.04, 'distance_weight': 1.0, 'prefer_forward_weight': 0.10}]),
            Node(package='robot_logistics_semantic', executable='vision_qr_depth_node', name='vision_qr_depth_node', output='screen', parameters=[{'use_sim_time': True, 'enabled': True, 'map_frame': 'map', 'robot_frame': 'base_footprint', 'detection_radius': 4.0, 'publish_period_sec': 0.8}]),
            Node(package='robot_logistics_semantic', executable='semantic_task_manager_node', name='semantic_task_manager_node', output='screen', parameters=[{'use_sim_time': True, 'semantic_map_path': semantic_map_path, 'explore_enable_topic': '/explore/enable', 'task_timeout_sec': 240.0, 'check_period_sec': 1.0, 'stop_distance': 1.2, 'require_cell_confirm': False}]),
            Node(package='robot_logistics_semantic', executable='depth_align_node', name='depth_align_node', output='screen', parameters=[{'use_sim_time': True, 'auto_success_for_sim': True}]),
            Node(package='robot_logistics_semantic', executable='grab_execute_node', name='grab_execute_node', output='screen', parameters=[{'use_sim_time': True}]),
        ]),

        TimerAction(period=14.0, actions=[
            Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', parameters=[{'use_sim_time': True}], arguments=['-d', rviz_config])
        ]),
    
            Node(
                package='robot_logistics_semantic',
                executable='rviz_restrict_region_node',
                name='rviz_restrict_region_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'map_frame': 'map',
                                'front_qr_yaw_offset_deg': 0.0,
                'use_single_qr_normal_yaw': False,
}]
            ),
])
