#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    semantic_map_path = LaunchConfiguration('semantic_map_path')
    reset_semantic_map = LaunchConfiguration('reset_semantic_map')
    run_id = LaunchConfiguration('run_id')
    use_rviz_restrict = LaunchConfiguration('use_rviz_restrict')
    use_goal_relay = LaunchConfiguration('use_goal_relay')
    use_nav_to_qr = LaunchConfiguration('use_nav_to_qr')

    return LaunchDescription([
        DeclareLaunchArgument('semantic_map_path', default_value='/host-home/maps/semantic_library.json'),
        DeclareLaunchArgument('reset_semantic_map', default_value='false'),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('use_rviz_restrict', default_value='true'),
        DeclareLaunchArgument('use_goal_relay', default_value='true'),
        DeclareLaunchArgument('use_nav_to_qr', default_value='true'),

        Node(
            package='robot_logistics_semantic', executable='semantic_relocalizer_node',
            name='semantic_relocalizer_node', output='screen',
            parameters=[{'use_sim_time': False, 'semantic_map_path': semantic_map_path, 'run_id': run_id}]
        ),

        Node(
            package='robot_logistics_semantic', executable='vision_qr_depth_node',
            name='vision_qr_depth_node', output='screen',
            parameters=[{
                'use_sim_time': False,
                'enabled': True,
                'map_frame': 'map',
                'robot_frame': 'base_link',
                'rgb_topic': '/camera/camera/color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'depth_topics': ['/camera/camera/aligned_depth_to_color/image_raw'],
                'detection_radius': 4.0,
                'publish_period_sec': 0.5,
            }]
        ),

        Node(
            package='robot_logistics_semantic', executable='semantic_registrar_node',
            name='semantic_registrar_node', output='screen',
            parameters=[{
                'use_sim_time': False,
                'semantic_map_path': semantic_map_path,
                'reset_semantic_map': reset_semantic_map,
                'map_frame': 'map',
                'robot_frame': 'base_link',
                'run_id': run_id,
                'semantic_observation_topic': '/vision/semantic_observation',
                'semantic_observation_json_topic': '/vision/semantic_observation_json',
                'qr_info_topic': '/vision/qr_info',
                'target_pose_topic': '/target_pose_base',
                'default_rows': 3,
                'default_cols': 3,
                'shelf_width': 1.85,
                'shelf_depth': 0.35,
                'shelf_height': 1.50,
                'approach_distance': 0.90,
                'infer_yaw_from_robot': False,
                'single_anchor_relocalize': True,
                'tf_timeout_sec': 0.5,
            }]
        ),

        Node(
            package='robot_logistics_semantic', executable='semantic_marker_publisher_node',
            name='semantic_marker_publisher_node', output='screen',
            parameters=[{'use_sim_time': False, 'semantic_map_path': semantic_map_path, 'frame_id': 'map', 'publish_period_sec': 0.5}]
        ),

        Node(
            package='robot_logistics_semantic', executable='nav_to_qr_server',
            name='nav_to_qr_server', output='screen', condition=IfCondition(use_nav_to_qr),
            parameters=[{'use_sim_time': False, 'map_frame': 'map', 'default_stop_distance': 1.2}]
        ),

        Node(
            package='robot_logistics_semantic', executable='rviz_goal_relay_node',
            name='rviz_goal_relay_node', output='screen', condition=IfCondition(use_goal_relay),
            parameters=[{'use_sim_time': False, 'goal_pose_topic': '/goal_pose'}]
        ),

        Node(
            package='robot_logistics_semantic', executable='frontier_explorer_node',
            name='frontier_explorer_node', output='screen',
            parameters=[{
                'use_sim_time': False,
                'map_topic': '/map',
                'enable_topic': '/explore/enable',
                'status_topic': '/explore/status',
                'map_frame': 'map',
                'robot_frame': 'base_link',
                'goal_period_sec': 3.0,
                'goal_timeout_sec': 45.0,
                'min_frontier_cluster_size': 5,
                'min_frontier_distance': 0.80,
                'max_frontier_distance': 8.0,
                'frontier_blacklist_radius': 1.20,
                'unknown_value': -1,
                'free_threshold': 10,
                'occupied_threshold': 65,
                'info_gain_weight': 0.12,
                'distance_weight': 0.45,
                'prefer_forward_weight': 0.10,
            }]
        ),

        Node(
            package='robot_logistics_semantic', executable='semantic_task_manager_node',
            name='semantic_task_manager_node', output='screen',
            parameters=[{
                'use_sim_time': False,
                'semantic_map_path': semantic_map_path,
                'explore_enable_topic': '/explore/enable',
                'task_timeout_sec': 300.0,
                'check_period_sec': 1.0,
                'stop_distance': 1.2,
                'require_cell_confirm': False,
            }]
        ),

        Node(
            package='robot_logistics_semantic', executable='rviz_restrict_region_node',
            name='rviz_restrict_region_node', output='screen', condition=IfCondition(use_rviz_restrict),
            parameters=[{'use_sim_time': False, 'map_frame': 'map'}]
        ),
    ])
