#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('target_pose_bridge')
    config_file = os.path.join(pkg_dir, 'config', 'target_pose_bridge.yaml')

    base_frame = LaunchConfiguration('base_frame')
    camera_link_frame = LaunchConfiguration('camera_link_frame')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')
    publish_optical_tf = LaunchConfiguration('publish_optical_tf')
    target_frame = LaunchConfiguration('target_frame')

    optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_camera_link_to_optical_tf',
        arguments=[
            '0.0', '0.0', '0.0',
            '-1.57079632679', '0.0', '-1.57079632679',
            camera_link_frame,
            camera_optical_frame
        ],
        condition=IfCondition(publish_optical_tf)
    )

    fusion_node = Node(
        package='target_pose_bridge',
        executable='target_3d_fusion_node.py',
        name='target_3d_fusion_node',
        output='screen',
        parameters=[config_file, {
            'detections_topic': LaunchConfiguration('detections_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        }]
    )

    tf_node = Node(
        package='target_pose_bridge',
        executable='target_tf_transform_node.py',
        name='target_to_base_node',
        output='screen',
        parameters=[{
            'input_topic': '/target_pose_camera',
            'output_topic': '/target_pose_base',
            'target_frame': target_frame,
            'debug_log': True,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('camera_link_frame', default_value='oak_camera_link'),
        DeclareLaunchArgument('camera_optical_frame', default_value='oak_rgb_camera_optical_frame'),
        DeclareLaunchArgument('publish_optical_tf', default_value='false'),
        DeclareLaunchArgument('detections_topic', default_value='/target_matcher/matched_target'),
        DeclareLaunchArgument('depth_topic', default_value='/oak/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/oak/rgb/camera_info'),
        optical_tf,
        fusion_node,
        tf_node,
    ])
