#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    oak_dir = get_package_share_directory('oak_vision_driver')
    oak_launch = os.path.join(oak_dir, 'launch', 'oak_vision_driver.launch.py')

    pose_bridge_dir = get_package_share_directory('target_pose_bridge')
    overlay_launch = os.path.join(pose_bridge_dir, 'launch', 'single_camera_target_overlay.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('start_oak', default_value='true'),
        DeclareLaunchArgument('start_overlay', default_value='true'),
        DeclareLaunchArgument('oak_params_file', default_value='/host-home/src/target_detection/oak_vision_driver/config/oak_yolo_runtime_ros_params.yaml'),
        DeclareLaunchArgument('oak_blob_path', default_value='/host-home/src/target_detection/oak_vision_driver/weights/best_fp16.blob'),
        DeclareLaunchArgument('oak_conf', default_value='0.14'),
        DeclareLaunchArgument('oak_iou', default_value='0.25'),
        DeclareLaunchArgument('target_frame', default_value='oak_camera_link'),
        DeclareLaunchArgument('publish_optical_tf', default_value='true'),
        DeclareLaunchArgument('camera_link_frame', default_value='oak_camera_link'),
        DeclareLaunchArgument('camera_optical_frame', default_value='oak_rgb_camera_optical_frame'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oak_launch),
            launch_arguments={
                'params_file': LaunchConfiguration('oak_params_file'),
                'enable_depth': 'true',
                'blob_path': LaunchConfiguration('oak_blob_path'),
                'conf': LaunchConfiguration('oak_conf'),
                'iou': LaunchConfiguration('oak_iou'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_oak')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(overlay_launch),
            launch_arguments={
                'image_topic': '/oak/rgb/image_raw',
                'detections_topic': '/yolov5/detections',
                'depth_topic': '/oak/depth/image_raw',
                'camera_info_topic': '/oak/rgb/camera_info',
                'target_frame': LaunchConfiguration('target_frame'),
                'publish_optical_tf': LaunchConfiguration('publish_optical_tf'),
                'camera_link_frame': LaunchConfiguration('camera_link_frame'),
                'camera_optical_frame': LaunchConfiguration('camera_optical_frame'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_overlay')),
        ),
    ])
