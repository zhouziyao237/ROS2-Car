#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    start_oak = LaunchConfiguration('start_oak')
    start_matcher = LaunchConfiguration('start_matcher')
    start_overlay = LaunchConfiguration('start_overlay')
    start_decision = LaunchConfiguration('start_decision')

    oak_params_file = LaunchConfiguration('oak_params_file')
    blob_path = LaunchConfiguration('blob_path')
    conf = LaunchConfiguration('conf')
    iou = LaunchConfiguration('iou')

    image_topic = LaunchConfiguration('image_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    target_frame = LaunchConfiguration('target_frame')
    publish_optical_tf = LaunchConfiguration('publish_optical_tf')
    camera_link_frame = LaunchConfiguration('camera_link_frame')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')

    auto_config = os.path.join(
        get_package_share_directory('target_auto_decision'),
        'config',
        'auto_decision.yaml'
    )

    oak_launch = os.path.join(
        get_package_share_directory('oak_vision_driver'),
        'launch',
        'oak_vision_driver.launch.py'
    )

    overlay_launch = os.path.join(
        get_package_share_directory('target_pose_bridge'),
        'launch',
        'single_camera_target_overlay.launch.py'
    )

    oak = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oak_launch),
        launch_arguments={
            'params_file': oak_params_file,
            'enable_depth': 'true',
            'blob_path': blob_path,
            'conf': conf,
            'iou': iou,
        }.items(),
        condition=IfCondition(start_oak)
    )

    matcher = Node(
        package='target_matcher',
        executable='target_matcher_node',
        name='target_matcher_node',
        output='screen',
        condition=IfCondition(start_matcher)
    )

    overlay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(overlay_launch),
        launch_arguments={
            'image_topic': image_topic,
            'detections_topic': detections_topic,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
            'target_frame': target_frame,
            'publish_optical_tf': publish_optical_tf,
            'camera_link_frame': camera_link_frame,
            'camera_optical_frame': camera_optical_frame,
        }.items(),
        condition=IfCondition(start_overlay)
    )

    decision = Node(
        package='target_auto_decision',
        executable='auto_decision_node.py',
        name='auto_decision_node',
        output='screen',
        parameters=[auto_config, {
            'target_pose_topic': '/target_pose_base',
        }],
        condition=IfCondition(start_decision)
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_oak', default_value='true'),
        DeclareLaunchArgument('start_matcher', default_value='false'),
        DeclareLaunchArgument('start_overlay', default_value='true'),
        DeclareLaunchArgument('start_decision', default_value='true'),
        DeclareLaunchArgument('oak_params_file', default_value='/host-home/src/oak_vision_driver/config/oak_yolo_runtime_ros_params.yaml'),
        DeclareLaunchArgument('blob_path', default_value='/host-home/src/oak_vision_driver/weights/best_fp16.blob'),
        DeclareLaunchArgument('conf', default_value='0.14'),
        DeclareLaunchArgument('iou', default_value='0.25'),
        DeclareLaunchArgument('image_topic', default_value='/oak/rgb/image_raw'),
        DeclareLaunchArgument('detections_topic', default_value='/yolov5/detections'),
        DeclareLaunchArgument('depth_topic', default_value='/oak/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/oak/rgb/camera_info'),
        DeclareLaunchArgument('target_frame', default_value='oak_camera_link'),
        DeclareLaunchArgument('publish_optical_tf', default_value='true'),
        DeclareLaunchArgument('camera_link_frame', default_value='oak_camera_link'),
        DeclareLaunchArgument('camera_optical_frame', default_value='oak_rgb_camera_optical_frame'),
        oak,
        matcher,
        overlay,
        decision,
    ])
