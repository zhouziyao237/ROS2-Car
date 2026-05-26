#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('target_pickup_bringup')
    pickup_control_config = os.path.join(bringup_dir, 'config', 'pickup_control.yaml')

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

    target_code = LaunchConfiguration('target_code')
    target_frame = LaunchConfiguration('target_frame')

    start_oak = LaunchConfiguration('start_oak')
    start_qr = LaunchConfiguration('start_qr')
    start_overlay = LaunchConfiguration('start_overlay')
    start_lift = LaunchConfiguration('start_lift')
    start_chassis_align = LaunchConfiguration('start_chassis_align')

    oak_params_file = LaunchConfiguration('oak_params_file')
    oak_blob_path = LaunchConfiguration('oak_blob_path')
    oak_conf = LaunchConfiguration('oak_conf')
    oak_iou = LaunchConfiguration('oak_iou')

    image_topic = LaunchConfiguration('image_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    matched_detection_topic = LaunchConfiguration('matched_detection_topic')

    camera_link_frame = LaunchConfiguration('camera_link_frame')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')
    publish_optical_tf = LaunchConfiguration('publish_optical_tf')

    oak = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oak_launch),
        launch_arguments={
            'params_file': oak_params_file,
            'enable_depth': 'true',
            'enable_yolo': 'false',
            'blob_path': oak_blob_path,
            'conf': oak_conf,
            'iou': oak_iou,
        }.items(),
        condition=IfCondition(start_oak),
    )

    qr = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='target_qr_matcher',
                executable='qr_matcher_node',
                name='target_matcher_node',
                output='screen',
                parameters=[{
                    'image_topic': image_topic,
                    'target_code': ParameterValue(target_code, value_type=str),
                    'target_code_topic': '/target_code',
                    'enable_topic': '/target_matcher/enable',
                    'reset_lock_topic': '/target_matcher/reset_lock',
                    'arrived_topic': '/target_matcher/arrived',
                    'matched_detection_topic': matched_detection_topic,
                    'state_topic': '/target_matcher/state',
                    'debug_image_topic': '/target_qr_matcher/debug_image',
                    'decode_scale': 1.0,
                    'publish_rate_hz': 5.0,
                    'lock_hold_sec': 1.2,
                    'publish_debug_image': True,
                    'min_qr_size_px': 20,
                    'match_digits_only': False,
                    'use_pyzbar_fallback': True,
                }],
                condition=IfCondition(start_qr),
            )
        ],
    )

    overlay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(overlay_launch),
        launch_arguments={
            'image_topic': image_topic,
            'detections_topic': matched_detection_topic,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
            'target_frame': target_frame,
            'publish_optical_tf': publish_optical_tf,
            'camera_link_frame': camera_link_frame,
            'camera_optical_frame': camera_optical_frame,
        }.items(),
        condition=IfCondition(start_overlay),
    )

    lift = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='qr_platform_lift_control',
                executable='qr_lift_control_node',
                name='qr_lift_control_node',
                output='screen',
                parameters=[pickup_control_config],
                condition=IfCondition(start_lift),
            )
        ],
    )

    chassis_align = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='qr_chassis_align_control',
                executable='qr_chassis_align_node',
                name='qr_chassis_align_node',
                output='screen',
                parameters=[pickup_control_config],
                condition=IfCondition(start_chassis_align),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_code', default_value='4309'),
        DeclareLaunchArgument('target_frame', default_value='oak_camera_link'),
        DeclareLaunchArgument('start_oak', default_value='true'),
        DeclareLaunchArgument('start_qr', default_value='true'),
        DeclareLaunchArgument('start_overlay', default_value='true'),
        DeclareLaunchArgument('start_lift', default_value='true'),
        DeclareLaunchArgument('start_chassis_align', default_value='false'),
        DeclareLaunchArgument('oak_params_file', default_value='/host-home/src/target_detection/oak_vision_driver/config/oak_yolo_runtime_ros_params.yaml'),
        DeclareLaunchArgument('oak_blob_path', default_value='/host-home/src/target_detection/oak_vision_driver/weights/best_fp16.blob'),
        DeclareLaunchArgument('oak_conf', default_value='0.14'),
        DeclareLaunchArgument('oak_iou', default_value='0.25'),
        DeclareLaunchArgument('image_topic', default_value='/oak/rgb/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/oak/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/oak/rgb/camera_info'),
        DeclareLaunchArgument('matched_detection_topic', default_value='/target_matcher/matched_detection'),
        DeclareLaunchArgument('camera_link_frame', default_value='oak_camera_link'),
        DeclareLaunchArgument('camera_optical_frame', default_value='oak_rgb_camera_optical_frame'),
        DeclareLaunchArgument('publish_optical_tf', default_value='true'),
        oak,
        qr,
        overlay,
        lift,
        chassis_align,
    ])
