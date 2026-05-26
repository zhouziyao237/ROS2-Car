#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('target_pickup_bringup')
    decision_config = os.path.join(bringup_dir, 'config', 'decision_tree.yaml')

    oak_dir = get_package_share_directory('oak_vision_driver')
    oak_launch = os.path.join(oak_dir, 'launch', 'oak_vision_driver.launch.py')

    ocr_dir = get_package_share_directory('ocr_node')
    ocr_config = os.path.join(ocr_dir, 'config', 'ocr_en.yaml')

    pose_bridge_dir = get_package_share_directory('target_pose_bridge')
    overlay_launch = os.path.join(
        pose_bridge_dir,
        'launch',
        'single_camera_target_overlay.launch.py'
    )

    start_oak = LaunchConfiguration('start_oak')
    start_ocr = LaunchConfiguration('start_ocr')
    start_matcher = LaunchConfiguration('start_matcher')
    start_overlay = LaunchConfiguration('start_overlay')
    start_decision = LaunchConfiguration('start_decision')

    target_code = LaunchConfiguration('target_code')
    auto_publish_target_code = LaunchConfiguration('auto_publish_target_code')

    oak_params_file = LaunchConfiguration('oak_params_file')
    oak_blob_path = LaunchConfiguration('oak_blob_path')
    oak_conf = LaunchConfiguration('oak_conf')
    oak_iou = LaunchConfiguration('oak_iou')

    image_topic = LaunchConfiguration('image_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    target_frame = LaunchConfiguration('target_frame')
    camera_link_frame = LaunchConfiguration('camera_link_frame')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')
    publish_optical_tf = LaunchConfiguration('publish_optical_tf')

    matcher_executable = LaunchConfiguration('matcher_executable')
    decision_executable = LaunchConfiguration('decision_executable')

    oak = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oak_launch),
        launch_arguments={
            'params_file': oak_params_file,
            'enable_depth': 'true',
            'blob_path': oak_blob_path,
            'conf': oak_conf,
            'iou': oak_iou,
        }.items(),
        condition=IfCondition(start_oak),
    )

    ocr = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ocr_node',
                executable='paddle_ocr_node',
                name='ocr_node',
                output='screen',
                parameters=[{
                    'image_topic': image_topic,
                    'request_topic': '/vision/ocr_requests',
                    'result_topic': '/vision/ocr_results',
                    'debug_image_topic': '/vision/ocr_debug_image',

                    'lang': 'en',
                    'use_angle_cls': False,
                    'use_gpu': False,
                    'show_log': False,

                    'image_cache_size': 80,
                    'max_image_time_diff_ms': 5000.0,
                    'ocr_queue_size': 4,

                    'min_roi_width': 8,
                    'min_roi_height': 8,
                    'roi_padding_ratio': 0.05,

                    'publish_debug_image': True,
                    'log_recognized_text': True,
                }],
                condition=IfCondition(start_ocr),
            )
        ],
    )

    matcher = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='target_matcher',
                executable=matcher_executable,
                name='target_matcher_node',
                output='screen',
                parameters=[{
                    'target_text': '',
                    'target_code': '',

                    'yolo_topic': '/yolov5/detections',
                    'ocr_request_topic': '/vision/ocr_requests',
                    'ocr_result_topic': '/vision/ocr_results',

                    'target_code_topic': '/target_code',
                    'enable_topic': '/target_matcher/enable',
                    'state_topic': '/target_matcher/state',
                    'arrived_topic': '/target_matcher/arrived',
                    'reset_lock_topic': '/target_matcher/reset_lock',
                    'recognition_mode_topic': '/target_matcher/recognition_mode',

                    'tracked_detections_topic': '/target_matcher/tracked_detections',
                    'tracked_targets_topic': '/target_matcher/tracked_targets',
                    'matched_detection_topic': '/target_matcher/matched_detection',
                    'matched_target_topic': '/target_matcher/matched_target',

                    'tracking_enabled': True,
                    'only_need_ocr': False,

                    'image_width': 640,
                    'image_height': 640,
                    'yolo_image_width': 640,
                    'yolo_image_height': 640,
                    'ocr_image_width': 640,
                    'ocr_image_height': 640,
                    'map_yolo_bbox_to_ocr_image': False,

                    'min_yolo_confidence': 0.03,

                    'min_bbox_width': 5.0,
                    'max_bbox_width': 640.0,
                    'min_bbox_height': 5.0,
                    'max_bbox_height': 640.0,
                    'min_bbox_aspect': 0.1,
                    'max_bbox_aspect': 20.0,

                    'ocr_roi_expand_scale': 2.2,
                    'ocr_roi_expand_x': 2.8,
                    'ocr_roi_expand_y': 2.0,

                    'require_target_code_for_ocr': True,
                    'avoid_repeat_ocr': False,

                    'ocr_retry_interval_sec': 1.0,
                    'ocr_pending_timeout_sec': 8.0,
                    'max_ocr_requests_per_frame': 3,
                    'min_track_age_for_ocr': 1,

                    'recognition_mode': 'lock_then_pose',
                    'stable_match_required_count': 2,
                    'lock_lost_timeout_sec': 20.0,
                    'hold_calibration_after_arrived': True,
                }],
                condition=IfCondition(start_matcher),
            )
        ],
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
        condition=IfCondition(start_overlay),
    )

    decision = Node(
        package='target_auto_decision',
        executable=decision_executable,
        name='target_auto_decision_node',
        output='screen',
        parameters=[decision_config],
        condition=IfCondition(start_decision),
    )

    enable_matcher = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/target_matcher/enable',
                    'std_msgs/msg/Bool',
                    '{data: true}'
                ],
                output='screen',
                condition=IfCondition(start_matcher),
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/target_matcher/recognition_mode',
                    'std_msgs/msg/String',
                    "{data: 'lock_then_pose'}"
                ],
                output='screen',
                condition=IfCondition(start_matcher),
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/target_matcher/arrived',
                    'std_msgs/msg/Bool',
                    '{data: true}'
                ],
                output='screen',
                condition=IfCondition(start_matcher),
            ),
        ],
    )

    publish_code = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/target_code',
                    'std_msgs/msg/String',
                    PythonExpression(["\"{data: '", target_code, "'}\""])
                ],
                output='screen',
                condition=IfCondition(auto_publish_target_code),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_oak', default_value='true'),
        DeclareLaunchArgument('start_ocr', default_value='true'),
        DeclareLaunchArgument('start_matcher', default_value='true'),
        DeclareLaunchArgument('start_overlay', default_value='true'),
        DeclareLaunchArgument('start_decision', default_value='true'),

        DeclareLaunchArgument('target_code', default_value='3412'),
        DeclareLaunchArgument('auto_publish_target_code', default_value='true'),

        DeclareLaunchArgument(
            'oak_params_file',
            default_value='/host-home/src/target_detection/oak_vision_driver/config/oak_yolo_runtime_ros_params.yaml'
        ),
        DeclareLaunchArgument(
            'oak_blob_path',
            default_value='/host-home/src/target_detection/oak_vision_driver/weights/best_fp16.blob'
        ),
        DeclareLaunchArgument('oak_conf', default_value='0.14'),
        DeclareLaunchArgument('oak_iou', default_value='0.25'),

        DeclareLaunchArgument('image_topic', default_value='/oak/rgb/image_raw'),
        DeclareLaunchArgument('detections_topic', default_value='/target_matcher/matched_detection'),
        DeclareLaunchArgument('depth_topic', default_value='/oak/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/oak/rgb/camera_info'),

        DeclareLaunchArgument('target_frame', default_value='oak_camera_link'),
        DeclareLaunchArgument('camera_link_frame', default_value='oak_camera_link'),
        DeclareLaunchArgument('camera_optical_frame', default_value='oak_rgb_camera_optical_frame'),
        DeclareLaunchArgument('publish_optical_tf', default_value='true'),

        DeclareLaunchArgument('matcher_executable', default_value='target_matcher_node'),
        DeclareLaunchArgument('decision_executable', default_value='auto_decision_node.py'),

        oak,
        ocr,
        matcher,
        overlay,
        decision,
        enable_matcher,
        publish_code,
    ])
