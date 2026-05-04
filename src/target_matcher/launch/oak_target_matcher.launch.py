import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    target_text = LaunchConfiguration("target_text")
    oak_config_path = LaunchConfiguration("oak_config_path")

    oak_launch = os.path.join(
        get_package_share_directory("oak_vision_driver"),
        "launch",
        "oak_yolo_depth.launch.py",
    )

    default_oak_config = "/host-home/src/oak_vision_driver/config/yolov5_oak.yaml"

    return LaunchDescription([
        DeclareLaunchArgument(
            "target_text",
            default_value="4309",
            description="Target package label text, for example 4309",
        ),

        DeclareLaunchArgument(
            "oak_config_path",
            default_value=default_oak_config,
            description="OAK YOLO runtime YAML config path",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oak_launch),
            launch_arguments={
                "config_path": oak_config_path,
            }.items(),
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="ocr_node",
                    executable="ocr_node",
                    name="ocr_node",
                    output="screen",
                    parameters=[{
                        "input_mode": "topic",
                        "image_topic": "/oak/rgb/image_raw",
                        "request_topic": "/vision/ocr_requests",
                        "result_topic": "/vision/ocr_results",
                    }],
                ),
            ],
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="target_matcher",
                    executable="target_matcher_node",
                    name="target_matcher_node",
                    output="screen",
                    parameters=[{
                        "target_text": target_text,
                        "yolo_topic": "/yolov5/detections",
                        "ocr_request_topic": "/vision/ocr_requests",
                        "ocr_result_topic": "/vision/ocr_results",
                        "target_code_topic": "/target_code",
                        "enable_topic": "/target_matcher/enable",
                        "tracked_targets_topic": "/target_matcher/tracked_targets",
                        "matched_target_topic": "/target_matcher/matched_target",
                        "state_topic": "/target_matcher/state",

                        "tracking_enabled": True,
                        "min_yolo_confidence": 0.10,

                        "require_target_code_for_ocr": True,
                        "avoid_repeat_ocr": True,
                        "ocr_retry_interval_sec": 2.0,
                        "ocr_pending_timeout_sec": 8.0,
                        "max_ocr_requests_per_frame": 1,
                        "min_track_age_for_ocr": 2,

                        "recognition_mode": "lock_then_pose",
                        "stable_match_required_count": 2,
                    }],
                ),
            ],
        ),
    ])
