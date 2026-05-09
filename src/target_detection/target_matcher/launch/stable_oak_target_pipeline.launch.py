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
    image_topic = LaunchConfiguration("image_topic")
    ocr_image_topic = LaunchConfiguration("ocr_image_topic")
    ocr_max_diff_ns = LaunchConfiguration("ocr_max_diff_ns")
    ocr_image_cache_size = LaunchConfiguration("ocr_image_cache_size")
    ocr_queue_size = LaunchConfiguration("ocr_queue_size")
    min_yolo_confidence = LaunchConfiguration("min_yolo_confidence")

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

        DeclareLaunchArgument(
            "image_topic",
            default_value="/oak/rgb/image_raw",
            description="Single shared image topic used by YOLO/OCR/matcher",
        ),

        DeclareLaunchArgument(
            "ocr_image_topic",
            default_value="/oak/rgb/image_raw",
            description="Image topic used by OCR cache. Keep same as image_topic for stability.",
        ),

        DeclareLaunchArgument(
            "ocr_max_diff_ns",
            default_value="1500000000",
            description="Max timestamp difference for OCR image sync, ns. 1.5s is stable for low FPS OAK.",
        ),

        DeclareLaunchArgument(
            "ocr_image_cache_size",
            default_value="60",
            description="Larger image cache avoids missing low FPS OAK frames.",
        ),

        DeclareLaunchArgument(
            "ocr_queue_size",
            default_value="8",
            description="OCR request queue size.",
        ),

        DeclareLaunchArgument(
            "min_yolo_confidence",
            default_value="0.14",
            description="Matcher-side YOLO confidence threshold.",
        ),

        # 1. OAK RGB + OAK NN YOLO
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oak_launch),
            launch_arguments={
                "config_path": oak_config_path,
            }.items(),
        ),

        # 2. OCR node. It caches /oak/rgb/image_raw and waits for ROI requests.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="ocr_node",
                    executable="ocr_node",
                    name="ocr_node",
                    output="screen",
                    parameters=[
                        "/host-home/src/ocr_node/config/ocr_en.yaml",
                        {
                            "image_topic": "/oak/rgb/image_raw",
                            "request_topic": "/vision/ocr_requests",
                            "result_topic": "/vision/ocr_results",
                            "max_image_time_diff_ms": 3000.0,
                            "ocr_queue_size": 8,
                            "rec_batch_size": 1,
                            "drop_score": 0.35,
                            "crop_padding_ratio": 0.08,
                        }
                    ],
                ),
            ],
        ),

        # 3. Target matcher. It receives OAK YOLO detections and sends OCR ROI requests.
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
                        "target_code": target_text,

                        "yolo_topic": "/yolov5/detections",
                        "ocr_request_topic": "/vision/ocr_requests",
                        "ocr_result_topic": "/vision/ocr_results",

                        "target_code_topic": "/target_code",
                        "enable_topic": "/target_matcher/enable",

                        "tracked_targets_topic": "/target_matcher/tracked_targets",
                        "matched_target_topic": "/target_matcher/matched_target",
                        "state_topic": "/target_matcher/state",

                        "tracking_enabled": True,
                "ocr_roi_expand_y": 1.12,
                "ocr_roi_expand_x": 1.35,
                "map_yolo_bbox_to_ocr_image": False,
                "ocr_image_height": 1080,
                "ocr_image_width": 1920,
                "yolo_image_height": 1080,
                "yolo_image_width": 1920,
                        "only_need_ocr": False,

                        # OAK preview 是 640x640，这两个必须加
                        "image_width": 1920,
                        "image_height": 1080,

                        # 先过滤低质量 YOLO 框，减少乱 OCR
                        "min_yolo_confidence": 0.25,

                        # OCR ROI 不要扩太大，避免把背景文字也裁进去
                        "ocr_roi_expand_scale": 1.20,

                        "require_target_code_for_ocr": True,
                        "avoid_repeat_ocr": True,

                        # 降低 OCR 请求频率，避免队列和同步混乱
                        "ocr_retry_interval_sec": 8.0,
                        "ocr_pending_timeout_sec": 15.0,
                        "max_ocr_requests_per_frame": 1,
                        "min_track_age_for_ocr": 8,

                        "recognition_mode": "lock_then_pose",

                        # 调试阶段先 1，看到 found target 框后再改回 2
                        "stable_match_required_count": 1,

                        "lock_lost_timeout_sec": 5.0,
                        "hold_calibration_after_arrived": True,
                    }],
                ),
            ],
        ),
    ])
