from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    astra_share = get_package_share_directory("astra_s_cam")
    yolo_share = get_package_share_directory("yolov5_node")
    ocr_share = get_package_share_directory("ocr_node")
    matcher_share = get_package_share_directory("target_matcher")

    astra_launch = os.path.join(
        astra_share,
        "launch",
        "astra.launch.py",
    )

    yolo_launch = os.path.join(
        yolo_share,
        "launch",
        "yolov5_onnx.launch.py",
    )

    ocr_launch = os.path.join(
        ocr_share,
        "launch",
        "ocr.launch.py",
    )

    matcher_config = os.path.join(
        matcher_share,
        "config",
        "target_matcher.yaml",
    )

    image_fix_config = os.path.join(
        matcher_share,
        "config",
        "image_fix.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_camera",
            default_value="false",
            description="Whether to launch Astra S camera",
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Whether to launch RViz from astra_s_cam",
        ),

        DeclareLaunchArgument(
            "use_locator",
            default_value="false",
            description="Whether astra_s_cam launches locator node",
        ),

        DeclareLaunchArgument(
            "use_image_flip",
            default_value="true",
            description="Whether to run RGB flip node",
        ),

        DeclareLaunchArgument(
            "raw_image_topic",
            default_value="/camera/rgb/image_raw",
            description="Raw RGB image topic from camera",
        ),

        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/rgb/image_unmirrored",
            description="Processed image topic used by YOLO",
        ),

        DeclareLaunchArgument(
            "ocr_image_topic",
            default_value="/camera/rgb/image_unmirrored",
            description="Image topic used by OCR. Must match YOLO image_topic when using flipped image.",
        ),

        DeclareLaunchArgument(
            "target_code",
            default_value="",
            description="Initial target code",
        ),

        DeclareLaunchArgument(
            "yolo_num_threads",
            default_value="1",
            description="YOLO ONNXRuntime thread number",
        ),

        DeclareLaunchArgument(
            "yolo_conf_thres",
            default_value="0.15",
            description="YOLO confidence threshold",
        ),

        DeclareLaunchArgument(
            "ocr_image_cache_size",
            default_value="30",
            description="OCR image cache size",
        ),

        DeclareLaunchArgument(
            "ocr_max_diff_ns",
            default_value="300000000",
            description="Max timestamp difference for OCR image sync, ns",
        ),

        DeclareLaunchArgument(
            "ocr_queue_size",
            default_value="8",
            description="OCR request queue size",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(astra_launch),
            condition=IfCondition(LaunchConfiguration("use_camera")),
            launch_arguments={
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_locator": LaunchConfiguration("use_locator"),
            }.items(),
        ),

        Node(
            package="image_fix",
            executable="rgb_flip_node",
            name="rgb_flip_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_image_flip")),
            parameters=[
                image_fix_config,
                {
                    "input_topic": LaunchConfiguration("raw_image_topic"),
                    "output_topic": LaunchConfiguration("image_topic"),
                },
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolo_launch),
            launch_arguments={
                "input_mode": "topic",
                "image_topic": LaunchConfiguration("image_topic"),
                "num_threads": LaunchConfiguration("yolo_num_threads"),
                "conf_thres": LaunchConfiguration("yolo_conf_thres"),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ocr_launch),
            launch_arguments={
                "input_mode": "topic",
                "image_topic": LaunchConfiguration("ocr_image_topic"),
                "image_cache_size": LaunchConfiguration("ocr_image_cache_size"),
                "max_diff_ns": LaunchConfiguration("ocr_max_diff_ns"),
                "ocr_queue_size": LaunchConfiguration("ocr_queue_size"),
            }.items(),
        ),

        Node(
            package="target_matcher",
            executable="target_matcher_node",
            name="target_matcher_node",
            output="screen",
            parameters=[
                matcher_config,
                {
                    "target_code": LaunchConfiguration("target_code"),
                },
            ],
        ),
    ])