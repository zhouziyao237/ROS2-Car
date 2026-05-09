from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("oak_vision_driver")

    default_params_file = os.path.join(
        pkg_share,
        "config",
        "oak_vision_driver.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="oak",
        ),
        DeclareLaunchArgument(
            "parser",
            default_value="NeuralNetwork",
        ),
        DeclareLaunchArgument(
            "enable_depth",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "blob_path",
            default_value="/host-home/src/target_detection/oak_vision_driver/weights/best.blob",
        ),
        DeclareLaunchArgument(
            "force_usb2",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="6.0",
        ),
        DeclareLaunchArgument(
            "conf",
            default_value="0.25",
        ),
        DeclareLaunchArgument(
            "iou",
            default_value="0.45",
        ),
        DeclareLaunchArgument(
            "decoder_debug",
            default_value="false",
        ),

        Node(
            package="oak_vision_driver",
            executable="oak_vision_driver_node",
            name="oak_vision_driver_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "enable_rgb": True,
                    "enable_depth": ParameterValue(
                        LaunchConfiguration("enable_depth"),
                        value_type=bool,
                    ),

                    "yolo_backend": LaunchConfiguration("mode"),
                    "enable_yolo": True,

                    "parser": LaunchConfiguration("parser"),
                    "blob_path": LaunchConfiguration("blob_path"),

                    "fps": ParameterValue(
                        LaunchConfiguration("fps"),
                        value_type=float,
                    ),
                    "conf": ParameterValue(
                        LaunchConfiguration("conf"),
                        value_type=float,
                    ),
                    "iou": ParameterValue(
                        LaunchConfiguration("iou"),
                        value_type=float,
                    ),

                    "force_usb2": ParameterValue(
                        LaunchConfiguration("force_usb2"),
                        value_type=bool,
                    ),
                    "decoder_debug": ParameterValue(
                        LaunchConfiguration("decoder_debug"),
                        value_type=bool,
                    ),

                    "queue_size": 1,
                    "timer_period": 0.01,
                    "stop_on_xlink_error": True,

                    "rgb_topic": "/oak/rgb/image_raw",
                    "depth_topic": "/oak/depth/image_raw",
                    "camera_info_topic": "/oak/rgb/camera_info",
                    "detections_topic": "/yolov5/detections",

                    "annotated_image_topic": "/oak/yolo_annotated_image",
                    "yolo_table_topic": "/oak/yolo_table",
                    "yolo_debug_topic": "/oak/yolo_debug_text",
                },
            ],
        ),
    ])