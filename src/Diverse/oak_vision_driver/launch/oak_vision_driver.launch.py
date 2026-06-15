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
        DeclareLaunchArgument("params_file", default_value=default_params_file),

        DeclareLaunchArgument("enable_rgb", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="false"),
        DeclareLaunchArgument("enable_yolo", default_value="false"),
        DeclareLaunchArgument("enable_stereo_fusion", default_value="false"),

        DeclareLaunchArgument("publish_left_image", default_value="true"),
        DeclareLaunchArgument("publish_right_image", default_value="true"),
        DeclareLaunchArgument("publish_stereo_fused_image", default_value="true"),
        DeclareLaunchArgument("publish_stereo_concat_image", default_value="false"),

        DeclareLaunchArgument("sensor_resolution", default_value="1080p"),
        DeclareLaunchArgument("mono_resolution", default_value="400p"),

        DeclareLaunchArgument("rgb_output", default_value="video"),
        DeclareLaunchArgument("preview_width", default_value="640"),
        DeclareLaunchArgument("preview_height", default_value="640"),
        DeclareLaunchArgument("video_width", default_value="1920"),
        DeclareLaunchArgument("video_height", default_value="1080"),
        DeclareLaunchArgument("input_width", default_value="640"),
        DeclareLaunchArgument("input_height", default_value="640"),

        DeclareLaunchArgument("fps", default_value="10.0"),
        DeclareLaunchArgument("force_usb2", default_value="false"),
        DeclareLaunchArgument("focus_mode", default_value="auto"),
        DeclareLaunchArgument("manual_focus", default_value="145"),
        DeclareLaunchArgument("autofocus_mode", default_value="continuous_video"),

        DeclareLaunchArgument("stereo_fusion_method", default_value="blend"),
        DeclareLaunchArgument("stereo_fusion_alpha", default_value="0.5"),

        DeclareLaunchArgument("parser", default_value="NeuralNetwork"),
        DeclareLaunchArgument(
            "blob_path",
            default_value="/host-home/src/target_detection/oak_vision_driver/weights/best_fp16_new.blob",
        ),
        DeclareLaunchArgument("conf", default_value="0.25"),
        DeclareLaunchArgument("iou", default_value="0.45"),

        Node(
            package="oak_vision_driver",
            executable="oak_vision_driver_node",
            name="oak_vision_driver_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "enable_rgb": ParameterValue(LaunchConfiguration("enable_rgb"), value_type=bool),
                    "enable_depth": ParameterValue(LaunchConfiguration("enable_depth"), value_type=bool),
                    "enable_yolo": ParameterValue(LaunchConfiguration("enable_yolo"), value_type=bool),
                    "enable_stereo_fusion": ParameterValue(
                        LaunchConfiguration("enable_stereo_fusion"),
                        value_type=bool,
                    ),

                    "publish_left_image": ParameterValue(
                        LaunchConfiguration("publish_left_image"),
                        value_type=bool,
                    ),
                    "publish_right_image": ParameterValue(
                        LaunchConfiguration("publish_right_image"),
                        value_type=bool,
                    ),
                    "publish_stereo_fused_image": ParameterValue(
                        LaunchConfiguration("publish_stereo_fused_image"),
                        value_type=bool,
                    ),
                    "publish_stereo_concat_image": ParameterValue(
                        LaunchConfiguration("publish_stereo_concat_image"),
                        value_type=bool,
                    ),

                    "sensor_resolution": LaunchConfiguration("sensor_resolution"),
                    "mono_resolution": LaunchConfiguration("mono_resolution"),

                    "rgb_output": LaunchConfiguration("rgb_output"),

                    "preview_width": ParameterValue(
                        LaunchConfiguration("preview_width"),
                        value_type=int,
                    ),
                    "preview_height": ParameterValue(
                        LaunchConfiguration("preview_height"),
                        value_type=int,
                    ),
                    "video_width": ParameterValue(
                        LaunchConfiguration("video_width"),
                        value_type=int,
                    ),
                    "video_height": ParameterValue(
                        LaunchConfiguration("video_height"),
                        value_type=int,
                    ),
                    "input_width": ParameterValue(
                        LaunchConfiguration("input_width"),
                        value_type=int,
                    ),
                    "input_height": ParameterValue(
                        LaunchConfiguration("input_height"),
                        value_type=int,
                    ),

                    "fps": ParameterValue(LaunchConfiguration("fps"), value_type=float),
                    "force_usb2": ParameterValue(
                        LaunchConfiguration("force_usb2"),
                        value_type=bool,
                    ),

                    "focus_mode": LaunchConfiguration("focus_mode"),
                    "manual_focus": ParameterValue(
                        LaunchConfiguration("manual_focus"),
                        value_type=int,
                    ),
                    "autofocus_mode": LaunchConfiguration("autofocus_mode"),

                    "stereo_fusion_method": LaunchConfiguration("stereo_fusion_method"),
                    "stereo_fusion_alpha": ParameterValue(
                        LaunchConfiguration("stereo_fusion_alpha"),
                        value_type=float,
                    ),

                    "parser": LaunchConfiguration("parser"),
                    "blob_path": LaunchConfiguration("blob_path"),
                    "conf": ParameterValue(LaunchConfiguration("conf"), value_type=float),
                    "iou": ParameterValue(LaunchConfiguration("iou"), value_type=float),
                },
            ],
        ),
    ])
