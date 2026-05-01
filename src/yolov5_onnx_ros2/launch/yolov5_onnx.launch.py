from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("yolov5_onnx_ros2")
    weights_file = os.path.join(pkg_share, "weights", "best_int8.onnx")
    result_file = os.path.join(pkg_share, "test", "result.jpg")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=weights_file,
            description="Path to YOLOv5 ONNX / INT8 ONNX model",
        ),

        DeclareLaunchArgument(
            "input_mode",
            default_value="topic",
            description="Input mode: topic, image, or auto",
        ),

        DeclareLaunchArgument(
            "image_topic",
            default_value="/image_raw",
            description="Input image topic when input_mode=topic",
        ),

        DeclareLaunchArgument(
            "image_path",
            default_value="",
            description="Input image path when input_mode=image or auto",
        ),

        DeclareLaunchArgument(
            "save_path",
            default_value= result_file,
            description="Path to save annotated result image in image mode",
        ),

        DeclareLaunchArgument(
            "exit_after_image",
            default_value="true",
            description="Exit node after single image detection",
        ),

        DeclareLaunchArgument(
            "detections_topic",
            default_value="/yolov5/detections",
            description="Output detection topic",
        ),

        DeclareLaunchArgument(
            "annotated_topic",
            default_value="/yolov5/annotated_image",
            description="Output annotated image topic",
        ),

        DeclareLaunchArgument(
            "img_size",
            default_value="640",
            description="YOLOv5 input image size",
        ),

        DeclareLaunchArgument(
            "conf_thres",
            default_value="0.25",
            description="Confidence threshold",
        ),

        DeclareLaunchArgument(
            "iou_thres",
            default_value="0.45",
            description="NMS IoU threshold",
        ),

        DeclareLaunchArgument(
            "num_threads",
            default_value="1",
            description="ONNX Runtime CPU thread number",
        ),

        Node(
            package="yolov5_onnx_ros2",
            executable="yolov5_onnx_node",
            name="yolov5_onnx_node",
            output="screen",
            parameters=[
                {
                    "model_path": LaunchConfiguration("model_path"),
                    "input_mode": LaunchConfiguration("input_mode"),

                    "image_topic": LaunchConfiguration("image_topic"),
                    "image_path": LaunchConfiguration("image_path"),
                    "save_path": LaunchConfiguration("save_path"),
                    "exit_after_image": LaunchConfiguration("exit_after_image"),

                    "detections_topic": LaunchConfiguration("detections_topic"),
                    "annotated_topic": LaunchConfiguration("annotated_topic"),

                    "img_size": LaunchConfiguration("img_size"),
                    "conf_thres": LaunchConfiguration("conf_thres"),
                    "iou_thres": LaunchConfiguration("iou_thres"),
                    "num_threads": LaunchConfiguration("num_threads"),
                }
            ],
        ),
    ])