from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

import os


def str_to_bool(value: str) -> bool:
    return str(value).lower() in ["true", "1", "yes", "on"]


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).lower().strip()

    target_code = LaunchConfiguration("target_code").perform(context)

    enable_depth = LaunchConfiguration("enable_depth").perform(context)
    force_usb2 = LaunchConfiguration("force_usb2").perform(context)
    fps = LaunchConfiguration("fps").perform(context)
    conf = LaunchConfiguration("conf").perform(context)
    iou = LaunchConfiguration("iou").perform(context)
    parser = LaunchConfiguration("parser").perform(context)
    blob_path = LaunchConfiguration("blob_path").perform(context)

    use_target_matcher = str_to_bool(
        LaunchConfiguration("use_target_matcher").perform(context)
    )
    use_ocr = str_to_bool(
        LaunchConfiguration("use_ocr").perform(context)
    )
    use_depth_locator = str_to_bool(
        LaunchConfiguration("use_depth_locator").perform(context)
    )

    pkg_share = get_package_share_directory("oak_vision_driver")

    oak_params_file = LaunchConfiguration("oak_params_file").perform(context)
    if oak_params_file == "":
        oak_params_file = os.path.join(
            pkg_share,
            "config",
            "oak_vision_driver.yaml",
        )

    target_matcher_params_file = LaunchConfiguration(
        "target_matcher_params_file"
    ).perform(context)

    ocr_params_file = LaunchConfiguration("ocr_params_file").perform(context)

    image_topic = LaunchConfiguration("image_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    detections_topic = LaunchConfiguration("detections_topic").perform(context)

    matched_target_topic = LaunchConfiguration("matched_target_topic").perform(context)
    matched_point_topic = LaunchConfiguration("matched_point_topic").perform(context)
    matched_depth_text_topic = LaunchConfiguration(
        "matched_depth_text_topic"
    ).perform(context)

    actions = []

    # =========================
    # Decide YOLO backend
    # =========================
    if mode == "oak":
        yolo_backend = "oak"
        enable_yolo = True
    elif mode == "minipc":
        yolo_backend = "minipc"
        enable_yolo = False
    elif mode == "off":
        yolo_backend = "off"
        enable_yolo = False
    else:
        raise RuntimeError(
            f"Invalid mode={mode}. Use oak / minipc / off."
        )

    # =========================
    # OAK camera / OAK VPU node
    # =========================
    actions.append(
        Node(
            package="oak_vision_driver",
            executable="oak_vision_driver_node",
            name="oak_vision_driver_node",
            output="screen",
            parameters=[
                oak_params_file,
                {
                    "enable_rgb": True,
                    "enable_depth": ParameterValue(
                        LaunchConfiguration("enable_depth"),
                        value_type=bool,
                    ),

                    "yolo_backend": yolo_backend,
                    "enable_yolo": enable_yolo,

                    "parser": parser,

                    "rgb_topic": image_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                    "detections_topic": detections_topic,

                    "blob_path": blob_path,

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

                    "queue_size": 1,
                    "timer_period": 0.01,
                    "force_usb2": ParameterValue(
                        LaunchConfiguration("force_usb2"),
                        value_type=bool,
                    ),
                    "stop_on_xlink_error": True,

                    "annotated_image_topic": "/oak/yolo_annotated_image",
                    "yolo_table_topic": "/oak/yolo_table",
                    "yolo_debug_topic": "/oak/yolo_debug_text",
                },
            ],
        )
    )

    # =========================
    # mini PC YOLO fallback
    # =========================
    if mode == "minipc":
        yolov5_params_file = LaunchConfiguration("yolov5_params_file").perform(context)

        yolov5_params = {
            "input_mode": "topic",
            "image_topic": image_topic,
            "detections_topic": detections_topic,
            "conf_thres": conf,
            "iou_thres": iou,
            "num_threads": 1,
        }

        if yolov5_params_file != "":
            actions.append(
                Node(
                    package="yolov5_node",
                    executable="yolov5_node",
                    name="yolov5_node",
                    output="screen",
                    parameters=[
                        yolov5_params_file,
                        yolov5_params,
                    ],
                )
            )
        else:
            actions.append(
                Node(
                    package="yolov5_node",
                    executable="yolov5_node",
                    name="yolov5_node",
                    output="screen",
                    parameters=[yolov5_params],
                )
            )

    # =========================
    # target matcher
    # =========================
    if use_target_matcher:
        matcher_params = {
            "target_code": target_code,
            "detections_topic": detections_topic,
            "matched_target_topic": matched_target_topic,
            "state_topic": "/target_matcher/state",
            "ocr_request_topic": "/vision/ocr_requests",
            "ocr_result_topic": "/vision/ocr_results",
        }

        if target_matcher_params_file != "":
            actions.append(
                Node(
                    package="target_matcher",
                    executable="target_matcher_node",
                    name="target_matcher_node",
                    output="screen",
                    parameters=[
                        target_matcher_params_file,
                        matcher_params,
                    ],
                )
            )
        else:
            actions.append(
                Node(
                    package="target_matcher",
                    executable="target_matcher_node",
                    name="target_matcher_node",
                    output="screen",
                    parameters=[matcher_params],
                )
            )

    # =========================
    # OCR node
    # =========================
    if use_ocr:
        ocr_override_params = {
            "image_topic": image_topic,
            "request_topic": "/vision/ocr_requests",
            "result_topic": "/vision/ocr_results",
        }

        if ocr_params_file != "":
            actions.append(
                Node(
                    package="ocr_node",
                    executable="ocr_node",
                    name="ocr_node",
                    output="screen",
                    parameters=[
                        ocr_params_file,
                        ocr_override_params,
                    ],
                )
            )
        else:
            actions.append(
                Node(
                    package="ocr_node",
                    executable="ocr_node",
                    name="ocr_node",
                    output="screen",
                    parameters=[ocr_override_params],
                )
            )

    # =========================
    # Depth locator
    # matched bbox center + depth image => camera-frame XYZ
    # =========================
    if use_depth_locator:
        actions.append(
            Node(
                package="target_matcher",
                executable="matched_depth_locator_node",
                name="matched_depth_locator_node",
                output="screen",
                parameters=[{
                    "matched_target_topic": matched_target_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                    "point_topic": matched_point_topic,
                    "text_topic": matched_depth_text_topic,

                    "depth_window_radius": 4,
                    "min_depth_m": 0.15,
                    "max_depth_m": 5.0,
                    "mirror_x_for_depth": False,
                }],
            )
        )

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory("oak_vision_driver")

    default_oak_params = os.path.join(
        pkg_share,
        "config",
        "oak_vision_driver.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="oak",
            description="oak / minipc / off",
        ),
        DeclareLaunchArgument(
            "target_code",
            default_value="2-3-2050",
        ),

        DeclareLaunchArgument(
            "oak_params_file",
            default_value=default_oak_params,
        ),
        DeclareLaunchArgument(
            "target_matcher_params_file",
            default_value="",
        ),
        DeclareLaunchArgument(
            "ocr_params_file",
            default_value="/host-home/install/ocr_node/share/ocr_node/config/ocr_realtimes.yaml",
        ),
        DeclareLaunchArgument(
            "yolov5_params_file",
            default_value="",
        ),

        DeclareLaunchArgument(
            "image_topic",
            default_value="/oak/rgb/image_raw",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/oak/depth/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/oak/rgb/camera_info",
        ),
        DeclareLaunchArgument(
            "detections_topic",
            default_value="/yolov5/detections",
        ),

        DeclareLaunchArgument(
            "matched_target_topic",
            default_value="/target_matcher/matched_target",
        ),
        DeclareLaunchArgument(
            "matched_point_topic",
            default_value="/target_matcher/matched_point",
        ),
        DeclareLaunchArgument(
            "matched_depth_text_topic",
            default_value="/target_matcher/matched_depth_text",
        ),

        DeclareLaunchArgument(
            "use_target_matcher",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_ocr",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_depth_locator",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "enable_depth",
            default_value="true",
            description="Must be true if you want XYZ position from depth.",
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
            "parser",
            default_value="NeuralNetwork",
            description="NeuralNetwork or YoloDetectionNetwork",
        ),
        DeclareLaunchArgument(
            "blob_path",
            default_value="/host-home/src/target_detection/oak_vision_driver/weights/best_fp16_new.blob",
        ),

        OpaqueFunction(function=launch_setup),
    ])