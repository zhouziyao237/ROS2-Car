from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os
import yaml


# Load the package YAML once so launch arguments, driver parameters, and node
# parameters all come from the same source of truth.
def load_yaml_file(yaml_file):
    if not os.path.exists(yaml_file):
        return {}
    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f)
    return data or {}


# Read nested keys like "calibration.rgb_info_uri" from the YAML dictionary.
def read_yaml_value(data, key, default_value=None):
    cur = data
    for k in key.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return default_value
        cur = cur[k]
    return cur


# Extract the ROS parameters block for a named node from the shared YAML.
def load_node_params(data, node_name):
    node_data = data.get(node_name, {})
    return node_data.get("ros__parameters", {})


# OpenNI2 libraries are required at launch time, so fail early with a clear
# message instead of silently falling back to a broken configuration.
def read_required_path(env_name, data, key, config_file):
    value = os.environ.get(env_name) or read_yaml_value(data, key)
    if value:
        return value
    raise RuntimeError(
        f"Missing required OpenNI2 path: set {env_name} or define '{key}' in {config_file}"
    )


# Support either:
# 1. an explicit file:// URI,
# 2. an absolute filesystem path, or
# 3. a path relative to the installed package share directory.
def resolve_camera_info_url(pkg_dir, value):
    if value is None:
        return ""

    value = str(value).strip()
    if not value:
        return ""

    if "://" in value:
        return value

    value = os.path.expanduser(value)
    if os.path.isabs(value):
        return f"file://{value}"

    return f"file://{os.path.join(pkg_dir, value)}"


def generate_launch_description():
    # Resolve package resources from the installed share directory. This keeps
    # runtime behavior consistent whether the workspace is run from source or
    # from an installed layout.
    pkg_dir = get_package_share_directory("ros_astra_s_camera")
    config_file = os.path.join(pkg_dir, "config", "astra_s_camera.yaml")
    config_data = load_yaml_file(config_file)
    rviz_config = os.path.join(pkg_dir, "rviz", "astra_s_camera.rviz")
    rviz_lite_config = os.path.join(pkg_dir, "rviz", "astra_s_camera_lite.rviz")

    openni2_lib = read_required_path("OPENNI2_REDIST", config_data, "openni2.redist", config_file)
    openni2_drivers = read_required_path(
        "OPENNI2_DRIVERS_PATH", config_data, "openni2.drivers", config_file
    )
    rgb_camera_info_url = resolve_camera_info_url(
        pkg_dir, read_yaml_value(config_data, "calibration.rgb_info_uri", "")
    )
    depth_camera_info_url = resolve_camera_info_url(
        pkg_dir, read_yaml_value(config_data, "calibration.depth_info_uri", "")
    )

    # Keep namespace/tf_prefix as launch arguments so the camera can be reused
    # in larger systems without editing the script.
    namespace_param_name = "namespace"
    namespace = LaunchConfiguration(namespace_param_name)
    namespace_launch_arg = DeclareLaunchArgument(namespace_param_name, default_value="camera")

    tf_prefix_param_name = "tf_prefix"
    tf_prefix = LaunchConfiguration(tf_prefix_param_name)
    tf_prefix_launch_arg = DeclareLaunchArgument(tf_prefix_param_name, default_value="")

    # OpenNI2 requires both the redistributable libs and the vendor driver path.
    ld_library_parts = [openni2_lib, openni2_drivers]
    existing_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    if existing_ld_library_path:
        ld_library_parts.append(existing_ld_library_path)
    ld_library_path = ":".join(ld_library_parts)

    use_cloud = LaunchConfiguration("use_cloud")
    use_rviz = LaunchConfiguration("use_rviz")
    lite_mode = LaunchConfiguration("lite_mode")

    # These parameters are passed directly into openni2_camera.
    driver_params = [
        {"depth_registration": True},
        {"use_device_time": True},
        {"rgb_frame_id": [namespace, "_rgb_optical_frame"]},
        {"depth_frame_id": [namespace, "_depth_optical_frame"]},
        {"ir_frame_id": [namespace, "_ir_optical_frame"]},
    ]
    if rgb_camera_info_url:
        driver_params.append({"rgb_camera_info_url": rgb_camera_info_url})
    if depth_camera_info_url:
        driver_params.append({"depth_camera_info_url": depth_camera_info_url})

    # Normal camera mode: publish RGB + depth without the point cloud pipeline.
    camera_only_driver = ComposableNodeContainer(
        name="container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="openni2_camera",
                plugin="openni2_wrapper::OpenNI2Driver",
                name="driver",
                namespace=namespace,
                parameters=driver_params,
            ),
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", use_cloud, "' != 'true' or '", lite_mode, "' == 'true'"])
        )
    )

    # Cloud mode: add depth_image_proc so RViz or downstream nodes can consume
    # an XYZRGB point cloud from the registered depth image.
    camera_cloud_driver = ComposableNodeContainer(
        name="container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="openni2_camera",
                plugin="openni2_wrapper::OpenNI2Driver",
                name="driver",
                namespace=namespace,
                parameters=driver_params,
            ),
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="points_xyzrgb",
                namespace=namespace,
                parameters=[{"queue_size": 3}],
                remappings=[
                    ("rgb/image_rect_color", "rgb/image_raw"),
                    ("rgb/camera_info", "rgb/camera_info"),
                    ("depth_registered/image_rect", "depth/image"),
                    ("points", "depth/points"),
                ],
            ),
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", use_cloud, "' == 'true' and '", lite_mode, "' != 'true'"])
        )
    )

    # Reuse the standard OpenNI2 static TF tree so frames stay compatible with
    # existing OpenNI-based camera tooling.
    tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("openni2_camera"),
                "launch",
                "tfs.launch.py"
            ])
        ),
        launch_arguments={namespace_param_name: namespace, tf_prefix_param_name: tf_prefix}.items(),
    )

    # Custom RGB-D fusion node parameters come from the shared YAML.
    rgbd_params = load_node_params(config_data, "rgbd_position_node")
    rgbd_lite_params = {
        "sync_queue_size": 2,
        "sync_slop_sec": 0.03,
        "target_sync_slop_sec": 0.05,
        "target_history_size": 10,
        "publish_debug_image": False,
        "use_center_when_no_target": False,
    }

    rgbd_position_node = Node(
        package="ros_astra_s_camera",
        executable="rgbd_position_node.py",
        name="rgbd_position_node",
        output="screen",
        parameters=[rgbd_params],
        condition=UnlessCondition(lite_mode)
    )

    rgbd_position_node_lite = Node(
        package="ros_astra_s_camera",
        executable="rgbd_position_node.py",
        name="rgbd_position_node",
        output="screen",
        parameters=[rgbd_params, rgbd_lite_params],
        condition=IfCondition(lite_mode)
    )

    # RViz is optional because many deployments run headless over SSH.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(
            PythonExpression(["'", use_rviz, "' == 'true' and '", lite_mode, "' != 'true'"])
        )
    )

    rviz_node_lite = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_lite_config],
        condition=IfCondition(
            PythonExpression(["'", use_rviz, "' == 'true' and '", lite_mode, "' == 'true'"])
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_cloud", default_value="false", description="Use openni2 camera_with_cloud launch."),
        DeclareLaunchArgument("use_rviz", default_value="false", description="Open RViz2 with Astra S config."),
        DeclareLaunchArgument(
            "lite_mode",
            default_value="false",
            description="Use low-resource RGB-D parameters and the lightweight RViz profile."
        ),
        namespace_launch_arg,
        tf_prefix_launch_arg,

        SetEnvironmentVariable("OPENNI2_REDIST", openni2_lib),
        SetEnvironmentVariable("OPENNI2_DRIVERS_PATH", openni2_drivers),
        SetEnvironmentVariable("LD_LIBRARY_PATH", ld_library_path),
        SetEnvironmentVariable("LD_PRELOAD", os.path.join(openni2_lib, "libOpenNI2.so")),

        camera_only_driver,
        camera_cloud_driver,
        tfs,
        rgbd_position_node,
        rgbd_position_node_lite,
        rviz_node,
        rviz_node_lite,
    ])
