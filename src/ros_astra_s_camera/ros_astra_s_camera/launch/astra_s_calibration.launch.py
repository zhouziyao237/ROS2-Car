from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os
import yaml


# Load the shared YAML config once and reuse it for all launch decisions.
def load_yaml_file(yaml_file):
    if not os.path.exists(yaml_file):
        return {}
    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f)
    return data or {}


# Read nested keys like "calibration.board_size" from the YAML dictionary.
def read_yaml_value(data, key, default_value=None):
    cur = data
    for k in key.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return default_value
        cur = cur[k]
    return cur


# Extract one node's ros__parameters block from the shared YAML file.
def load_node_params(data, node_name):
    node_data = data.get(node_name, {})
    return node_data.get("ros__parameters", {})


# OpenNI2 library paths are required; fail fast with a clear message if they
# were not configured in the environment or YAML.
def read_required_path(env_name, data, key, config_file):
    value = os.environ.get(env_name) or read_yaml_value(data, key)
    if value:
        return value
    raise RuntimeError(
        f"Missing required OpenNI2 path: set {env_name} or define '{key}' in {config_file}"
    )


# Support explicit URIs, absolute paths, and package-relative calibration files.
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
    # Use the installed package share directory so runtime assets are found
    # after `colcon build` and `source install/setup.bash`.
    pkg_dir = get_package_share_directory("ros_astra_s_camera")
    config_file = os.path.join(pkg_dir, "config", "astra_s_camera.yaml")
    config_data = load_yaml_file(config_file)
    rviz_config = os.path.join(pkg_dir, "rviz", "astra_s_camera.rviz")
    rviz_lite_config = os.path.join(pkg_dir, "rviz", "astra_s_camera_lite.rviz")

    openni2_lib = read_required_path("OPENNI2_REDIST", config_data, "openni2.redist", config_file)
    openni2_drivers = read_required_path(
        "OPENNI2_DRIVERS_PATH", config_data, "openni2.drivers", config_file
    )
    board_size = str(read_yaml_value(config_data, "calibration.board_size", "8x11"))
    square_size = str(read_yaml_value(config_data, "calibration.square_size_m", "0.030"))
    rgb_camera_info_url = resolve_camera_info_url(
        pkg_dir, read_yaml_value(config_data, "calibration.rgb_info_uri", "")
    )
    depth_camera_info_url = resolve_camera_info_url(
        pkg_dir, read_yaml_value(config_data, "calibration.depth_info_uri", "")
    )

    # Keep the calibration launch reusable in other namespaces if needed.
    namespace_param_name = "namespace"
    namespace = LaunchConfiguration(namespace_param_name)
    namespace_launch_arg = DeclareLaunchArgument(namespace_param_name, default_value="camera")

    tf_prefix_param_name = "tf_prefix"
    tf_prefix = LaunchConfiguration(tf_prefix_param_name)
    tf_prefix_launch_arg = DeclareLaunchArgument(tf_prefix_param_name, default_value="")
    use_rviz = LaunchConfiguration("use_rviz")
    lite_mode = LaunchConfiguration("lite_mode")
    calibration_start_delay_sec = LaunchConfiguration("calibration_start_delay_sec")

    # OpenNI2 needs both the redistributable libs and the vendor driver path.
    ld_library_parts = [openni2_lib, openni2_drivers]
    existing_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    if existing_ld_library_path:
        ld_library_parts.append(existing_ld_library_path)
    ld_library_path = ":".join(ld_library_parts)

    # Feed calibration file URIs straight into openni2_camera so it does not
    # fall back to the default ~/.ros/camera_info search path.
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

    # Calibration mode still uses the same driver as runtime mode; only the
    # extra tools below are different.
    camera_driver = ComposableNodeContainer(
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
        output="screen"
    )

    # Publish the standard camera frame tree for RViz and calibration tools.
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

    # Start the ROS camera_calibration GUI against the RGB topic.
    rgb_calibration = ExecuteProcess(
        cmd=[
            "ros2", "run", "camera_calibration", "cameracalibrator",
            "--size", board_size,
            "--square", square_size,
            "--ros-args",
            "-r", ["image:=/", namespace, "/rgb/image_raw"],
            "-r", ["camera:=/", namespace, "/rgb"]
        ],
        output="screen"
    )

    delayed_rgb_calibration = TimerAction(
        period=calibration_start_delay_sec,
        actions=[rgb_calibration]
    )

    # The depth helper node reports the center-region median depth so a simple
    # scale/offset correction can be computed with a ruler.
    depth_params = load_node_params(config_data, "depth_calibration_node")
    depth_lite_params = {
        "sample_radius": 4,
    }

    depth_calibration_node = Node(
        package="ros_astra_s_camera",
        executable="depth_calibration_node.py",
        name="depth_calibration_node",
        output="screen",
        parameters=[depth_params],
        condition=IfCondition(PythonExpression(["'", lite_mode, "' != 'true'"]))
    )

    depth_calibration_node_lite = Node(
        package="ros_astra_s_camera",
        executable="depth_calibration_node.py",
        name="depth_calibration_node",
        output="screen",
        parameters=[depth_params, depth_lite_params],
        condition=IfCondition(PythonExpression(["'", lite_mode, "' == 'true'"]))
    )

    # RViz is optional so calibration can still run on a headless machine.
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
        DeclareLaunchArgument("use_rviz", default_value="false", description="Open RViz2 with Astra S config."),
        DeclareLaunchArgument(
            "lite_mode",
            default_value="false",
            description="Use the lightweight RViz profile and smaller depth sampling window."
        ),
        DeclareLaunchArgument(
            "calibration_start_delay_sec",
            default_value="2.0",
            description="Delay starting camera_calibration until the RGB stream is available."
        ),
        namespace_launch_arg,
        tf_prefix_launch_arg,
        SetEnvironmentVariable("OPENNI2_REDIST", openni2_lib),
        SetEnvironmentVariable("OPENNI2_DRIVERS_PATH", openni2_drivers),
        SetEnvironmentVariable("LD_LIBRARY_PATH", ld_library_path),
        SetEnvironmentVariable("LD_PRELOAD", os.path.join(openni2_lib, "libOpenNI2.so")),

        camera_driver,
        tfs,
        delayed_rgb_calibration,
        depth_calibration_node,
        depth_calibration_node_lite,
        rviz_node,
        rviz_node_lite,
    ])
