from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("astra_s_cam")
    config_file = os.path.join(pkg_dir, "config", "astra_s.yaml")

    openni2_lib_default = os.environ.get("OPENNI2_REDIST", "/host-home/Software/OpenNi2/sdk/libs")
    openni2_drivers_default = os.environ.get("OPENNI2_DRIVERS_PATH", "/host-home/Software/OpenNi2/sdk/libs/OpenNI2/Drivers")

    mode = LaunchConfiguration("mode")
    board_size = LaunchConfiguration("board_size")
    square = LaunchConfiguration("square")
    openni2_lib = LaunchConfiguration("openni2_lib")
    openni2_drivers = LaunchConfiguration("openni2_drivers")

    is_rgb = PythonExpression(["'", mode, "' == 'rgb' or '", mode, "' == 'all'"])
    is_depth = PythonExpression(["'", mode, "' == 'depth' or '", mode, "' == 'all'"])

    driver_node = Node(
        package="astra_s_cam",
        executable="astra_s_driver_node",
        name="astra_s_driver_node",
        output="screen",
        parameters=[config_file],
    )

    rgb_calib = ExecuteProcess(
        cmd=[
            "ros2", "run", "camera_calibration", "cameracalibrator",
            "--size", board_size,
            "--square", square,
            "--no-service-check",
            "image:=/camera/rgb/image_raw",
            "camera:=/camera/rgb",
        ],
        output="screen",
        condition=IfCondition(is_rgb),
    )

    depth_calib = Node(
        package="astra_s_cam",
        executable="depth_calib_node.py",
        name="depth_calib_node",
        output="screen",
        parameters=[config_file],
        condition=IfCondition(is_depth),
    )

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="depth", description="rgb, depth, or all"),
        DeclareLaunchArgument("board_size", default_value="8x11"),
        DeclareLaunchArgument("square", default_value="0.020"),
        DeclareLaunchArgument("openni2_lib", default_value=openni2_lib_default),
        DeclareLaunchArgument("openni2_drivers", default_value=openni2_drivers_default),

        SetEnvironmentVariable("OPENNI2_REDIST", openni2_lib),
        SetEnvironmentVariable("OPENNI2_DRIVERS_PATH", openni2_drivers),
        SetEnvironmentVariable("LD_LIBRARY_PATH", [openni2_lib, ":", openni2_drivers, ":", os.environ.get("LD_LIBRARY_PATH", "")]),
        SetEnvironmentVariable("LD_PRELOAD", [openni2_lib, "/libOpenNI2.so"]),

        driver_node,
        rgb_calib,
        depth_calib,
    ])
