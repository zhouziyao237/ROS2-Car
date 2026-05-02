from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("astra_s_cam")
    config_file = os.path.join(pkg_dir, "config", "astra_s.yaml")
    rviz_config = os.path.join(pkg_dir, "rviz", "astra.rviz")

    openni2_lib_default = os.environ.get("OPENNI2_REDIST", "/host-home/Software/OpenNi2/sdk/libs")
    openni2_drivers_default = os.environ.get("OPENNI2_DRIVERS_PATH", "/host-home/Software/OpenNi2/sdk/libs/OpenNI2/Drivers")

    use_rviz = LaunchConfiguration("use_rviz")
    use_locator = LaunchConfiguration("use_locator")
    enable_rgb = LaunchConfiguration("enable_rgb")
    enable_depth = LaunchConfiguration("enable_depth")

    openni2_lib = LaunchConfiguration("openni2_lib")
    openni2_drivers = LaunchConfiguration("openni2_drivers")

    ld_library_path = [
        openni2_lib,
        ":",
        openni2_drivers,
        ":",
        os.environ.get("LD_LIBRARY_PATH", ""),
    ]

    driver_node = Node(
        package="astra_s_cam",
        executable="astra_s_driver_node",
        name="astra_s_driver_node",
        output="screen",
        parameters=[
            config_file,
            {
                "enable_rgb": enable_rgb,
                "enable_depth": enable_depth,
            },
        ],
    )

    # Static TFs matching the old openni2/astra_camera style.
    tf_camera_depth = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "-0.020", "0", "0", "0", "0", "camera_link", "camera_depth_frame"],
        output="screen",
    )

    tf_camera_rgb = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "-0.045", "0", "0", "0", "0", "camera_link", "camera_rgb_frame"],
        output="screen",
    )

    tf_depth_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "-0.5", "0.5", "-0.5", "0.5", "camera_depth_frame", "camera_depth_optical_frame"],
        output="screen",
    )

    tf_rgb_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "-0.5", "0.5", "-0.5", "0.5", "camera_rgb_frame", "camera_rgb_optical_frame"],
        output="screen",
    )

    locator_node = Node(
        package="astra_s_cam",
        executable="rgbd_locator_node.py",
        name="rgbd_locator_node",
        output="screen",
        parameters=[config_file],
        condition=IfCondition(use_locator),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_locator", default_value="true"),
        DeclareLaunchArgument("enable_rgb", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("openni2_lib", default_value=openni2_lib_default),
        DeclareLaunchArgument("openni2_drivers", default_value=openni2_drivers_default),

        SetEnvironmentVariable("OPENNI2_REDIST", openni2_lib),
        SetEnvironmentVariable("OPENNI2_DRIVERS_PATH", openni2_drivers),
        SetEnvironmentVariable("LD_LIBRARY_PATH", ld_library_path),
        SetEnvironmentVariable("LD_PRELOAD", [openni2_lib, "/libOpenNI2.so"]),

        driver_node,
        tf_camera_depth,
        tf_camera_rgb,
        tf_depth_optical,
        tf_rgb_optical,
        locator_node,
        rviz_node,
    ])
