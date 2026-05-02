import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_serial_driver_arg = DeclareLaunchArgument(
        "start_serial_driver",
        default_value="true",
        description="Whether to launch rm_serial_driver together.",
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/red_standard_robot1/cmd_vel",
        description="Target cmd_vel topic for rm_serial_driver.",
    )
    speed_arg = DeclareLaunchArgument(
        "speed",
        default_value="0.5",
        description="Linear speed scale for keyboard teleop.",
    )
    turn_arg = DeclareLaunchArgument(
        "turn",
        default_value="1.0",
        description="Angular speed scale for keyboard teleop.",
    )

    serial_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rm_serial_driver"),
                "launch",
                "serial_driver.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_serial_driver")),
    )

    keyboard_teleop_node = Node(
        package="rm_serial_driver",
        executable="keyboard_teleop_node.py",
        name="keyboard_teleop",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "speed": LaunchConfiguration("speed"),
                "turn": LaunchConfiguration("turn"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            start_serial_driver_arg,
            cmd_vel_topic_arg,
            speed_arg,
            turn_arg,
            serial_driver_launch,
            keyboard_teleop_node,
        ]
    )
