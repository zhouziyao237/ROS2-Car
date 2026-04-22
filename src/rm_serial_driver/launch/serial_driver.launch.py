import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_serial_driver'),
        'config',
        'serial_driver.yaml'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel_selected',
        description='Final selected cmd_vel topic subscribed by serial driver'
    )

    rm_serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ('/red_standard_robot1/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
        ],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        rm_serial_driver_node,
    ])