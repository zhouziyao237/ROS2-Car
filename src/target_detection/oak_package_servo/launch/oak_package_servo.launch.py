from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('oak_package_servo')
    default_config = os.path.join(pkg_share, 'config', 'oak_package_servo.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='oak_package_servo',
            executable='oak_package_servo_node',
            name='oak_package_servo_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
