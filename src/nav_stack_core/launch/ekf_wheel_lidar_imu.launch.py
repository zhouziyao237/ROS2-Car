import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_stack_core')
    default_params = os.path.join(pkg_share, 'config', 'ekf_wheel_lidar_imu.yaml')
    params_file = LaunchConfiguration('params_file')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[params_file],
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to EKF parameter file'
        ),
        ekf_node,
    ])
