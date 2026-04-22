import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_pkg = get_package_share_directory('nav_stack_core')
    lidar_odom_pkg = get_package_share_directory('lidar_odom_pkg')
    serial_pkg = get_package_share_directory('rm_serial_driver')

    serial_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(serial_pkg, 'launch', 'serial_driver.launch.py')
        )
    )

    lidar_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_odom_pkg, 'launch', 'lidar_odom.launch.py')
        )
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'ekf_wheel_lidar_imu.launch.py')
        )
    )

    nav_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation_system_with_lidar_odom.launch.py')
        )
    )

    return LaunchDescription([
        serial_driver_launch,
        lidar_odom_launch,
        ekf_launch,
        nav_system_launch,
    ])
