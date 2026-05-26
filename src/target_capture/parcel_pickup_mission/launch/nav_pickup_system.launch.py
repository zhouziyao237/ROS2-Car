from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def pkg_share(pkg):
    return get_package_share_directory(pkg)


def generate_launch_description():
    # SLAM + Nav2. If your nav launch filename is different, edit this path.
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share('nav_stack_core'), 'launch', 'mapping_nav_with_serial.launch.py')
        )
    )

    pickup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share('parcel_pickup_mission'), 'launch', 'pickup_system.launch.py')
        )
    )

    return LaunchDescription([
        nav_launch,
        TimerAction(period=5.0, actions=[pickup_launch]),
    ])
