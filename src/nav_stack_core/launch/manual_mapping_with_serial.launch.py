import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('nav_stack_core')
    lidar_odom_pkg = get_package_share_directory('lidar_odom_pkg')
    serial_pkg = get_package_share_directory('rm_serial_driver')

    serial_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(serial_pkg, 'launch', 'serial_driver.launch.py')
        ),
        launch_arguments={
            'cmd_vel_topic': '/cmd_vel'
        }.items()
    )

    lidar_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_odom_pkg, 'launch', 'lidar_odom.launch.py')
        )
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(
                os.path.join(nav_pkg, 'description', 'minimal_lidar_robot.urdf')
            ).read()
        }]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(nav_pkg, 'config', 'slam_toolbox_online.yaml')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        serial_driver_launch,
        lidar_odom_launch,
        robot_state_publisher_node,
        slam_toolbox_node,
        rviz_node,
    ])