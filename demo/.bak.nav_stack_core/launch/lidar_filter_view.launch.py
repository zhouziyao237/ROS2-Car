import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('nav_stack_core')
    lslidar_pkg = get_package_share_directory('lslidar_driver')

    lidar_serial_port = LaunchConfiguration('lidar_serial_port')

    rviz_config = os.path.join(
        nav_pkg,
        'rviz',
        'lidar_filter_view.rviz'
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                lslidar_pkg,
                'launch',
                'lsm10p_uart_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': lidar_serial_port
        }.items()
    )

    front_scan_filter_node = Node(
        package='nav_stack_core',
        executable='front_scan_filter.py',
        name='front_scan_filter',
        output='screen',
        parameters=[{
            'input_topic': '/scan',
            'output_topic': '/scan_front',

            # 保留节点名不变，只改变过滤逻辑
            # 180~270 度不看
            'filter_mode': 'block_sector',
            'block_start_deg': 180.0,
            'block_end_deg': 275.0,

            'min_range': 0.18,
            'max_range': 200.0,
            'frame_id': '',
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_lidar_view',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device used by the M10P lidar driver'
        ),

        lidar_launch,
        front_scan_filter_node,
        rviz_node,
    ])