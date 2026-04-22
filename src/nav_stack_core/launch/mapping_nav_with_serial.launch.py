import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    serial_pkg = get_package_share_directory('rm_serial_driver')
    odom_pkg = get_package_share_directory('lidar_odom_pkg')
    nav_pkg = get_package_share_directory('nav_stack_core')
    lslidar_pkg = get_package_share_directory('lslidar_driver')

    serial_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(serial_pkg, 'launch', 'serial_driver.launch.py')
        ),
        launch_arguments={
            'cmd_vel_topic': '/cmd_vel_selected'
        }.items()
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lslidar_pkg, 'launch', 'lsm10p_uart_launch.py')
        )
    )

    lidar_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_pkg, 'launch', 'lidar_odom.launch.py')
        )
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'ekf_wheel_lidar_imu.launch.py')
        )
    )

    # 这里假设你已经有这个 launch，里面负责：
    # robot_state_publisher + slam_toolbox + nav2 + rviz
    nav_with_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation_system_with_lidar_odom.launch.py')
        )
    )

    cmd_vel_mux_node = Node(
        package='nav_stack_core',
        executable='cmd_vel_mode_mux.py',
        name='cmd_vel_mode_mux',
        output='screen',
        parameters=[{
            'manual_topic': '/cmd_vel_manual',
            'auto_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_selected',
            'mode_topic': '/control_mode',
            'publish_rate': 20.0,
            'cmd_timeout': 0.5,
            'default_mode': 'manual'
        }]
    )

    serial_imu_bridge_node = Node(
        package='nav_stack_core',
        executable='serial_imu_bridge.py',
        name='serial_imu_bridge',
        output='screen',
        parameters=[{
            'input_topic': '/serial/gimbal_joint_state',
            'output_topic': '/imu/data_serial',
            'imu_frame_id': 'imu_link',
            'assume_roll_zero': False
        }]
    )

    wheel_twist_odom_node = Node(
        package='nav_stack_core',
        executable='wheel_twist_odom.py',
        name='wheel_twist_odom',
        output='screen',
        parameters=[{
            'input_topic': '/wheel/twist',
            'odom_topic': '/wheel/odom',
            'odom_frame': 'wheel_odom',
            'base_frame': 'base_link',
            'publish_tf': True
        }]
    )

    return LaunchDescription([
        serial_driver_launch,
        lidar_launch,
        lidar_odom_launch,
        ekf_launch,
        serial_imu_bridge_node,
        wheel_twist_odom_node,
        cmd_vel_mux_node,
        nav_with_slam_launch,
    ])
