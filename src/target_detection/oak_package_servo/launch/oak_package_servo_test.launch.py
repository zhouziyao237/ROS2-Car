from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    urdf_default = '/host-home/src/Diverse/robot2/urdf/robot2.urdf'
    config_default = '/host-home/src/target_detection/oak_package_servo/config/oak_package_servo.yaml'
    stage = LaunchConfiguration('stage')
    code = ParameterValue(LaunchConfiguration('package_code'), value_type=str)
    stage_value = ParameterValue(LaunchConfiguration('stage'), value_type=str)
    robot_description = ParameterValue(
        Command(['cat ', LaunchConfiguration('urdf_file')]),
        value_type=str
    )
    return LaunchDescription([
        DeclareLaunchArgument('urdf_file', default_value=urdf_default),
        DeclareLaunchArgument('config', default_value=config_default),
        DeclareLaunchArgument('stage', default_value='approach'),
        DeclareLaunchArgument('package_code', default_value='0301'),
        DeclareLaunchArgument('camera_x_mm', default_value='-120.0'),
        DeclareLaunchArgument('camera_y_mm', default_value='0.0'),
        DeclareLaunchArgument('camera_z_mm', default_value='900.0'),
        DeclareLaunchArgument('shelf_front_yaw', default_value='0.0'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_oak_servo_test',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher_oak_servo_test',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='oak_package_servo',
            executable='oak_package_servo_node',
            name='oak_package_servo_node',
            output='screen',
            parameters=[LaunchConfiguration('config'), {'enable_scan_guard': False, 'enable_map_guard': False}],
        ),
        Node(
            package='oak_package_servo',
            executable='fake_oak_package_target',
            name='fake_oak_package_target',
            output='screen',
            parameters=[{
                'stage': stage_value,
                'package_code': code,
                'camera_x_mm': LaunchConfiguration('camera_x_mm'),
                'camera_y_mm': LaunchConfiguration('camera_y_mm'),
                'camera_z_mm': LaunchConfiguration('camera_z_mm'),
                'shelf_front_yaw': LaunchConfiguration('shelf_front_yaw'),
            }],
        ),
        Node(
            package='oak_package_servo',
            executable='oak_servo_error_watch',
            name='oak_servo_error_watch',
            output='screen',
        ),
    ])
