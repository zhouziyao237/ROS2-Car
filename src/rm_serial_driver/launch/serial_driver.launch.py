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
    wheel_vx_scale_arg = DeclareLaunchArgument('wheel_vx_scale', default_value='1.0')
    wheel_vy_scale_arg = DeclareLaunchArgument('wheel_vy_scale', default_value='1.0')
    wheel_wz_scale_arg = DeclareLaunchArgument('wheel_wz_scale', default_value='1.0')
    cmd_vx_scale_arg = DeclareLaunchArgument('cmd_vx_scale', default_value='1.0')
    cmd_vy_scale_arg = DeclareLaunchArgument('cmd_vy_scale', default_value='1.0')
    cmd_wz_scale_arg = DeclareLaunchArgument('cmd_wz_scale', default_value='1.0')

    rm_serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config, {
            'wheel_vx_scale': LaunchConfiguration('wheel_vx_scale'),
            'wheel_vy_scale': LaunchConfiguration('wheel_vy_scale'),
            'wheel_wz_scale': LaunchConfiguration('wheel_wz_scale'),
            'cmd_vx_scale': LaunchConfiguration('cmd_vx_scale'),
            'cmd_vy_scale': LaunchConfiguration('cmd_vy_scale'),
            'cmd_wz_scale': LaunchConfiguration('cmd_wz_scale'),
        }],
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ('/red_standard_robot1/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
        ],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        wheel_vx_scale_arg,
        wheel_vy_scale_arg,
        wheel_wz_scale_arg,
        cmd_vx_scale_arg,
        cmd_vy_scale_arg,
        cmd_wz_scale_arg,
        rm_serial_driver_node,
    ])
