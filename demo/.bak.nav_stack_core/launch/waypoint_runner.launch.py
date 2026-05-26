from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('waypoints_file', default_value=PathJoinSubstitution([
            FindPackageShare('nav_stack_core'), 'config', 'waypoints_example.yaml'
        ])),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower_external',
            output='screen',
            parameters=[{'waypoints_file': LaunchConfiguration('waypoints_file')}]
        )
    ])
