from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('oak_package_grabber'),
        'config',
        'oak_package_grabber.yaml',
    ])

    return LaunchDescription([
        Node(
            package='oak_package_grabber',
            executable='oak_package_grabber_node',
            name='oak_package_grabber_node',
            output='screen',
            parameters=[config_file],
        )
    ])
