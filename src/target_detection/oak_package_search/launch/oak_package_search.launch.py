from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('oak_package_search')
    default_config = os.path.join(pkg_share, 'config', 'oak_package_search.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to oak_package_search YAML config.'
    )

    return LaunchDescription([
        config_arg,
        Node(
            package='oak_package_search',
            executable='oak_package_search_node',
            name='oak_package_search_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
