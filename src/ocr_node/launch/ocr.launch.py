from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ocr_node')
    # config_file = os.path.join(pkg_share, 'config', 'ocr.yaml')
    config_file = os.path.join(
        pkg_share,
        "config",
        "ocr_realtime.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument('input_mode', default_value='topic'),
        DeclareLaunchArgument('image_path', default_value=''),
        DeclareLaunchArgument('save_path', default_value=''),
        DeclareLaunchArgument('exit_after_image', default_value='true'),
        DeclareLaunchArgument('log_recognized_text', default_value='false'),
        Node(
            package='ocr_node',
            executable='ocr_node',
            name='ocr_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'input_mode': LaunchConfiguration('input_mode'),
                    'image_path': LaunchConfiguration('image_path'),
                    'save_path': LaunchConfiguration('save_path'),
                    'exit_after_image': LaunchConfiguration('exit_after_image'),
                    'log_recognized_text': LaunchConfiguration('log_recognized_text'),
                },
            ],
        )
    ])
