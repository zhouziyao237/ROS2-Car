import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = LaunchConfiguration('config_path')
    blob_path = LaunchConfiguration('blob_path')
    parser = LaunchConfiguration('parser')
    conf = LaunchConfiguration('conf')
    iou = LaunchConfiguration('iou')

    default_config = os.path.join(
        get_package_share_directory('oak_vision_driver'),
        'config',
        'yolov5_oak.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('blob_path', default_value=''),
        DeclareLaunchArgument('parser', default_value=''),
        DeclareLaunchArgument('conf', default_value='-1.0'),
        DeclareLaunchArgument('iou', default_value='-1.0'),

        Node(
            package='oak_vision_driver',
            executable='oak_vision_driver_node',
            name='oak_vision_driver_node',
            output='screen',
            parameters=[{
                'config_path': config_path,
                'override_blob_path': blob_path,
                'override_parser': parser,
                'override_conf': conf,
                'override_iou': iou,
            }]
        )
    ])
