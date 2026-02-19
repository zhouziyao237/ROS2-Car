import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(
        get_package_share_directory('my_nav'),
        'config',
        'nav2_params.yaml'
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true',
            'map': ''  # 不启动 map_server（已由 slam-nav 提供）
        }.items()
    )

    return LaunchDescription([
        nav2_bringup_launch
    ])