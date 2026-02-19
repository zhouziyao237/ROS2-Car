import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 slam-nav 包的安装路径
    pkg_share = get_package_share_directory('slam-nav')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    map_yaml = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    amcl_params = os.path.join(pkg_share, 'config', 'amcl_params.yaml')

    # 地图服务器节点
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': use_sim_time,
            'map_publish_rate': 1.0   # 新增：定期发布地图，时间戳会更新为当前仿真时间
        }]
    )

    # AMCL 节点
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params]
    )

    # 生命周期管理器（激活 map_server 和 amcl）
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        map_server,
        amcl,
        lifecycle_manager
    ])