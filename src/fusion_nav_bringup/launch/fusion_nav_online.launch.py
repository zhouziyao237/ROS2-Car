import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取当前包和Nav2包的路径
    pkg_dir = get_package_share_directory('fusion_nav_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 1. 启动LSLIDAR雷达驱动节点
    lslidar_node = Node(
        package='lslidar_driver',           # 请替换为你的雷达驱动包名
        executable='lslidar_driver_node',   # 请替换为你的雷达驱动节点名
        name='lslidar_driver',
        output='screen'
    )

    # 2. 启动OAK-D相机驱动 (使用官方launch文件)
    oak_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')
        )
    )

    # 3. 启动深度图转激光节点
    depth_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_laserscan',
        remappings=[
            ('depth', '/oak/stereo/image_raw'),        # 映射深度话题
            ('depth_camera_info', '/oak/stereo/camera_info'), # 映射相机信息话题
            ('scan', '/oak_scan')                     # 输出话题
        ],
        parameters=[{
            'scan_height': 10,
            'range_min': 0.45,
            'range_max': 5.0,
            'output_frame': 'oak_rgb_camera_optical_frame'
        }]
    )

    # 4. 启动SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(pkg_dir, 'config', 'slam_params.yaml')],
        output='screen'
    )

    # 5. 启动Navigation2 (使用SLAM模式)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'True',                                # 关键：启用SLAM模式
            'use_sim_time': 'false',
            'map': os.path.join(pkg_dir, 'maps', 'empty.yaml'), # 提供一个空地图占位
            'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'fusion_nav.rviz')]
    )

    return LaunchDescription([
        lslidar_node,
        oak_launch,
        depth_to_laserscan_node,
        slam_node,
        nav2_launch,
        rviz_node,
    ])