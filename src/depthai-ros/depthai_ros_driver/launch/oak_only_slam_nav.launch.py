from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # OAK 相机
    oak_node = Node(
        package='depthai_examples',
        executable='rgb_stereo_node',
        name='oak_camera'
    )
    # 深度转激光
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        remappings=[
            ('depth', '/stereo/depth'),
            ('depth_camera_info', '/stereo/camera_info'),
            ('scan', '/oak_scan')
        ],
        parameters=[{
            'scan_height': 10,
            'range_min': 0.2,
            'range_max': 5.0,
            'output_frame': 'oak_rgb_camera_optical_frame'
        }]
    )
    # 激光里程计
    laser_odom = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='laser_odometry',
        parameters=[{
            'laser_scan_topic': '/oak_scan',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'freq': 10.0
        }]
    )
    # SLAM Toolbox
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'scan_topic': '/oak_scan',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'resolution': 0.05,
            'max_laser_range': 5.0,
            'mode': 'mapping'
        }]
    )
    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'use_sim_time': 'False',
            'map': '/path/to/empty_map.yaml',  # 创建空地图
            'params_file': '/path/to/nav2_oak_params.yaml'
        }.items()
    )
    return LaunchDescription([
        oak_node, depth_to_scan, laser_odom, slam, nav2
    ])