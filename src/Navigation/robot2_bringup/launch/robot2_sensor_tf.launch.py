from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def static_tf_node(
    node_name,
    parent_frame,
    child_frame,
    x,
    y,
    z,
    roll,
    pitch,
    yaw,
):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=node_name,
        output='screen',
        arguments=[
            '--x', x,
            '--y', y,
            '--z', z,
            '--roll', roll,
            '--pitch', pitch,
            '--yaw', yaw,
            '--frame-id', parent_frame,
            '--child-frame-id', child_frame,
        ],
    )


def generate_launch_description():
    base_frame = LaunchConfiguration('base_frame')

    lidar_frame = LaunchConfiguration('lidar_frame')
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    camera_link_frame = LaunchConfiguration('camera_link_frame')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame. ROS convention: x forward, y left, z up.',
        ),

        DeclareLaunchArgument(
            'lidar_frame',
            default_value='SCAN-LINK',
            description='LiDAR frame id. This must exactly match /scan.header.frame_id.',
        ),

        DeclareLaunchArgument(
            'camera_link_frame',
            default_value='camera_link',
            description='RealSense camera base frame. Do not use optical frame here.',
        ),

        # LiDAR pose relative to base_link.
        # x forward, y left, z up.
        # 如果你的雷达在左前方，y 通常是正数。
        DeclareLaunchArgument('lidar_x', default_value='0.10'),
        DeclareLaunchArgument('lidar_y', default_value='0.18'),
        DeclareLaunchArgument('lidar_z', default_value='0.18'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        # Camera pose relative to base_link.
        # 如果 D435 正面朝车头，roll/pitch/yaw 先保持 0。
        # 只发布 base_link -> camera_link。
        # RealSense 内部 frame 由 realsense2_camera 自己发布。
        DeclareLaunchArgument('camera_x', default_value='0.22'),
        DeclareLaunchArgument('camera_y', default_value='0.00'),
        DeclareLaunchArgument('camera_z', default_value='0.45'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),

        static_tf_node(
            node_name='base_to_lidar_static_tf',
            parent_frame=base_frame,
            child_frame=lidar_frame,
            x=lidar_x,
            y=lidar_y,
            z=lidar_z,
            roll=lidar_roll,
            pitch=lidar_pitch,
            yaw=lidar_yaw,
        ),

        static_tf_node(
            node_name='base_to_camera_link_static_tf',
            parent_frame=base_frame,
            child_frame=camera_link_frame,
            x=camera_x,
            y=camera_y,
            z=camera_z,
            roll=camera_roll,
            pitch=camera_pitch,
            yaw=camera_yaw,
        ),
    ])
