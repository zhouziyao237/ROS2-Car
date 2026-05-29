from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def static_tf_node(name, parent, child, x, y, z, roll, pitch, yaw):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        arguments=[
            '--x', x,
            '--y', y,
            '--z', z,
            '--roll', roll,
            '--pitch', pitch,
            '--yaw', yaw,
            '--frame-id', parent,
            '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    # =========================
    # Frame names
    # =========================
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')
    camera_link_frame = LaunchConfiguration('camera_link_frame')

    # =========================
    # LiDAR pose relative to base_link
    # 你的雷达如果在左前方：
    # x 正前方，y 左方，z 上方
    # 例如：x=0.10, y=0.18, z=0.18
    # =========================
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    # =========================
    # Camera pose relative to base_link
    # 注意：这里只发布 base_link -> camera_link
    # RealSense 内部的 camera_depth_frame / optical_frame 不要手动发
    # =========================
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    return LaunchDescription([
        # -------------------------
        # Common frames
        # -------------------------
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame'
        ),

        DeclareLaunchArgument(
            'lidar_frame',
            default_value='SCAN-LINK',
            description='LiDAR frame id, must match LaserScan header.frame_id'
        ),

        DeclareLaunchArgument(
            'camera_link_frame',
            default_value='camera_link',
            description='External camera mounting frame. Do not use optical frame here.'
        ),

        # -------------------------
        # LiDAR transform
        # -------------------------
        DeclareLaunchArgument('lidar_x', default_value='0.10'),
        DeclareLaunchArgument('lidar_y', default_value='0.18'),
        DeclareLaunchArgument('lidar_z', default_value='0.18'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        # -------------------------
        # Camera transform
        # 这里先给一个常见安装值：
        # 相机在车体前方 0.22m，中心偏左/右可改 y，高度 0.45m
        # 如果相机正朝前，roll/pitch/yaw 全部先用 0
        # -------------------------
        DeclareLaunchArgument('camera_x', default_value='0.22'),
        DeclareLaunchArgument('camera_y', default_value='0.00'),
        DeclareLaunchArgument('camera_z', default_value='0.45'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),

        # base_link -> SCAN-LINK
        static_tf_node(
            name='base_to_lidar_static_tf',
            parent=base_frame,
            child=lidar_frame,
            x=lidar_x,
            y=lidar_y,
            z=lidar_z,
            roll=lidar_roll,
            pitch=lidar_pitch,
            yaw=lidar_yaw,
        ),

        # base_link -> camera_link
        static_tf_node(
            name='base_to_camera_link_static_tf',
            parent=base_frame,
            child=camera_link_frame,
            x=camera_x,
            y=camera_y,
            z=camera_z,
            roll=camera_roll,
            pitch=camera_pitch,
            yaw=camera_yaw,
        ),
    ])
