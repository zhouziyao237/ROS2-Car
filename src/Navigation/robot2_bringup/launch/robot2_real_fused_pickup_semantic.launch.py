import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch(package_name, launch_file, launch_arguments=None, condition=None):
    launch_path = os.path.join(
        get_package_share_directory(package_name),
        'launch',
        launch_file,
    )

    kwargs = {
        'launch_description_source': PythonLaunchDescriptionSource(launch_path),
    }

    if launch_arguments is not None:
        kwargs['launch_arguments'] = launch_arguments.items()

    if condition is not None:
        kwargs['condition'] = condition

    return IncludeLaunchDescription(**kwargs)


def generate_launch_description():
    # ----------------------------
    # Main switches
    # ----------------------------
    start_robot_stack = LaunchConfiguration('start_robot_stack')
    start_sensor_tf = LaunchConfiguration('start_sensor_tf')
    start_semantic = LaunchConfiguration('start_semantic')

    # ----------------------------
    # Original robot stack args
    # These should match robot2_real_fused_pickup.launch.py
    # ----------------------------
    start_lidar = LaunchConfiguration('start_lidar')
    start_serial = LaunchConfiguration('start_serial')
    serial_port = LaunchConfiguration('serial_port')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    serial_cmd_vel_topic = LaunchConfiguration('serial_cmd_vel_topic')

    start_robot_state_publisher = LaunchConfiguration('start_robot_state_publisher')
    start_realsense = LaunchConfiguration('start_realsense')
    start_d435_scan = LaunchConfiguration('start_d435_scan')
    start_scan_fusion = LaunchConfiguration('start_scan_fusion')
    start_nav2 = LaunchConfiguration('start_nav2')

    robot_description_file = LaunchConfiguration('robot_description_file')

    raw_scan_topic = LaunchConfiguration('raw_scan_topic')
    nav_scan_topic = LaunchConfiguration('nav_scan_topic')
    d435_scan_topic = LaunchConfiguration('d435_scan_topic')
    fused_scan_topic = LaunchConfiguration('fused_scan_topic')
    slam_scan_topic = LaunchConfiguration('slam_scan_topic')

    use_rviz = LaunchConfiguration('use_rviz')

    # ----------------------------
    # Fixed sensor TF args
    # ----------------------------
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')
    camera_link_frame = LaunchConfiguration('camera_link_frame')

    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    # ----------------------------
    # Semantic args
    # ----------------------------
    semantic_use_sim_time = LaunchConfiguration('semantic_use_sim_time')
    semantic_map_frame = LaunchConfiguration('semantic_map_frame')
    semantic_odom_frame = LaunchConfiguration('semantic_odom_frame')
    semantic_base_frame = LaunchConfiguration('semantic_base_frame')
    semantic_camera_frame = LaunchConfiguration('semantic_camera_frame')
    semantic_qr_topic = LaunchConfiguration('semantic_qr_topic')
    semantic_db_path = LaunchConfiguration('semantic_db_path')

    shelf_default_width = LaunchConfiguration('shelf_default_width')
    shelf_default_depth = LaunchConfiguration('shelf_default_depth')
    shelf_default_height = LaunchConfiguration('shelf_default_height')

    allow_side_qr_create_candidate_pose = LaunchConfiguration(
        'allow_side_qr_create_candidate_pose'
    )
    column_side_yaw_offset_deg = LaunchConfiguration('column_side_yaw_offset_deg')
    side_label_width_offset = LaunchConfiguration('side_label_width_offset')
    side_label_front_offset = LaunchConfiguration('side_label_front_offset')

    return LaunchDescription([
        # ----------------------------
        # Main switches
        # ----------------------------
        DeclareLaunchArgument('start_robot_stack', default_value='true'),
        DeclareLaunchArgument('start_sensor_tf', default_value='true'),
        DeclareLaunchArgument('start_semantic', default_value='true'),

        # ----------------------------
        # Robot stack arguments
        # ----------------------------
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_serial', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_cmd_vel_topic', default_value='/cmd_vel_selected'),

        DeclareLaunchArgument('start_robot_state_publisher', default_value='true'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('start_d435_scan', default_value='true'),
        DeclareLaunchArgument('start_scan_fusion', default_value='true'),
        DeclareLaunchArgument('start_nav2', default_value='true'),

        DeclareLaunchArgument(
            'robot_description_file',
            default_value='/host-home/src/Diverse/robot2/urdf/robot2.urdf',
        ),

        DeclareLaunchArgument('raw_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('nav_scan_topic', default_value='/scan_front'),
        DeclareLaunchArgument('d435_scan_topic', default_value='/d435_scan'),
        DeclareLaunchArgument('fused_scan_topic', default_value='/scan_fused'),
        DeclareLaunchArgument('slam_scan_topic', default_value='/scan_front'),

        DeclareLaunchArgument('use_rviz', default_value='true'),

        # ----------------------------
        # Sensor TF arguments
        # ----------------------------
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='SCAN-LINK'),
        DeclareLaunchArgument('camera_link_frame', default_value='camera_link'),

        DeclareLaunchArgument('lidar_x', default_value='0.10'),
        DeclareLaunchArgument('lidar_y', default_value='0.18'),
        DeclareLaunchArgument('lidar_z', default_value='0.18'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        DeclareLaunchArgument('camera_x', default_value='0.22'),
        DeclareLaunchArgument('camera_y', default_value='0.00'),
        DeclareLaunchArgument('camera_z', default_value='0.45'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),

        # ----------------------------
        # Semantic arguments
        # ----------------------------
        DeclareLaunchArgument('semantic_use_sim_time', default_value='false'),
        DeclareLaunchArgument('semantic_map_frame', default_value='map'),
        DeclareLaunchArgument('semantic_odom_frame', default_value='odom'),
        DeclareLaunchArgument('semantic_base_frame', default_value='base_link'),

        # 图像/二维码检测结果一般使用 optical frame。
        # 如果你的 QR 检测节点发布的是 depth optical，就改成 camera_depth_optical_frame。
        DeclareLaunchArgument(
            'semantic_camera_frame',
            default_value='camera_color_optical_frame',
        ),

        DeclareLaunchArgument(
            'semantic_qr_topic',
            default_value='/semantic/qr_detections',
        ),

        DeclareLaunchArgument(
            'semantic_db_path',
            default_value='/host-home/semantic_shelf_db.yaml',
        ),

        DeclareLaunchArgument('shelf_default_width', default_value='1.20'),
        DeclareLaunchArgument('shelf_default_depth', default_value='0.45'),
        DeclareLaunchArgument('shelf_default_height', default_value='1.80'),

        DeclareLaunchArgument(
            'allow_side_qr_create_candidate_pose',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'column_side_yaw_offset_deg',
            default_value='90.0',
        ),
        DeclareLaunchArgument(
            'side_label_width_offset',
            default_value='0.00',
        ),
        DeclareLaunchArgument(
            'side_label_front_offset',
            default_value='0.00',
        ),

        # ----------------------------
        # 1. Start original robot stack
        # ----------------------------
        include_launch(
            package_name='robot2_bringup',
            launch_file='robot2_real_fused_pickup.launch.py',
            launch_arguments={
                'start_lidar': start_lidar,
                'start_serial': start_serial,
                'serial_port': serial_port,
                'lidar_serial_port': lidar_serial_port,
                'serial_cmd_vel_topic': serial_cmd_vel_topic,

                'start_robot_state_publisher': start_robot_state_publisher,
                'start_realsense': start_realsense,
                'start_d435_scan': start_d435_scan,
                'start_scan_fusion': start_scan_fusion,
                'start_nav2': start_nav2,

                # 语义系统由本 wrapper 单独启动，避免重复启动。
                'start_semantic': 'false',

                'robot_description_file': robot_description_file,

                'raw_scan_topic': raw_scan_topic,
                'nav_scan_topic': nav_scan_topic,
                'd435_scan_topic': d435_scan_topic,
                'fused_scan_topic': fused_scan_topic,
                'slam_scan_topic': slam_scan_topic,

                'use_rviz': use_rviz,
            },
            condition=IfCondition(start_robot_stack),
        ),

        # ----------------------------
        # 2. Start fixed external sensor TF
        # ----------------------------
        include_launch(
            package_name='robot2_bringup',
            launch_file='robot2_sensor_tf.launch.py',
            launch_arguments={
                'base_frame': base_frame,
                'lidar_frame': lidar_frame,
                'camera_link_frame': camera_link_frame,

                'lidar_x': lidar_x,
                'lidar_y': lidar_y,
                'lidar_z': lidar_z,
                'lidar_roll': lidar_roll,
                'lidar_pitch': lidar_pitch,
                'lidar_yaw': lidar_yaw,

                'camera_x': camera_x,
                'camera_y': camera_y,
                'camera_z': camera_z,
                'camera_roll': camera_roll,
                'camera_pitch': camera_pitch,
                'camera_yaw': camera_yaw,
            },
            condition=IfCondition(start_sensor_tf),
        ),

        # ----------------------------
        # 3. Start semantic system
        # ----------------------------
        include_launch(
            package_name='robot_logistics_semantic',
            launch_file='real_semantic_only.launch.py',
            launch_arguments={
                'use_sim_time': semantic_use_sim_time,

                'map_frame': semantic_map_frame,
                'odom_frame': semantic_odom_frame,
                'base_frame': semantic_base_frame,
                'camera_frame': semantic_camera_frame,
                'qr_topic': semantic_qr_topic,
                'semantic_db_path': semantic_db_path,

                'shelf_default_width': shelf_default_width,
                'shelf_default_depth': shelf_default_depth,
                'shelf_default_height': shelf_default_height,

                'allow_side_qr_create_candidate_pose': allow_side_qr_create_candidate_pose,
                'column_side_yaw_offset_deg': column_side_yaw_offset_deg,
                'side_label_width_offset': side_label_width_offset,
                'side_label_front_offset': side_label_front_offset,
            },
            condition=IfCondition(start_semantic),
        ),
    ])
