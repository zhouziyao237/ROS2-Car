#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def read_robot_description(context):
    robot_description_file = LaunchConfiguration('robot_description_file').perform(context)

    if not os.path.exists(robot_description_file):
        raise FileNotFoundError(
            f'robot_description_file not found: {robot_description_file}'
        )

    with open(robot_description_file, 'r', encoding='utf-8') as f:
        return f.read()


def make_robot_state_publisher(context, *args, **kwargs):
    robot_description = read_robot_description(context)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_robot_state_publisher')),
            parameters=[{
                'use_sim_time': False,
                'robot_description': robot_description,
            }],
        )
    ]


def make_joint_state_publisher(context, *args, **kwargs):
    """
    robot2.urdf 里的 4 个轮子 joint 是 continuous：
      FL-Joint, FR-Joint, BL-Joint, BR-Joint

    continuous/revolute joint 不会被 robot_state_publisher 自动发 TF。
    必须有 /joint_states，轮子 link 才会出现在 TF 树里。
    """
    robot_description = read_robot_description(context)

    return [
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_robot_state_publisher')),
            parameters=[{
                'use_sim_time': False,
                'robot_description': robot_description,
                'rate': 30,
            }],
        )
    ]


def make_semantic_system(context, *args, **kwargs):
    start_semantic = LaunchConfiguration('start_semantic').perform(context).lower()
    if start_semantic not in ['true', '1', 'yes']:
        return []

    semantic_map_path = LaunchConfiguration('semantic_map_path').perform(context)
    reset_semantic_map = LaunchConfiguration('reset_semantic_map').perform(context)

    try:
        semantic_pkg = get_package_share_directory('robot_logistics_semantic')
    except Exception as e:
        print(f'[semantic] robot_logistics_semantic not found: {e}')
        return []

    semantic_launch = os.path.join(
        semantic_pkg,
        'launch',
        'real_semantic_only.launch.py'
    )

    if not os.path.exists(semantic_launch):
        print(f'[semantic] launch file not found: {semantic_launch}')
        return []

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(semantic_launch),
            launch_arguments={
                'semantic_map_path': semantic_map_path,
                'reset_semantic_map': reset_semantic_map,
            }.items()
        )
    ]


def generate_launch_description():
    start_lidar = LaunchConfiguration('start_lidar')
    start_serial = LaunchConfiguration('start_serial')
    start_robot_state_publisher = LaunchConfiguration('start_robot_state_publisher')
    start_realsense = LaunchConfiguration('start_realsense')
    start_d435_scan = LaunchConfiguration('start_d435_scan')
    start_scan_fusion = LaunchConfiguration('start_scan_fusion')
    start_nav2 = LaunchConfiguration('start_nav2')
    start_semantic = LaunchConfiguration('start_semantic')
    use_rviz = LaunchConfiguration('use_rviz')

    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    serial_port = LaunchConfiguration('serial_port')
    serial_cmd_vel_topic = LaunchConfiguration('serial_cmd_vel_topic')

    raw_scan_topic = LaunchConfiguration('raw_scan_topic')
    nav_scan_topic = LaunchConfiguration('nav_scan_topic')
    d435_scan_topic = LaunchConfiguration('d435_scan_topic')
    fused_scan_topic = LaunchConfiguration('fused_scan_topic')
    slam_scan_topic = LaunchConfiguration('slam_scan_topic')

    nav_pkg = get_package_share_directory('nav_stack_core_v2')
    serial_pkg = get_package_share_directory('rm_serial_driver')
    lslidar_pkg = get_package_share_directory('lslidar_driver')

    rf2o_params_file = LaunchConfiguration('rf2o_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    # -------------------------
    # 底盘串口：只负责接收 /cmd_vel_selected 控制底盘
    # 注意：rm_serial_driver 不能再发布 /odom 或 odom->base_link
    # -------------------------
    serial_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(serial_pkg, 'launch', 'serial_driver.launch.py')
        ),
        condition=IfCondition(start_serial),
        launch_arguments={
            'cmd_vel_topic': serial_cmd_vel_topic,
            'serial_port': serial_port,
            'device_name': serial_port,
            'port': serial_port,

            'wheel_vx_scale': '0.0775',
            'wheel_vy_scale': '0.0775',
            'wheel_wz_scale': '1.0',

            'cmd_vx_scale': '1.0',
            'cmd_vy_scale': '1.0',
            'cmd_wz_scale': '1.0',

            'nav_cmd_timeout': '0.10',
            'nav_cmd_send_rate': '50.0',

            'platform_target_topic': '/platform/target',
            'platform_current_status_topic': '/platform/current_status',
        }.items()
    )

    # -------------------------
    # LSLIDAR M10P
    # -------------------------
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lslidar_pkg, 'launch', 'lsm10p_uart_launch.py')
        ),
        condition=IfCondition(start_lidar),
        launch_arguments={
            'serial_port': lidar_serial_port,
        }.items()
    )

    # -------------------------
    # URDF + joint states
    # -------------------------
    robot_state_publisher = OpaqueFunction(function=make_robot_state_publisher)
    joint_state_publisher = OpaqueFunction(function=make_joint_state_publisher)

    # -------------------------
    # /scan -> /scan_front
    # SLAM 固定用 /scan_front
    # 不要把 nav_scan_topic 改成 /scan_fused，否则 /scan_front 可能没了
    # -------------------------
    front_scan_filter_node = Node(
        package='nav_stack_core_v2',
        executable='front_scan_filter',
        name='front_scan_filter',
        output='screen',
        parameters=[{
            'input_topic': raw_scan_topic,
            'output_topic': nav_scan_topic,

            'filter_mode': 'block_sector',
            'block_start_deg': 180.0,
            'block_end_deg': 280.0,

            'min_range': 0.18,
            'max_range': 12.0,
            'frame_id': '',

            'reverse_scan': False,

            'force_fixed_count': True,
            'fixed_count': 1324,
            'fixed_angle_min': 0.0,
            'fixed_angle_max': 6.283185307179586,

            # 如果 front_scan_filter.py 支持这两个参数，就启用低延迟策略
            'output_max_hz': 12.0,
            'stamp_policy': 'now',
        }]
    )

    # -------------------------
    # cmd_vel mux
    # Nav2: /cmd_vel 或 /cmd_vel_smoothed
    # 输出：/cmd_vel_selected -> rm_serial_driver
    # -------------------------
    cmd_vel_mux_node = Node(
        package='nav_stack_core_v2',
        executable='cmd_vel_mode_mux',
        name='cmd_vel_mode_mux',
        output='screen',
        parameters=[{
            'manual_topic': '/cmd_vel_manual',
            'manual_topic_alt': '',

            'auto_topic': '/cmd_vel_smoothed',
            'auto_topic_alt': '/cmd_vel',

            'output_topic': serial_cmd_vel_topic,

            'mode_topic': '/control_mode',
            'chassis_mode_topic': '',

            'publish_rate': 50.0,
            'cmd_timeout': 0.08,
            'default_mode': 'auto',

            # 按你当前底盘坐标修正
            'swap_xy': True,
            'invert_x': True,
            'invert_y': False,
            'invert_z': False,

            'debug_cmd': True,
            'debug_period': 0.5,
        }]
    )

    # -------------------------
    # RF2O：只输出 /odom_laser，不发 TF，不直接发 /odom
    # -------------------------
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_laser',
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 12.0,
        }],
    )

    # -------------------------
    # /odom_laser -> /odom + odom -> base_link
    # -------------------------
    rf2o_odom_adapter_node = Node(
        package='nav_stack_core_v2',
        executable='rf2o_odom_adapter',
        name='rf2o_odom_adapter',
        output='screen',
        parameters=[{
            'input_odom_topic': '/odom_laser',
            'output_odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_odom_topic': True,
            'publish_tf': True,
            'zero_z': True,
        }],
    )

    # -------------------------
    # RealSense D435
    # -------------------------
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        condition=IfCondition(start_realsense),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'false',
            'align_depth.enable': 'true',
            'enable_sync': 'true',

            'rgb_camera.color_profile': '424x240x15',
            'depth_module.depth_profile': '424x240x15',
        }.items()
    )

    # 把 RealSense 驱动自己的 camera_link 接到 robot2 URDF 里的 cam1_Link。
    # URDF 已经有 base_link -> cam1_Link，所以这里用 0 0 0 0 0 0。
    camera_link_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_static_tf',
        output='screen',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'cam1_Link',
            'camera_link'
        ],
        condition=IfCondition(start_realsense),
    )

    # -------------------------
    # D435 depth -> /d435_scan
    # -------------------------
    d435_depth_to_scan = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='nav_stack_core_v2',
                executable='d435_depth_to_scan',
                name='d435_depth_to_scan_node',
                output='screen',
                condition=IfCondition(start_d435_scan),
                parameters=[{
                    'depth_image_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                    'camera_info_topic': '/camera/camera/color/camera_info',
                    'output_scan_topic': d435_scan_topic,

                    # 直接输出到 base_link，方便 scan_fusion 和 costmap 使用
                    'output_frame': 'base_link',

                    'scan_height_pixels': 16,
                    'range_min': 0.25,
                    'range_max': 3.0,
                    'angle_min_deg': -35.0,
                    'angle_max_deg': 35.0,
                }],
            )
        ]
    )

    # -------------------------
    # /scan_front + /d435_scan -> /scan_fused
    # -------------------------
    scan_fusion = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='nav_stack_core_v2',
                executable='scan_fusion_node',
                name='scan_fusion_node',
                output='screen',
                condition=IfCondition(start_scan_fusion),
                parameters=[{
                    'lidar_scan_topic': nav_scan_topic,
                    'depth_scan_topic': d435_scan_topic,
                    'output_scan_topic': fused_scan_topic,

                    'enable_d435_clearing': False,

                    'range_min': 0.18,
                    'range_max': 12.0,
                    'depth_timeout_sec': 0.5,
                }],
            )
        ]
    )

    # -------------------------
    # SLAM：只吃 /scan_front
    # 输出 /map 和 map -> odom
    # -------------------------
    slam_toolbox = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {
                        'use_sim_time': False,
                        'odom_frame': 'odom',
                        'map_frame': 'map',
                        'base_frame': 'base_link',
                        'scan_topic': slam_scan_topic,
                    }
                ],
            )
        ]
    )

    # -------------------------
    # Nav2：延迟启动，等 map -> base_link 出来
    # Nav2 是否用 /scan_fused，由 nav2_params_file 控制
    # -------------------------
    nav2 = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    )
                ),
                condition=IfCondition(start_nav2),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params_file,
                    'autostart': 'true',
                }.items()
            )
        ]
    )

    semantic_system = TimerAction(
        period=20.0,
        actions=[
            OpaqueFunction(function=make_semantic_system)
        ]
    )

    rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                condition=IfCondition(use_rviz),
                arguments=['-d', rviz_config],
                parameters=[{
                    'use_sim_time': False,
                }],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_serial', default_value='true'),
        DeclareLaunchArgument('start_robot_state_publisher', default_value='true'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('start_d435_scan', default_value='true'),
        DeclareLaunchArgument('start_scan_fusion', default_value='true'),
        DeclareLaunchArgument('start_nav2', default_value='true'),
        DeclareLaunchArgument('start_semantic', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        DeclareLaunchArgument(
            'robot_description_file',
            default_value='/host-home/src/Diverse/robot2/urdf/robot2.urdf'
        ),

        DeclareLaunchArgument(
            'rf2o_params_file',
            default_value=os.path.join(nav_pkg, 'config', 'rf2o_lidar_odom.yaml')
        ),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(nav_pkg, 'config', 'slam_toolbox_online_highres.yaml')
        ),

        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav_pkg, 'config', 'nav2_params_min.yaml')
        ),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(nav_pkg, 'rviz', 'nav_stack.rviz')
        ),

        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_cmd_vel_topic', default_value='/cmd_vel_selected'),

        DeclareLaunchArgument('raw_scan_topic', default_value='/scan'),

        # 这个保持 /scan_front，不要改成 /scan_fused
        DeclareLaunchArgument('nav_scan_topic', default_value='/scan_front'),

        DeclareLaunchArgument('d435_scan_topic', default_value='/d435_scan'),
        DeclareLaunchArgument('fused_scan_topic', default_value='/scan_fused'),

        # slam_toolbox 固定吃 /scan_front
        DeclareLaunchArgument('slam_scan_topic', default_value='/scan_front'),

        DeclareLaunchArgument(
            'semantic_map_path',
            default_value='/host-home/maps/semantic_library.json'
        ),
        DeclareLaunchArgument('reset_semantic_map', default_value='false'),

        serial_driver_launch,
        lidar_launch,

        robot_state_publisher,
        joint_state_publisher,

        front_scan_filter_node,
        cmd_vel_mux_node,
        rf2o_node,
        rf2o_odom_adapter_node,

        realsense_launch,
        camera_link_static_tf,
        d435_depth_to_scan,
        scan_fusion,

        slam_toolbox,
        nav2,
        semantic_system,
        rviz,
    ])
