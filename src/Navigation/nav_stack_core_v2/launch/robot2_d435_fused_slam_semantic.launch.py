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


def delayed(period, actions):
    """把一组 action 延迟启动，避免硬件、TF、SLAM、Nav2 同时抢资源。"""
    return TimerAction(period=period, actions=actions)


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

    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    # -------------------------
    # Phase 0: URDF + joint states
    # 先把 base_link、SCAN-Link、cam1_Link 等静态/动态 TF 树建出来。
    # -------------------------
    robot_state_publisher = OpaqueFunction(function=make_robot_state_publisher)
    joint_state_publisher = OpaqueFunction(function=make_joint_state_publisher)

    # -------------------------
    # Phase 1: LSLIDAR M10P
    # 只把 serial_port 参数传给 lslidar，不要把外层 launch 参数乱传。
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
    # Phase 2: /scan -> /scan_front
    # 等 lslidar 先发布 /scan 后再启动，避免刚启动就空等。
    # -------------------------
    front_scan_filter_node = Node(
        package='nav_stack_core_v2',
        executable='front_scan_filter',
        name='front_scan_filter',
        output='screen',
        condition=IfCondition(start_lidar),
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

            'output_max_hz': 12.0,
            'stamp_policy': 'now',
        }]
    )

    # -------------------------
    # Phase 3: RF2O
    # 先让 /scan 稳定，再启动 rf2o。
    # rf2o 只输出 /odom_laser，不直接发布 TF，避免和 adapter 抢 odom->base_link。
    # -------------------------
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        condition=IfCondition(start_lidar),
        parameters=[{
            'laser_scan_topic': raw_scan_topic,
            'odom_topic': '/odom_laser',
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 12.0,
        }],
    )

    rf2o_odom_adapter_node = Node(
        package='nav_stack_core_v2',
        executable='rf2o_odom_adapter',
        name='rf2o_odom_adapter',
        output='screen',
        condition=IfCondition(start_lidar),
        parameters=[{
            'input_odom_topic': '/odom_laser',
            'output_odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_odom_topic': True,
            'publish_tf': True,
            'zero_z': True,
            'stamp_policy': 'now',
        }],
    )

    # -------------------------
    # Phase 4: cmd_vel mux
    # 这个节点不依赖底盘串口，可以先启动，后续 serial 启动后直接接 /cmd_vel_selected。
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

            'swap_xy': False,
            'invert_x': False,
            'invert_y': True,
            'invert_z': False,

            'debug_cmd': True,
            'debug_period': 0.5,
        }]
    )

    # -------------------------
    # Phase 5: RealSense D435
    # 只传 RealSense 自己支持的参数，避免出现大量 "Parameter not supported"。
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

    camera_link_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_static_tf',
        output='screen',
        condition=IfCondition(start_realsense),
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '3.1415926',
            '--pitch', '0.349066',
            '--yaw', '0.0',
            '--frame-id', 'cam1_Link',
            '--child-frame-id', 'camera_link'
        ],
    )

    # -------------------------
    # Phase 6: D435 depth -> /d435_scan
    # 等 D435 驱动完全启动后再启动。
    # -------------------------
    d435_depth_to_scan_node = Node(
        package='nav_stack_core_v2',
        executable='d435_depth_to_scan',
        name='d435_depth_to_scan_node',
        output='screen',
        condition=IfCondition(start_d435_scan),
        parameters=[{
            'depth_image_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'output_scan_topic': d435_scan_topic,

            'output_frame': 'base_link',

            'scan_height_pixels': 16,
            'range_min': 0.25,
            'range_max': 3.0,
            'angle_min_deg': -35.0,
            'angle_max_deg': 35.0,
        }],
    )

    # -------------------------
    # Phase 7: /scan_front + /d435_scan -> /scan_fused
    # scan_fusion 必须等 /scan_front 和 /d435_scan 都存在后再启动。
    # -------------------------
    scan_fusion_node = Node(
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

    # -------------------------
    # Phase 8: SLAM
    # SLAM 等 odom->base_link、base_link->SCAN-Link、/scan_front 都稳定后再启动。
    # 注意：SLAM 固定吃 /scan_front，不建议吃 /scan_fused。
    # -------------------------
    slam_toolbox_node = Node(
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

    # -------------------------
    # Phase 9: Nav2
    # Nav2 最后启动，等 SLAM 已经发布 map->odom。
    # -------------------------
    nav2 = IncludeLaunchDescription(
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

    # -------------------------
    # Phase 10: 底盘串口
    # 放到后面，避免 /dev/ttyACM0 不存在时一开始就刷崩整个启动日志。
    # 若只是调 SLAM，建议 start_serial:=false。
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
    # Phase 11: semantic
    # 放到 SLAM/Nav2 后面，避免抢 TF 和 topic。
    # -------------------------
    semantic_system = OpaqueFunction(function=make_semantic_system)

    # -------------------------
    # Phase 12: RViz
    # 最后启动，避免启动初期 RViz 大量订阅 scan/map/image 导致队列和 QoS 警告。
    # -------------------------
    rviz_node = Node(
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
        DeclareLaunchArgument('nav_scan_topic', default_value='/scan_front'),
        DeclareLaunchArgument('d435_scan_topic', default_value='/d435_scan'),
        DeclareLaunchArgument('fused_scan_topic', default_value='/scan_fused'),
        DeclareLaunchArgument('slam_scan_topic', default_value='/scan_front'),

        DeclareLaunchArgument(
            'semantic_map_path',
            default_value='/host-home/maps/semantic_library.json'
        ),
        DeclareLaunchArgument('reset_semantic_map', default_value='false'),

        # 0.0s：先建 TF 树
        delayed(0.0, [
            robot_state_publisher,
            joint_state_publisher,
        ]),

        # 1.0s：雷达先启动，先稳定 /scan
        delayed(1.0, [
            lidar_launch,
        ]),

        # 3.0s：有 /scan 后再启动前向过滤
        delayed(3.0, [
            front_scan_filter_node,
        ]),

        # 5.0s：有稳定 scan 后再启动 rf2o 和 odom TF
        delayed(5.0, [
            rf2o_node,
            rf2o_odom_adapter_node,
        ]),

        # 6.0s：cmd_vel mux 可以较早启动，但不抢硬件
        delayed(6.0, [
            cmd_vel_mux_node,
        ]),

        # 8.0s：再启动 D435，避免和雷达/TF 初始化抢资源
        delayed(8.0, [
            realsense_launch,
            camera_link_static_tf,
        ]),

        # 13.0s：D435 图像流稳定后，再转 /d435_scan
        delayed(13.0, [
            d435_depth_to_scan_node,
        ]),

        # 16.0s：scan_front 和 d435_scan 都稳定后，再 fusion
        delayed(16.0, [
            scan_fusion_node,
        ]),

        # 20.0s：最后启动 SLAM，等 TF 和 scan 都稳定
        delayed(20.0, [
            slam_toolbox_node,
        ]),

        # 28.0s：等 SLAM 发布 map->odom 后，再启动 Nav2
        delayed(28.0, [
            nav2,
        ]),

        # 32.0s：底盘串口靠后启动；调试 SLAM 时建议 start_serial:=false
        delayed(32.0, [
            serial_driver_launch,
        ]),

        # 36.0s：语义系统最后再进
        delayed(36.0, [
            semantic_system,
        ]),

        # 40.0s：RViz 最后启动，减少启动期队列爆掉和 QoS 警告
        delayed(40.0, [
            rviz_node,
        ]),
    ])
