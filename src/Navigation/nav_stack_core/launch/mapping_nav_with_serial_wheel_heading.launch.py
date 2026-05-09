import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_pkg = get_package_share_directory('rm_serial_driver')
    nav_pkg = get_package_share_directory('nav_stack_core')
    lslidar_pkg = get_package_share_directory('lslidar_driver')

    default_rf2o_params = os.path.join(
        nav_pkg, 'config', 'rf2o_lidar_odom.yaml'
    )

    default_xacro = os.path.join(
        nav_pkg, 'description', 'minimal_lidar_robot.xacro'
    )

    default_slam = os.path.join(
        nav_pkg, 'config', 'slam_toolbox_online_highres.yaml'
    )

    default_nav2 = os.path.join(
        nav_pkg, 'config', 'nav2_params_min.yaml'
    )

    default_rviz = os.path.join(
        nav_pkg, 'rviz', 'nav_stack.rviz'
    )

    serial_cmd_vel_topic = LaunchConfiguration('serial_cmd_vel_topic')
    rf2o_params_file = LaunchConfiguration('rf2o_params_file')
    xacro_path = LaunchConfiguration('xacro_path')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')

    wheel_vx_scale = LaunchConfiguration('wheel_vx_scale')
    wheel_vy_scale = LaunchConfiguration('wheel_vy_scale')
    wheel_wz_scale = LaunchConfiguration('wheel_wz_scale')

    cmd_vx_scale = LaunchConfiguration('cmd_vx_scale')
    cmd_vy_scale = LaunchConfiguration('cmd_vy_scale')
    cmd_wz_scale = LaunchConfiguration('cmd_wz_scale')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }],
    )

    serial_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                serial_pkg,
                'launch',
                'serial_driver.launch.py'
            )
        ),
        launch_arguments={
            'cmd_vel_topic': serial_cmd_vel_topic,

            'wheel_vx_scale': wheel_vx_scale,
            'wheel_vy_scale': wheel_vy_scale,
            'wheel_wz_scale': wheel_wz_scale,

            'cmd_vx_scale': cmd_vx_scale,
            'cmd_vy_scale': cmd_vy_scale,
            'cmd_wz_scale': cmd_wz_scale,

            # 新增：串口驱动持续发最新速度；超时强制发 0
            'nav_cmd_timeout': '0.10',
            'nav_cmd_send_rate': '50.0',

            'platform_target_topic': '/platform/target',
            'platform_current_status_topic': '/platform/current_status',
        }.items()
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                lslidar_pkg,
                'launch',
                'lsm10p_uart_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': lidar_serial_port
        }.items()
    )

    front_scan_filter_node = Node(
        package='nav_stack_core',
        executable='front_scan_filter.py',
        name='front_scan_filter',
        output='screen',
        parameters=[{
            'input_topic': '/scan',
            'output_topic': '/scan_front',
            'filter_mode': 'block_sector',

            # 你的屏蔽范围
            'block_start_deg': 180.0,
            'block_end_deg': 280.0,

            'min_range': 0.18,
            'max_range': 12.0,
            'frame_id': '',

            # 雷达方向：0 前，90 左，180 后，270 右
            'reverse_scan': False,

            # 固定点数，避免 RF2O / slam_toolbox 输入结构跳变
            'force_fixed_count': True,
            'fixed_count': 1324,
            'fixed_angle_min': -3.141592653589793,
            'fixed_angle_max': 3.141592653589793,
        }]
    )

    cmd_vel_mux_node = Node(
        package='nav_stack_core',
        executable='cmd_vel_mode_mux.py',
        name='cmd_vel_mode_mux',
        output='screen',
        parameters=[{
            'manual_topic': '/cmd_vel_manual',
            'manual_topic_alt': '',

            # 优先吃 velocity_smoother，如果没有则 fallback 到 /cmd_vel
            'auto_topic': '/cmd_vel_smoothed',
            'auto_topic_alt': '/cmd_vel',

            # 输出给 rm_serial_driver
            'output_topic': serial_cmd_vel_topic,

            'mode_topic': '/control_mode',
            'chassis_mode_topic': '',

            # 高频发控制
            'publish_rate': 50.0,

            # 关键：0.08 秒没有新速度，立即发 0
            'cmd_timeout': 0.08,

            # 默认直接 auto，避免你忘记发 /control_mode
            'default_mode': 'auto',

            # ROS:
            #   +linear.x = 前进
            #   +linear.y = 左平移
            #
            # 底盘:
            #   +vy = 前进
            #   -vx = 左平移
            #
            # 所以:
            #   chassis_vx = -ros_y
            #   chassis_vy = +ros_x
            'swap_xy': True,
            'invert_x': True,
            'invert_y': False,
            'invert_z': False,

            'debug_cmd': True,
            'debug_period': 0.5,
        }]
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_params_file],
        remappings=[
            ('/scan', '/scan_front'),
            ('scan', '/scan_front'),
            ('/odom', '/odom'),
            ('odom', '/odom'),
        ],
    )

    nav_with_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav_pkg,
                'launch',
                'navigation_system_with_lidar_odom.launch.py'
            )
        ),
        launch_arguments={
            'xacro_path': xacro_path,
            'slam_params_file': slam_params_file,
            'nav2_params_file': nav2_params_file,
            'rviz_config': rviz_config,
            'launch_robot_state_publisher': 'false',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_cmd_vel_topic',
            default_value='/cmd_vel_selected',
            description='cmd_vel topic forwarded to rm_serial_driver'
        ),

        DeclareLaunchArgument(
            'rf2o_params_file',
            default_value=default_rf2o_params,
            description='Path to RF2O parameter file'
        ),

        DeclareLaunchArgument(
            'xacro_path',
            default_value=default_xacro,
            description='Path to robot xacro used by robot_state_publisher'
        ),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam,
            description='Path to slam_toolbox parameter file'
        ),

        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2,
            description='Path to Nav2 parameter file'
        ),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Path to RViz config file'
        ),

        DeclareLaunchArgument(
            'lidar_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device used by the M10P lidar driver'
        ),

        DeclareLaunchArgument(
            'wheel_vx_scale',
            default_value='0.0775'
        ),

        DeclareLaunchArgument(
            'wheel_vy_scale',
            default_value='0.0775'
        ),

        DeclareLaunchArgument(
            'wheel_wz_scale',
            default_value='1.0'
        ),

        # 串口驱动层不做方向转换，保持 1.0
        DeclareLaunchArgument(
            'cmd_vx_scale',
            default_value='1.0'
        ),

        DeclareLaunchArgument(
            'cmd_vy_scale',
            default_value='1.0'
        ),

        DeclareLaunchArgument(
            'cmd_wz_scale',
            default_value='1.0'
        ),

        robot_state_publisher_node,
        serial_driver_launch,
        lidar_launch,
        front_scan_filter_node,
        cmd_vel_mux_node,
        rf2o_node,
        nav_with_slam_launch,
    ])