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

    default_lidar_odom_params = os.path.join(
        nav_pkg, 'config', 'm10p_lidar_odom_nav_wheel_heading.yaml'
    )
    default_ekf_params = os.path.join(
        nav_pkg, 'config', 'ekf_wheel_lidar_wheel_heading.yaml'
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
    lidar_odom_params_file = LaunchConfiguration('lidar_odom_params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    xacro_path = LaunchConfiguration('xacro_path')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')

    wheel_odom_use_cmd_vel = LaunchConfiguration('wheel_odom_use_cmd_vel')

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

    # 原节点名不改：front_scan_filter
    # 当前功能：
    #   /scan -> /scan_front
    #   屏蔽 180~270 度，其余角度保留
    front_scan_filter_node = Node(
        package='nav_stack_core',
        executable='front_scan_filter.py',
        name='front_scan_filter',
        output='screen',
        parameters=[{
            'input_topic': '/scan',
            'output_topic': '/scan_front',

            'filter_mode': 'block_sector',
            'block_start_deg': 180.0,
            'block_end_deg': 280.0,

            'min_range': 0.18,
            'max_range': 12.0,
            'frame_id': '',
        }]
    )

    wheel_twist_odom_node = Node(
        package='nav_stack_core',
        executable='wheel_twist_odom.py',
        name='wheel_twist_odom',
        output='screen',
        parameters=[{
            'input_topic': '/wheel/twist',
            'odom_topic': '/wheel/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': False,

            'use_cmd_vel_input': wheel_odom_use_cmd_vel,
            'cmd_vel_topic': serial_cmd_vel_topic,

            # 这个 wheel_heading 版本先保持不用 IMU
            'use_imu_yaw': False,
        }]
    )

    cmd_vel_mux_node = Node(
        package='nav_stack_core',
        executable='cmd_vel_mode_mux.py',
        name='cmd_vel_mode_mux',
        output='screen',
        parameters=[{
            'manual_topic': '/cmd_vel_manual',
            'manual_topic_alt': '/red_standard_robot1/cmd_vel',
            'auto_topic': '/cmd_vel',
            'auto_topic_alt': '/cmd_vel_nav',
            'output_topic': serial_cmd_vel_topic,
            'mode_topic': '/control_mode',
            'publish_rate': 20.0,
            'cmd_timeout': 0.5,
            'default_mode': 'manual'
        }]
    )

    lidar_odom_node = Node(
        package='lidar_odom_pkg',
        executable='scan_match_odom',
        name='scan_match_odom',
        output='screen',
        parameters=[lidar_odom_params_file],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
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
            description='Selected cmd_vel topic forwarded to rm_serial_driver'
        ),

        DeclareLaunchArgument(
            'lidar_odom_params_file',
            default_value=default_lidar_odom_params,
            description='Path to lidar_odom_pkg parameter file'
        ),

        DeclareLaunchArgument(
            'ekf_params_file',
            default_value=default_ekf_params,
            description='Path to EKF parameter file'
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
            'wheel_odom_use_cmd_vel',
            default_value='false',
            description='Use real /wheel/twist feedback for wheel odom'
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
            default_value='-0.0775'
        ),

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
            default_value='-1.0'
        ),

        robot_state_publisher_node,
        serial_driver_launch,
        lidar_launch,

        # /scan -> /scan_front
        front_scan_filter_node,

        wheel_twist_odom_node,
        cmd_vel_mux_node,
        lidar_odom_node,
        ekf_node,
        nav_with_slam_launch,
    ])