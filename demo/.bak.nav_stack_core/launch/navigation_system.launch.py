from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam')
    odom_mode = LaunchConfiguration('odom_mode')
    map_yaml = LaunchConfiguration('map')
    xacro_path = LaunchConfiguration('xacro_path')
    xacro_args = LaunchConfiguration('xacro_args')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    lidar_odom_topic = LaunchConfiguration('lidar_odom_topic')
    depth_odom_topic = LaunchConfiguration('depth_odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_rviz = LaunchConfiguration('use_rviz')

    pkg_share = FindPackageShare('nav_stack_core')
    nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    slam_params = PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox.yaml'])
    ekf_lidar = PathJoinSubstitution([pkg_share, 'config', 'ekf_lidar.yaml'])
    ekf_depth = PathJoinSubstitution([pkg_share, 'config', 'ekf_depth.yaml'])
    ekf_fused = PathJoinSubstitution([pkg_share, 'config', 'ekf_fused.yaml'])
    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'nav_stack.rviz'])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' ', xacro_args]),
        value_type=str
    )

    description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }]
    )

    mode_manager_node = Node(
        package='nav_stack_core',
        executable='nav_stack_mode_manager_node',
        output='screen',
        parameters=[{
            'odom_mode': odom_mode,
            'use_slam': use_slam
        }]
    )

    ekf_lidar_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_lidar',
        output='screen',
        condition=IfCondition(PythonExpression(["'", odom_mode, "' == 'lidar'"])),
        parameters=[ekf_lidar, {
            'use_sim_time': use_sim_time,
            'base_link_frame': base_frame,
            'odom_frame': odom_frame,
            'odom0': lidar_odom_topic
        }]
    )

    ekf_depth_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_depth',
        output='screen',
        condition=IfCondition(PythonExpression(["'", odom_mode, "' == 'depth'"])),
        parameters=[ekf_depth, {
            'use_sim_time': use_sim_time,
            'base_link_frame': base_frame,
            'odom_frame': odom_frame,
            'odom0': depth_odom_topic
        }]
    )

    ekf_fused_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_fused',
        output='screen',
        condition=IfCondition(PythonExpression(["'", odom_mode, "' == 'fused'"])),
        parameters=[ekf_fused, {
            'use_sim_time': use_sim_time,
            'base_link_frame': base_frame,
            'odom_frame': odom_frame,
            'odom0': lidar_odom_topic,
            'odom1': depth_odom_topic
        }]
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
        ]),
        condition=IfCondition(use_slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params
        }.items()
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py'])
        ]),
        condition=UnlessCondition(use_slam),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': nav2_params
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_slam', default_value='true'),
        DeclareLaunchArgument('odom_mode', default_value='fused', choices=['lidar', 'depth', 'fused']),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('xacro_path', default_value=PathJoinSubstitution([pkg_share, 'description', 'minimal_lidar_robot.urdf.xacro'])),
        DeclareLaunchArgument('xacro_args', default_value=''),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('lidar_odom_topic', default_value='/lidar/odom'),
        DeclareLaunchArgument('depth_odom_topic', default_value='/camera/odom/sample'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        description_node,
        mode_manager_node,
        ekf_lidar_node,
        ekf_depth_node,
        ekf_fused_node,
        slam_launch,
        localization_launch,
        navigation_launch,
        rviz_node,
    ])
