from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare('nav_stack_core')

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    xacro_path = LaunchConfiguration('xacro_path')
    xacro_args = LaunchConfiguration('xacro_args')
    use_rviz = LaunchConfiguration('use_rviz')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    map_frame = LaunchConfiguration('map_frame')

    default_xacro = PathJoinSubstitution([pkg_share, 'description', 'minimal_lidar_robot.urdf.xacro'])
    slam_params = PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox.yaml'])
    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'nav_stack.rviz'])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' ', xacro_args]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 30.0,
            'robot_description': robot_description,
        }]
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_rviz)
    )

    odom_tf_broadcaster = Node(
        package='nav_stack_core',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'odom_topic': '/odom',
            'parent_frame': 'odom',
            'child_frame': 'base_link',
            'use_msg_child_frame': True,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('xacro_path', default_value=default_xacro),
        DeclareLaunchArgument('xacro_args', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        robot_state_publisher_node,
        odom_tf_broadcaster,
        slam_node,
        rviz_node,
    ])
