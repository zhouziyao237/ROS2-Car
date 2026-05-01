from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    start_mux = LaunchConfiguration('start_mux')
    start_tag_pub = LaunchConfiguration('start_tag_pub')
    tag_odom_topic = LaunchConfiguration('tag_odom_topic')

    pkg_share = FindPackageShare('gazebo_nav_sim')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    world = PathJoinSubstitution([pkg_share, 'worlds', 'shelf_120deg.world'])
    robot_xacro = PathJoinSubstitution([pkg_share, 'urdf', 'sim_robot.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'sim_nav_view.rviz'])

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', robot_xacro
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_mux', default_value='true'),
        DeclareLaunchArgument('start_tag_pub', default_value='true'),
        DeclareLaunchArgument('tag_odom_topic', default_value='/wheel/odom'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_ros_share, 'launch', 'gazebo.launch.py'])
            ),
            launch_arguments={'world': world, 'verbose': 'false'}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_sim_nav_robot',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'sim_nav_robot',
                '-x', '0.0',
                '-y', '-3.0',
                '-z', '0.08',
                '-Y', '1.5708',
            ]
        ),

        Node(
            condition=IfCondition(start_mux),
            package='gazebo_nav_sim',
            executable='sim_cmd_vel_mode_mux.py',
            name='sim_cmd_vel_mode_mux',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            condition=IfCondition(start_tag_pub),
            package='gazebo_nav_sim',
            executable='sim_tag_zone_publisher.py',
            name='sim_tag_zone_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': tag_odom_topic,
                'trigger_distance': 0.55,
                'republish_sec': 3.0,
            }]
        ),

        Node(
            condition=IfCondition(start_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
