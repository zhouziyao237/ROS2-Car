from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    sim_pkg = FindPackageShare('gazebo_nav_sim')
    nav2_pkg = FindPackageShare('nav2_bringup')

    nav2_params = PathJoinSubstitution([sim_pkg, 'config', 'nav2_params_sim_localization.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sim_pkg, 'launch', 'sim_world.launch.py'])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'start_rviz': 'true',
                'start_mux': 'true',
                'start_tag_pub': 'true',
                'tag_odom_topic': '/wheel/odom',
            }.items()
        ),

        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([nav2_pkg, 'launch', 'localization_launch.py'])
                    ),
                    launch_arguments={
                        'map': map_yaml,
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_params,
                    }.items()
                ),
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([sim_pkg, 'launch', 'nav2_navigation_plain.launch.py'])
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_params,
                    }.items()
                ),
            ]
        ),

        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='nav_mission_core',
                    executable='mission_task_manager.py',
                    name='mission_task_manager',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'map_frame': 'map',
                        'base_frame': 'base_link',
                        'control_mode_topic': '/control_mode',
                        'auto_mode_value': 'auto',
                        'search_mode_value': 'search',
                    }]
                ),
                Node(
                    package='nav_mission_core',
                    executable='shelf_search_manager.py',
                    name='shelf_search_manager',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'cmd_vel_topic': '/cmd_vel_search',
                        'scan_topic': '/scan',
                        'map_frame': 'map',
                        'rotate_speed': 0.35,
                        'forward_speed': 0.08,
                        'rotate_time_sec': 8.0,
                        'forward_time_sec': 4.0,
                        'front_safety_distance': 0.45,
                        'front_angle_deg': 30.0,
                    }]
                ),
            ]
        ),
    ])
