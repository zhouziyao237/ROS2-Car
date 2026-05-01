from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_shelf_search = LaunchConfiguration('enable_shelf_search')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_shelf_search',
            default_value='true',
            description='Start shelf_search_manager'
        ),

        Node(
            package='nav_mission_core',
            executable='mission_task_manager.py',
            name='mission_task_manager',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'base_frame': 'base_link',
                'control_mode_topic': '/control_mode',
                'auto_mode_value': 'auto',
                'search_mode_value': 'search',
                'navigate_action_name': '/navigate_to_pose',
            }]
        ),

        Node(
            condition=IfCondition(enable_shelf_search),
            package='nav_mission_core',
            executable='shelf_search_manager.py',
            name='shelf_search_manager',
            output='screen',
            parameters=[{
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
    ])
