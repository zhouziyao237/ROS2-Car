import os
import re
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'chassis'
    pkg_share = get_package_share_directory(package_name)

    urdf_model_path = os.path.join(pkg_share, 'urdf', 'chassis.xacro')
    gazebo_world_path = os.path.join(pkg_share, 'worlds', 'my-world.world')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'rviz.rviz')
    meshes_abs_path = os.path.join(pkg_share, 'meshes')

    # 解析 xacro，移除注释
    raw_content = subprocess.check_output(
        ['xacro', urdf_model_path, 'use_sim:=true'],
        text=True
    )
    robot_description_content = re.sub(r'<!--.*?-->', '', raw_content, flags=re.DOTALL)
    robot_description_content = robot_description_content.replace(
        'package://chassis/meshes/',
        f'file://{meshes_abs_path}/'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            gazebo_world_path],
        output='screen',
        additional_env={
            'GAZEBO_PLUGIN_PATH': '/host-home/mecanum_gazebo_plugin/install/mecanum_gazebo_plugin/lib',
            'LD_LIBRARY_PATH': '/host-home/mecanum_gazebo_plugin/install/mecanum_gazebo_plugin/lib:/opt/ros/humble/lib'
        }
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'chassis', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.2'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo,
        TimerAction(period=3.0, actions=[spawn_entity]),   # 等待 Gazebo 完全启动
        rviz,
    ])