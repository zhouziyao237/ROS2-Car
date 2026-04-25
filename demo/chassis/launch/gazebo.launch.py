import os
import re
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'chassis'
    pkg_share = get_package_share_directory(package_name)

    urdf_model_path = os.path.join(pkg_share, 'urdf', 'chassis.xacro')
    gazebo_world_path = os.path.join(pkg_share, 'worlds', 'my-world.world')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'rviz.rviz')
    meshes_abs_path = os.path.join(pkg_share, 'meshes')
    robot_config = os.path.join(pkg_share, 'config', 'robot_config.yaml')

    # 读取 YAML
    with open(robot_config, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    # 正确读取 physics/ros__parameters
    physics = config.get('physics', {}).get('ros__parameters', {})
    wheel_mu1 = physics.get('wheel_mu1', 0.8)
    wheel_mu2 = physics.get('wheel_mu2', 0.6)
    wheel_kp = physics.get('wheel_kp', 100000.0)
    wheel_kd = physics.get('wheel_kd', 10.0)

    # xacro 参数
    xacro_args = [
        'use_sim:=true',
        f'wheel_mu1:={wheel_mu1}',
        f'wheel_mu2:={wheel_mu2}',
        f'wheel_kp:={wheel_kp}',
        f'wheel_kd:={wheel_kd}',
    ]

    # 生成 robot_description
    raw_content = subprocess.check_output(
        ['xacro', urdf_model_path] + xacro_args,
        text=True
    )

    # 去注释
    robot_description_content = re.sub(r'<!--.*?-->', '', raw_content, flags=re.DOTALL)

    # 替换 mesh 路径
    robot_description_content = robot_description_content.replace(
        'package://chassis/meshes/',
        f'file://{meshes_abs_path}/'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            gazebo_world_path
        ],
        output='screen',
        additional_env={
            'GAZEBO_PLUGIN_PATH': '/host-home/mecanum_gazebo_plugin/install/mecanum_gazebo_plugin/lib',
            'LD_LIBRARY_PATH': '/host-home/mecanum_gazebo_plugin/install/mecanum_gazebo_plugin/lib:/opt/ros/humble/lib'
        }
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'chassis',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.2'
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    reset_world = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/reset_simulation', 'std_srvs/srv/Empty'],
        output='screen'
    )

    unified_odometry = Node(
        package='mecanum_gazebo_plugin',
        executable='unified_odometry',
        name='unified_odometry',
        output='screen',
        parameters=[robot_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        robot_state_publisher,
        gazebo,

        # 先生成机器人
        TimerAction(
            period=3.0,
            actions=[spawn_entity]
        ),

        rviz,

        # 等 Gazebo 和机器人稳定后重置一次
        TimerAction(
            period=8.0,
            actions=[reset_world]
        ),

        # 最后再启动 odometry，避免第一帧 NaN
        TimerAction(
            period=12.0,
            actions=[unified_odometry]
        ),
    ])