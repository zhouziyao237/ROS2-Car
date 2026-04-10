import os
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'chassis'
    urdf_name = 'chassis.urdf'
    world_name = 'my-world.world'
    rviz_name = 'urdf.rviz'

    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)
    gazebo_world_path = os.path.join(pkg_share, 'worlds', world_name)
    rviz_config_path = os.path.join(pkg_share, 'rviz', rviz_name)

    # 读取 URDF 内容字符串
    with open(urdf_model_path, 'r', encoding='utf-8') as f:
        robot_description_content = f.read()

    # 为了 Gazebo spawn，需要将 URDF 中的 package:// 替换为绝对路径
    # 创建临时 URDF 文件
    temp_urdf = '/tmp/chassis_abs.urdf'
    subprocess.run(f"sed 's|package://chassis/meshes/|{pkg_share}/meshes/|g' {urdf_model_path} > {temp_urdf}", shell=True)

    # 设置 Gazebo 模型路径（可选，与临时 URDF 配合）
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', 
        os.path.join(pkg_share, '..')  # 指向 share 目录
    )

    # 启动 Gazebo
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen'
    )

    # 延迟 spawn 模型（等待 Gazebo 服务就绪）
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', package_name, '-file', temp_urdf, '-x', '0', '-y', '0', '-z', '0.2'],
        output='screen'
    )
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity_cmd])

    # 机器人状态发布（正确传递 URDF 字符串）
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )

    # rviz2 节点
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()
    ld.add_action(set_gazebo_model_path)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(delayed_spawn)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz2_node)
    return ld