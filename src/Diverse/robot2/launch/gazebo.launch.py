from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    urdf_path = PathJoinSubstitution([
        FindPackageShare('robot2'),
        'urdf',
        'robot2.urdf'
    ])

    empty_world = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'worlds',
        'empty.world'
    ])

    robot_description = ParameterValue(
        Command(['cat ', urdf_path]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),

        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.10'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                empty_world,
                '-s',
                'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_robot2',
                    output='screen',
                    arguments=[
                        '-topic', 'robot_description',
                        '-entity', 'robot2',
                        '-x', x,
                        '-y', y,
                        '-z', z,
                        '-Y', yaw
                    ]
                )
            ]
        )
    ])
