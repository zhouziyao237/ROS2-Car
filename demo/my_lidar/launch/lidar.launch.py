from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('lidar_model', default_value='X2BF'),
        DeclareLaunchArgument('frame_id', default_value='laser_frame'),
        DeclareLaunchArgument('scan_frequency', default_value='6.0'),
        DeclareLaunchArgument('range_min', default_value='0.06'),
        DeclareLaunchArgument('range_max', default_value='12.0'),
        DeclareLaunchArgument('reverse', default_value='True'),
        DeclareLaunchArgument('debug_log', default_value='false'),  # 添加开关

        Node(
            package='my_lidar',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'lidar_model': LaunchConfiguration('lidar_model'),
                'frame_id': LaunchConfiguration('frame_id'),
                'scan_frequency': LaunchConfiguration('scan_frequency'),
                'range_min': LaunchConfiguration('range_min'),
                'range_max': LaunchConfiguration('range_max'),
                'reverse': LaunchConfiguration('reverse'),
                'debug_log': LaunchConfiguration('debug_log')
            }]
        )
    ])