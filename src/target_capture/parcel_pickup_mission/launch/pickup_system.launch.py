from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def pkg_share(pkg):
    return get_package_share_directory(pkg)


def generate_launch_description():
    target_code = LaunchConfiguration('target_code')
    target_frame = LaunchConfiguration('target_frame')
    start_serial = LaunchConfiguration('start_serial')
    start_chassis_align = LaunchConfiguration('start_chassis_align')

    declare_target_code = DeclareLaunchArgument(
        'target_code',
        default_value='COL:1'
    )

    declare_target_frame = DeclareLaunchArgument(
        'target_frame',
        default_value='oak_camera_link'
    )

    # 调试时默认 false，避免 /dev/ttyACM0 不存在导致 launch 崩。
    # 实车接上下位机后启动时加 start_serial:=true
    declare_start_serial = DeclareLaunchArgument(
        'start_serial',
        default_value='false'
    )

    declare_start_chassis_align = DeclareLaunchArgument(
        'start_chassis_align',
        default_value='true'
    )

    # 1. OAK + QR matcher + target pose overlay
    qr_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share('target_pickup_bringup'),
                'launch',
                'qr_pickup_pipeline.launch.py'
            )
        ),
        launch_arguments={
            'target_code': target_code,
            'target_frame': target_frame,
            'start_lift': 'false',
            'start_chassis_align': 'false',
        }.items()
    )

    # 2. 串口，下位机没接时不要启动
    serial_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share('rm_serial_driver'),
                'launch',
                'serial_driver.launch.py'
            )
        ),
        launch_arguments={
            'cmd_vel_topic': '/cmd_vel_selected',
            'platform_target_topic': '/platform/target',
            'platform_current_status_topic': '/platform/current_status',
        }.items(),
        condition=IfCondition(start_serial)
    )

    # 3. 升降 + 两节伸出
    # 这里直接用源码路径，避免 config 没安装到 share 时报错。
    lift_config = '/host-home/src/Navigation/qr_platform_lift_control/config/pickup_control.yaml'

    lift_node = Node(
        package='qr_platform_lift_control',
        executable='qr_lift_control_node',
        name='qr_lift_control_node',
        output='screen',
        parameters=[lift_config]
    )

    # 4. 底盘视觉对齐
    chassis_align_config = '/host-home/src/Navigation/qr_chassis_align_control/config/qr_chassis_align.yaml'

    chassis_align_node = Node(
        package='qr_chassis_align_control',
        executable='qr_chassis_align_node',
        name='qr_chassis_align_node',
        output='screen',
        parameters=[
            chassis_align_config,
            {'enabled': False}
        ],
        condition=IfCondition(start_chassis_align)
    )

    # 5. 自动取件状态机
    mission_config = os.path.join(
        pkg_share('parcel_pickup_mission'),
        'config',
        'pickup_mission.yaml'
    )

    mission_node = Node(
        package='parcel_pickup_mission',
        executable='pickup_state_machine_node',
        name='parcel_pickup_mission_node',
        output='screen',
        parameters=[mission_config]
    )

    osd_node = Node(
        package='parcel_pickup_mission',
        executable='pickup_osd_node',
        name='pickup_osd_node',
        output='screen',
        parameters=[{
            'input_image_topic': '/target_position_overlay/image',
            'output_image_topic': '/pickup_osd/image',
            'mission_state_topic': '/mission/state',
            'mission_debug_topic': '/mission/debug',
            'qr_state_topic': '/target_matcher/state',
            'qr_raw_topic': '/target_matcher/raw',
            'target_pose_topic': '/target_pose_base',
        }]
    )

    return LaunchDescription([
        declare_target_code,
        declare_target_frame,
        declare_start_serial,
        declare_start_chassis_align,

        qr_pipeline,

        TimerAction(period=2.0, actions=[serial_driver]),
        TimerAction(period=3.0, actions=[lift_node]),
        TimerAction(period=3.5, actions=[chassis_align_node]),
        TimerAction(period=4.0, actions=[mission_node]),
        TimerAction(period=4.5, actions=[osd_node]),
    ])