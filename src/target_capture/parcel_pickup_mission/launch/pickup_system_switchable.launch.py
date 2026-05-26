from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def pkg_share(pkg):
    return get_package_share_directory(pkg)


def generate_launch_description():
    camera_mode = LaunchConfiguration('camera_mode')
    start_serial = LaunchConfiguration('start_serial')
    wide_video_device = LaunchConfiguration('wide_video_device')
    start_chassis_align = LaunchConfiguration('start_chassis_align')

    declare_camera_mode = DeclareLaunchArgument(
        'camera_mode',
        default_value='single_oak',
        description='single_oak or usb_wide_oak_pick'
    )

    declare_start_serial = DeclareLaunchArgument(
        'start_serial',
        default_value='false'
    )

    declare_wide_video_device = DeclareLaunchArgument(
        'wide_video_device',
        default_value='/dev/video0'
    )

    declare_start_chassis_align = DeclareLaunchArgument(
        'start_chassis_align',
        default_value='true'
    )

    use_wide = PythonExpression([
        "'", camera_mode, "' == 'usb_wide_oak_pick'"
    ])

    # 1. OAK 平台取件相机，两种模式都启动
    oak_pick_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share('target_pickup_bringup'),
                'launch',
                'qr_pickup_pipeline.launch.py'
            )
        ),
        launch_arguments={
            'target_code': 'COL:1',
            'target_frame': 'oak_camera_link',
            'start_lift': 'false',
            'start_chassis_align': 'false',
        }.items()
    )

    # 2. USB 广角相机，只在 usb_wide_oak_pick 模式启动
    wide_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='wide_usb_camera',
        namespace='wide',
        output='screen',
        parameters=[{
            'video_device': wide_video_device,
            'image_size': [1280, 720],
            'pixel_format': 'YUYV',
            'camera_frame_id': 'wide_camera_link',
        }],
        condition=IfCondition(use_wide)
    )

    # 3. USB 广角 QR matcher，只在 usb_wide_oak_pick 模式启动
    wide_qr = Node(
        package='parcel_pickup_mission',
        executable='wide_qr_matcher_node',
        name='wide_qr_matcher_node',
        output='screen',
        parameters=[{
            'image_topic': '/wide/image_raw',
            'target_code_topic': '/wide_target_code',
            'initial_target_code': 'COL:1',
            'state_topic': '/wide_qr/state',
            'raw_topic': '/wide_qr/raw',
            'center_error_x_topic': '/wide_qr/center_error_x',
            'debug_image_topic': '/wide_qr/debug_image',
            'decode_scale': 0.75,
            'publish_rate_hz': 8.0,
            'publish_debug_image': True,
        }],
        condition=IfCondition(use_wide)
    )

    # 4. 串口，下位机没接时 start_serial:=false
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

    # 5. 升降 + 两节伸出
    lift_node = Node(
        package='qr_platform_lift_control',
        executable='qr_lift_control_node',
        name='qr_lift_control_node',
        output='screen',
        parameters=[
            '/host-home/src/Navigation/qr_platform_lift_control/config/pickup_control.yaml'
        ]
    )

    # 6. 底盘视觉对齐
    chassis_node = Node(
        package='qr_chassis_align_control',
        executable='qr_chassis_align_node',
        name='qr_chassis_align_node',
        output='screen',
        parameters=[
            '/host-home/src/Navigation/qr_chassis_align_control/config/qr_chassis_align.yaml',
            {'enabled': False}
        ],
        condition=IfCondition(start_chassis_align)
    )

    # 7. 自动取件状态机
    mission_node = Node(
        package='parcel_pickup_mission',
        executable='pickup_state_machine_node',
        name='parcel_pickup_mission_node',
        output='screen',
        parameters=[
            '/host-home/src/Navigation/parcel_pickup_mission/config/pickup_mission_switchable.yaml',
            {'camera_mode': camera_mode}
        ]
    )

    # 8. OSD 叠加节点
    # 这个节点输入 OAK overlay 图像，输出 /pickup_osd/image
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

            # 如果你的 OSD 节点支持 wide 信息，可以后面再加；
            # 当前先显示 OAK + mission 状态，保证能看到。
        }]
    )

    return LaunchDescription([
        declare_camera_mode,
        declare_start_serial,
        declare_wide_video_device,
        declare_start_chassis_align,

        oak_pick_pipeline,

        TimerAction(period=1.5, actions=[wide_camera]),
        TimerAction(period=2.0, actions=[wide_qr]),

        TimerAction(period=2.5, actions=[serial_driver]),
        TimerAction(period=3.0, actions=[lift_node]),
        TimerAction(period=3.5, actions=[chassis_node]),
        TimerAction(period=4.0, actions=[mission_node]),
        TimerAction(period=4.5, actions=[osd_node]),
    ])