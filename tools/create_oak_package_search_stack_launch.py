from pathlib import Path
import time

candidates = [
    Path("/host-home/src/Navigation/robot2_bringup/launch"),
    Path("/host-home/src/Diverse/robot2_bringup/launch"),
    Path("/host-home/src/robot2_bringup/launch"),
]

launch_dir = None
for p in candidates:
    if p.exists():
        launch_dir = p
        break

if launch_dir is None:
    raise SystemExit("[ERROR] Cannot find robot2_bringup/launch folder.")

out = launch_dir / "oak_package_search_stack.launch.py"

if out.exists():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    bak = out.with_suffix(out.suffix + f".bak_{stamp}")
    bak.write_text(out.read_text(errors="ignore"))
    print(f"[BACKUP] {out} -> {bak}")

content = r'''from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_oak_driver = LaunchConfiguration("enable_oak_driver")
    enable_oak_parcel_pose = LaunchConfiguration("enable_oak_parcel_pose")
    enable_oak_package_search = LaunchConfiguration("enable_oak_package_search")

    # Keep motion disabled by default.
    # Real chassis rotation / scan should only be enabled after TF is correct.
    oak_package_search_enable_motion = LaunchConfiguration("oak_package_search_enable_motion")

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_oak_driver",
            default_value="true",
            description="Start OAK camera driver."
        ),
        DeclareLaunchArgument(
            "enable_oak_parcel_pose",
            default_value="true",
            description="Start OAK DataMatrix + 3D parcel pose node."
        ),
        DeclareLaunchArgument(
            "enable_oak_package_search",
            default_value="true",
            description="Start OAK package search state machine."
        ),
        DeclareLaunchArgument(
            "oak_package_search_enable_motion",
            default_value="false",
            description="Allow oak_package_search to publish /cmd_vel for alignment and scan."
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("oak_vision_driver"),
                    "launch",
                    "oak_vision_driver.launch.py"
                ])
            ),
            condition=IfCondition(enable_oak_driver),
            launch_arguments={
                "enable_rgb": "true",
                "enable_depth": "true",
                "enable_yolo": "false",
                "enable_stereo_fusion": "false",
                "sensor_resolution": "1080p",
                "rgb_output": "video",
                "video_width": "1920",
                "video_height": "1080",
                "fps": "10.0",
                "force_usb2": "true",
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("oak_parcel_pose"),
                    "launch",
                    "oak_parcel_pose.launch.py"
                ])
            ),
            condition=IfCondition(enable_oak_parcel_pose),
        ),

        Node(
            package="oak_package_search",
            executable="oak_package_search_node",
            name="oak_package_search_node",
            output="screen",
            condition=IfCondition(enable_oak_package_search),
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("oak_package_search"),
                    "config",
                    "oak_package_search.yaml"
                ]),
                {
                    "enable_motion": oak_package_search_enable_motion,

                    # Official search policy:
                    # request -> align -> scan start -> scan -> match.
                    "match_only_in_scan": True,
                    "ignore_detection_before_request": True,
                    "min_confirm_matches": 2,
                    "min_search_time_before_match_sec": 0.5,

                    # TF-based OAK direction.
                    "global_frame": "map",
                    "oak_camera_frame": "oak_camera_link",
                    "oak_forward_frame": "oak_camera_forward_link",

                    # Conservative motion parameters.
                    "scan_range_deg": 25.0,
                    "scan_speed_wz": 0.10,
                    "align_yaw_tolerance_deg": 3.0,
                    "kp_wz": 1.0,
                    "min_wz": 0.06,
                    "max_wz": 0.25,
                }
            ]
        ),
    ])
'''

out.write_text(content)
print(f"[OK] Created: {out}")
