# m10p_lidar_odom_bringup

This is a **bringup / verification package**. It does not copy the LiDAR odometry algorithm. Instead, it launches the previous package `lidar_odom_pkg` and provides Gazebo Classic + RViz2 simulation assets for an M10P-like 360-degree 2D LiDAR.

## Why this package exists

The referenced implementation flow is:

1. subscribe to `LaserScan`,
2. preprocess valid ranges and scan angles,
3. use a constant-velocity prediction as the ICP first guess,
4. run scan matching,
5. publish `nav_msgs/Odometry` and TF.

This bringup package validates that same pipeline in simulation before moving to the real robot.

## Package relationship

```text
~/ros2_ws/src/
├── lidar_odom_pkg/              # previous algorithm package, provides scan_match_odom
└── m10p_lidar_odom_bringup/     # this package, calls lidar_odom_pkg and starts simulation
```

The launch file starts the odometry node like this:

```python
Node(
    package='lidar_odom_pkg',
    executable='scan_match_odom',
    name='scan_match_odom',
    parameters=[params_file],
)
```

## Simulated LiDAR model

The URDF uses an M10P-like single-line 2D LiDAR:

- horizontal FOV: 360 deg
- update rate: 10 Hz
- range: 0.18 m to 12 m in this indoor verification world
- simulated beams: 720 samples per scan
- scan topic: `/scan`
- laser frame: `m10p_laser_link`

The real M10P has higher point rate than this lightweight simulation. The reduced beam count keeps Gazebo/RViz responsive while preserving the 360-degree scan geometry needed by the odometry algorithm.

## Dependencies

For ROS2 Humble + Gazebo Classic on Ubuntu 22.04:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  python3-numpy
```

You also need the previous package:

```bash
# should exist in the same workspace
~/ros2_ws/src/lidar_odom_pkg
```

## Build

```bash
cd ~/ros2_ws/src
unzip /path/to/m10p_lidar_odom_bringup.zip
cd ~/ros2_ws
colcon build --packages-select lidar_odom_pkg m10p_lidar_odom_bringup
source install/setup.bash
```

## Run Gazebo + RViz2 verification

```bash
ros2 launch m10p_lidar_odom_bringup m10p_gazebo_rviz_verify.launch.py
```

Optional arguments:

```bash
# no RViz, useful on remote server
ros2 launch m10p_lidar_odom_bringup m10p_gazebo_rviz_verify.launch.py use_rviz:=false

# no auto driving; control manually
ros2 launch m10p_lidar_odom_bringup m10p_gazebo_rviz_verify.launch.py auto_drive:=false

# headless Gazebo Classic
ros2 launch m10p_lidar_odom_bringup m10p_gazebo_rviz_verify.launch.py gui:=false use_rviz:=false
```

## What you should see

Topics:

```bash
ros2 topic hz /scan
ros2 topic hz /lidar/odom
ros2 topic echo /lidar/odom_debug --once
ros2 topic echo /lidar/odom_error
```

Frames:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link m10p_laser_link
```

RViz2:

- Fixed Frame: `odom`
- `/scan`: LiDAR scan around `m10p_laser_link`
- `/lidar/path`: path estimated by `lidar_odom_pkg`
- `/truth/path`: Gazebo diff-drive odometry reference

The node `odom_error_monitor` logs an approximate comparison between `/lidar/odom` and `/odom_truth`:

```text
odom error: ex=+0.012 m, ey=-0.008 m, trans=0.014 m, yaw=+0.50 deg
```

Do not treat `/odom_truth` as perfect ground truth; it is the Gazebo diff-drive plugin odometry. It is still useful for checking whether the LiDAR odom is basically following the simulated robot trajectory.

## Main tuning file

Edit:

```text
config/m10p_lidar_odom_sim.yaml
```

Important parameters:

```yaml
scan_topic: /scan
odom_topic: /lidar/odom
publish_tf: true
angle_windows_deg: [-180.0, 180.0]
scan_yaw_offset_deg: 0.0
scan_x_offset: 0.12
scan_y_offset: 0.0
icp_max_corr_dist: 0.18
reject_mean_err_above: 0.08
reject_trans_above: 0.18
reject_rot_above: 0.22
```

For front-sector testing:

```yaml
angle_windows_deg: [-95.0, 95.0]
```

For a side-mounted real LiDAR:

```yaml
scan_yaw_offset_deg: 90.0
```

## Notes for your real robot

- In this simulation, `publish_tf: true` is used so RViz can see `odom -> base_link` from the LiDAR odometry.
- On the real robot, if Nav2, `robot_localization`, or SLAM already publishes `odom -> base_link`, set `publish_tf: false` in the LiDAR odometry config to avoid duplicate TF.
- Empty or highly symmetric scenes are bad for ICP. The world intentionally includes asymmetric boxes and an oblique wall to make scan matching observable.
