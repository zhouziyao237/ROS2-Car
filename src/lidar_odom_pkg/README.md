# lidar_odom_pkg

ROS2 Python package for 2D LiDAR-only odometry. It is designed as a single-LiDAR odometry source before later fusion with IMU/wheel odometry/Nav2.

## Features

- Configurable LaserScan valid angle windows, e.g. front-only `[-95, 95]`, wide front `[-135, 135]`, or full scan `[-180, 180]`.
- Configurable LiDAR mounting direction and planar offset relative to `base_link`.
- Constant-velocity Kalman prediction before ICP. The prediction is used as the initial SE(2) guess for scan matching.
- Trimmed point-to-point ICP between consecutive scans.
- Observability-aware and geometry-aware constraints to suppress unreliable lateral/yaw estimates when the scan is front-heavy or one-sided.
- Dynamic covariance in `nav_msgs/Odometry` for later fusion.

## Build

```bash
cd ~/ros2_ws/src
cp -r /path/to/lidar_odom_pkg .
cd ~/ros2_ws
colcon build --packages-select lidar_odom_pkg
source install/setup.bash
```

## Run

```bash
ros2 launch lidar_odom_pkg lidar_odom.launch.py
```

Or with a custom config:

```bash
ros2 launch lidar_odom_pkg lidar_odom.launch.py params_file:=/absolute/path/to/lidar_odom.yaml
```

## Important parameters

```yaml
scan_topic: /scan_front
odom_topic: /lidar/odom
odom_frame: odom
base_frame: base_link
publish_tf: false

use_angle_filter: true
angle_windows_deg: [-95.0, 95.0]
scan_yaw_offset_deg: 0.0
scan_x_offset: 0.0
scan_y_offset: 0.0

use_kalman_predict: true
use_kalman_update: true
icp_max_corr_dist: 0.12
reject_mean_err_above: 0.06
reject_trans_above: 0.12
reject_rot_above: 0.18
```

## Suggested tuning

1. Start with `publish_tf: false` if Nav2/SLAM/robot_localization already publishes `odom -> base_link`.
2. If the LiDAR is mounted sideways, set `scan_yaw_offset_deg`, for example `90.0` or `-90.0`.
3. If the LiDAR only sees the front, use `angle_windows_deg: [-80, 80]` or `[-95, 95]`. If the LiDAR is 270-degree and has useful side walls, use `[-135, 135]`.
4. If odometry jumps, decrease `reject_trans_above`, `reject_rot_above`, and `icp_max_corr_dist`.
5. If matching often fails during faster movement, increase `icp_max_corr_dist` slightly and keep `use_kalman_predict: true`.
6. If lateral drift is large with a front-only LiDAR, keep `use_scan_geometry_gate: true` and reduce `alpha_y_max` or increase `min_cov_y`.

## Notes

This is local odometry, not global SLAM. It will still drift over time because it only aligns consecutive scans. The intended downstream design is to fuse this `/lidar/odom` with IMU/wheel odometry or use it as a local odometry source for SLAM/Nav2.


## Fast-motion update

This version adds external motion prior and scan deskew for fast motion.

New parameters:

```yaml
use_external_twist_prior: true
prior_source: auto        # auto / wheel_imu / wheel / cmd_vel / internal
external_odom_topic: /wheel/odom
external_imu_topic: /imu/data
external_cmd_vel_topic: /cmd_vel
external_prior_weight: 0.90

use_scan_deskew: true
deskew_center_scan: true
deskew_max_time: 0.12
```

Recommended real-car setup:

- `scan_match_odom` publishes `/lidar/odom` only.
- Set `publish_tf: false` when `robot_localization` publishes `odom -> base_link`.
- Use `prior_source: auto` after `/wheel/odom` and `/imu/data` are valid.
- Use `prior_source: cmd_vel` only for Gazebo validation or when feedback is not available yet.
