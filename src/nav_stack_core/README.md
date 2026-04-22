# nav_stack_core 操作手册（轮速 + 雷达 + IMU 融合，边建图边导航）

这份手册对应当前工程里已经接好的链路：

- 雷达：`/scan`
- 轮速反馈：`/wheel/twist`
- 底盘 AHRS（来自串口）：`/serial/gimbal_joint_state` -> `/imu/data_serial`
- 融合里程计：`/odometry/filtered`（EKF）
- 控制复用：`/cmd_vel_manual` + `/cmd_vel` -> `/cmd_vel_selected`

默认一键启动入口：`mapping_nav_with_serial.launch.py`。

## 1. 编译与启动

```bash
source /opt/ros/humble/setup.bash
cd /host-home
colcon build --packages-select lslidar_driver rm_serial_driver nav_stack_core
source install/setup.bash

ros2 launch nav_stack_core mapping_nav_with_serial.launch.py
```

这个 launch 会同时拉起：

- `rm_serial_driver`（串口底盘）
- `lslidar_driver`（雷达）
- `scan_match_odom`（雷达里程计）
- `wheel_twist_odom`（轮速积分）
- `serial_imu_bridge`（AHRS -> sensor_msgs/Imu）
- `robot_localization ekf_node`
- `slam_toolbox`（在线建图）
- `Nav2` 全套节点
- `RViz2`

## 2. RViz 显示说明（已配置）

当前 `rviz/nav_stack.rviz` 已经包含以下关键显示：

- `Map`：`/map`
- `Global Path`：`/plan`（全局规划轨迹）
- `Local Path`：`/local_plan`（局部轨迹）
- `Fused Odom`：`/odometry/filtered`（箭头+轨迹，表示车体运动方向）
- `LaserScan`：`/scan`
- `TF` / `RobotModel` / `Grid`

固定坐标系是 `map`。

如果你只想单独打开这套 RViz：

```bash
ros2 run rviz2 rviz2 -d /host-home/src/nav_stack_core/rviz/nav_stack.rviz
```

## 3. 手动控制 / 自动导航

默认模式是 `manual`。

切换手动模式：

```bash
ros2 topic pub /control_mode std_msgs/msg/String "{data: manual}" -1
```

手动发速度（示例：前进）：

```bash
ros2 topic pub /cmd_vel_manual geometry_msgs/msg/Twist \
"{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

停下：

```bash
ros2 topic pub /cmd_vel_manual geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

切换自动模式：

```bash
ros2 topic pub /control_mode std_msgs/msg/String "{data: auto}" -1
```

然后在 RViz 点 `2D Goal Pose` 下发目标点。

- Nav2 会输出 `/plan`、`/local_plan`
- 底盘最终收到的是 `/cmd_vel_selected`

## 4. 必查健康状态（最常用）

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 topic hz /scan
ros2 topic hz /wheel/twist
ros2 topic hz /imu/data_serial
ros2 topic hz /odometry/filtered
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

正常预期：

- `controller_server`、`bt_navigator` 为 `active [3]`
- `/scan`、`/odometry/filtered` 有稳定频率
- TF 链路连通：`map -> odom -> base_link`

## 5. 常见问题快速定位

1. RViz 里 `No map received`
- 先看 `/scan` 是否连续发布。
- 再看 `map -> odom -> base_link` 是否断链。
- 若 TF 断开，`slam_toolbox` 不会稳定出图。

2. 有轨迹但车不动
- 检查当前模式是否 `auto`。
- 查看 `/cmd_vel_selected` 是否有非零速度。
- 查看串口节点日志是否持续打印 `Wheel feedback vx/vy/wz`。

3. `map` 和 `base_link` 不是同一棵树
- 说明 TF 仍有多树问题，优先检查 EKF 输出 `/odometry/filtered` 与 `odom` 链接。

## 6. 保存地图（建图完成后）

```bash
ros2 run nav2_map_server map_saver_cli -f /host-home/maps/my_map
```

会生成：

- `/host-home/maps/my_map.yaml`
- `/host-home/maps/my_map.pgm`
