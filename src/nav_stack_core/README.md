# nav_stack_core README（边建图边导航 + RViz2 显示路径）

本手册目标只有两件事：

- 在 RViz2 里稳定看到规划路径：`/plan`、`/local_plan`
- 车辆按规划路径运动（自动模式）

当前 README 默认以 `mapping_nav_with_serial_wheel_heading.launch.py` 为准。
也就是：当前推荐使用轮上角速度积分航向，不把 `/imu/data` 接进建图导航主链路。

## 0. 先统一工作空间

不要混用 `/host-home` 和 `/workspace`。  
下面示例用 `WS` 表示你的实际工作空间目录（两种二选一）：

- `WS=/host-home`
- `WS=/workspace`

后面每个终端都要执行：

```bash
source /opt/ros/humble/setup.bash
cd $WS
source install/setup.bash
```

## 1. 一键启动（建图 + 导航 + 串口控制）

```bash
source /opt/ros/humble/setup.bash
cd $WS
colcon build --packages-select lslidar_driver lidar_odom_pkg rm_serial_driver nav_stack_core
source install/setup.bash

ros2 launch nav_stack_core mapping_nav_with_serial_wheel_heading.launch.py
```

当前实车推荐入口是 `mapping_nav_with_serial_wheel_heading.launch.py`。
它使用“轮速平移 + 轮上角速度积分航向”，不用 IMU 参与航向融合。

旧入口 `mapping_nav_with_serial.launch.py` 仍然保留，但只建议用于对比或排查 IMU 行为：

```bash
ros2 launch nav_stack_core mapping_nav_with_serial.launch.py
```

该入口会启动：串口底盘、雷达、雷达里程计、EKF、slam_toolbox、Nav2、RViz2。
区别是：

- `mapping_nav_with_serial_wheel_heading.launch.py`：`wheel_twist_odom` 用轮上角速度积分航向，EKF 和 lidar odom 先验也不再吃 `/imu/data`
- `mapping_nav_with_serial.launch.py`：`wheel_twist_odom` 用 IMU 提供航向，EKF 也会融合 `/imu/data`

为什么当前默认不用 IMU 航向：

- 当前串口 IMU 的 yaw 在实车上有明显低频漂移，静止时朝向也会慢慢跑
- 一旦把 IMU yaw 接进 `wheel_twist_odom`、EKF 和 lidar odom 先验，漂移会同时影响三层航向链
- 实际表现通常是地图慢转、路径抖动、原地不动时朝向还在变化
- 轮上角速度虽然也会累计误差，但短时间内更稳定，而且会被 LiDAR odom 持续纠偏

如果雷达串口不是默认的 `/dev/ttyUSB0`，或底盘前进/转向符号和 ROS 坐标不一致，可以直接在 launch 参数里覆盖：

```bash
ros2 launch nav_stack_core mapping_nav_with_serial_wheel_heading.launch.py \
  lidar_serial_port:=/dev/ttyACM0 \
  cmd_vx_scale:=-1.0 \
  wheel_vx_scale:=-1.0 \
  cmd_wz_scale:=-1.0 \
  wheel_wz_scale:=-1.0
```

如果你要对比旧的 IMU 航向版，再用：

```bash
ros2 launch nav_stack_core mapping_nav_with_serial.launch.py \
  lidar_serial_port:=/dev/ttyACM0 \
  cmd_vx_scale:=-1.0 \
  wheel_vx_scale:=-1.0 \
  cmd_wz_scale:=-1.0 \
  wheel_wz_scale:=-1.0 \
  imu_yaw_scale:=-1.0
```

说明：

- 轮上方向版保留了 `imu_yaw_scale`、`imu_yaw_offset_deg` 等 launch 参数兼容旧命令行
- 但该模式下这些 `imu_*` 参数不会参与航向计算

当前 TF 架构固定为：

- `map -> odom`：`slam_toolbox`
- `odom -> base_link`：`robot_localization/ekf_node`
- `base_link -> laser`、`base_link -> imu_link`：`robot_state_publisher`

以下节点都不会发布 `odom -> base_link`：

- `scan_match_odom`
- `wheel_twist_odom`
- `rm_serial_driver`

## 2. 必须先确认 Nav2 已激活

路径显示和导航控制都依赖 lifecycle 处于 active。

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
ros2 action info /navigate_through_poses
```

预期：

- `/controller_server` = `active [3]`
- `/bt_navigator` = `active [3]`
- `/navigate_to_pose` 有 `Action servers: 1`
- `/navigate_through_poses` 有 `Action servers: 1`

如果 `bt_navigator` 是 `inactive`，执行：

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

## 3. RViz2 显示路径（重点）

本工程默认 RViz 配置文件：

- `src/nav_stack_core/rviz/nav_stack.rviz`

默认已包含并启用：

- `Map`：`/map`
- `Global Path`：`/plan`
- `Local Path`：`/local_plan`
- `Goal Marker`：`/goal_markers`

如果你要手动打开 RViz：

```bash
ros2 run rviz2 rviz2 -d $WS/src/nav_stack_core/rviz/nav_stack.rviz
```

RViz 内检查 3 项：

- `Global Options -> Fixed Frame = map`
- `Global Path` 话题是 `/plan`
- `Local Path` 话题是 `/local_plan`

## 4. 自动模式下按路径运动

切换到自动模式：

```bash
ros2 topic pub /control_mode std_msgs/msg/String "{data: auto}" -1
```

在 RViz 使用 `2D Goal Pose` 下发目标（发布到 `/goal_pose`，由 `goal_pose_bridge` 转发到 `/navigate_to_pose`）。

也可命令行发目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

后面如果要做多点导航，当前栈已经可以直接调用 `/navigate_through_poses`：

```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
"{poses: [
  {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
  {header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {z: 0.7071, w: 0.7071}}}
]}"
```

详细接口说明见：

- [docs/multi_waypoint_navigation.md](docs/multi_waypoint_navigation.md)

如果后面要把航点固化到配置文件，参考：

- `src/nav_stack_core/config/waypoints_example.yaml`

建议后续做一个自己的小节点或脚本：读取 YAML 后调用 `/navigate_through_poses`，这样最贴合现在这套建图/导航入口。

检查控制链：

```bash
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel_selected --once
```

至少一个应出现非零速度，车辆才会动。

## 5. 手动/自动切换

手动模式：

```bash
ros2 topic pub /control_mode std_msgs/msg/String "{data: manual}" -1
```

手动速度示例：

```bash
ros2 topic pub /cmd_vel_manual geometry_msgs/msg/Twist \
"{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

## 6. 路径不显示的快速排查

按顺序查：

1. `/bt_navigator` 是否 `active [3]`
2. `/navigate_to_pose` 是否有 `Action servers: 1`
3. `/plan` 是否有数据：

```bash
ros2 topic echo /plan --once
ros2 topic echo /local_plan --once
```

4. TF 是否连通：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

5. 若仍看不到路径，确认 RViz 不是默认空布局，重新用 `-d $WS/src/nav_stack_core/rviz/nav_stack.rviz` 打开。

## 7. 说明

- 日志中 `LaserRangeScan contains xxx range readings, expected yyy` 为雷达点数波动告警，通常不会直接导致路径完全不显示。
- 当前控制参数已偏向“先跑通平移控制”，并放宽了朝向约束，便于 `vx/vy` 主控底盘先稳定导航。
- 当前全局/局部代价地图参数也偏调试友好：机器人半径和膨胀半径较小，目的是先确认 `/plan -> /cmd_vel_nav -> /cmd_vel_selected` 链路能跑通；实车稳定后再按车体尺寸调大。

## 8. 定位与 TF 验证

启动后先执行下面这些命令，逐项核对传感器输入、融合输出和 TF 归属：

```bash
ros2 topic echo /wheel/odom --once
ros2 topic echo /lidar/odom --once
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /tf | grep -A 8 "child_frame_id: base_link"
```

如果你是在对比旧的 IMU 航向版，再额外检查：

```bash
ros2 topic echo /imu/data --once
```

验收时重点看：

- `/tf` 中只保留一条 `odom -> base_link`，来源应是 `ekf_node`
- `base_link -> laser` 和 `base_link -> imu_link` 应是 fixed TF，零旋转对应四元数 `0 0 0 1`
- 车前进时 `/wheel/odom.twist.twist.linear.x` 不应为 `0`
- 轮上方向版左转时，`/wheel/odom.pose.pose.orientation` 与 `/odometry/filtered.pose.pose.orientation` 应连续变化
- 旧的 IMU 航向版左转时，`/imu/data.angular_velocity.z` 应为正
- `/lidar/odom` 只发话题，不发 `odom -> base_link` TF
- Nav2 不再出现 `map/base_link transform missing`
