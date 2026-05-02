# Lidar Mapping and Navigation Stack README

## 1. 目标

本项目当前的目标是把系统分成三层，并逐步实现：

1. **雷达驱动层**：发布 `/scan`
2. **激光雷达里程计层**：发布 `/odom` 和 `odom -> base_link`
3. **建图 / 导航层**：使用 `slam_toolbox` 和 Nav2

当前推荐的开发顺序是：

- 先确认雷达驱动正常发布 `/scan`
- 再跑通激光雷达里程计
- 再实现在线建图
- 最后再做边建图边导航
- 当前实车先使用轮上方向里程计
- 后续等 IMU yaw 漂移问题解决后，再考虑接入融合增强

---

## 2. 系统架构

### 2.1 在线建图

```text
lslidar_driver        -> /scan
lidar_odom_pkg        -> /odom + odom->base_link
robot_state_publisher -> base_link->laser
slam_toolbox          -> /map + map->odom
rviz2
```

TF 链应当完整为：

```text
map -> odom -> base_link -> laser
```

### 2.2 边建图边导航

```text
lslidar_driver        -> /scan
wheel_twist_odom      -> /wheel/odom
lidar_odom_pkg        -> /lidar/odom
robot_localization    -> /odometry/filtered + odom->base_link
robot_state_publisher -> base_link->laser
slam_toolbox          -> /map + map->odom
Nav2                  -> /cmd_vel
cmd_vel_mode_mux      -> /cmd_vel_selected
rm_serial_driver      -> 串口执行 /cmd_vel_selected
teleop_twist_keyboard -> /cmd_vel_manual
```

其中：

- `Nav2` 默认输出自动控制速度 `/cmd_vel`
- 键盘手动控制输出 `/cmd_vel_manual`
- `cmd_vel_mode_mux` 根据当前模式在手动和自动之间切换
- 串口底盘节点只订阅一个最终控制话题：`/cmd_vel_selected`
- 当前实车推荐使用轮上角速度积分航向，不把 IMU yaw 接进主航向链

---

## 3. 包职责划分

### 3.1 `lslidar_driver`

职责：

- 驱动雷达
- 发布标准 `/scan`

说明：

- 这一层只负责硬件驱动，不负责里程计，不负责建图，不负责导航
- 已经有 `/scan` 后，就不需要再做额外的“转换层”

### 3.2 `lidar_odom_pkg`

职责：

- 订阅 `/scan`
- 做 scan-to-scan 匹配
- 输出 `/odom`
- 输出 `odom -> base_link`

说明：

- 这是当前最关键的一层
- 没有这层时，`slam_toolbox` 很难稳定建图
- 在当前集成导航方案中，`lidar_odom_pkg` 主要负责发布 `/lidar/odom`
- `odom -> base_link` 由 `robot_localization/ekf_node` 根据 `/wheel/odom` 和 `/lidar/odom` 发布

### 3.3 `nav_stack_core`

职责：

- 提供 `robot_state_publisher`
- 提供 `slam_toolbox`
- 提供 Nav2
- 提供 launch 组织与模式切换
- 提供 RViz 入口

---

## 4. 推荐的开发顺序

### 阶段 1：只做激光雷达里程计

目标：

- `/scan` 正常
- `/odom` 正常
- `odom -> base_link` 正常

检查命令：

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

### 阶段 2：只做在线建图

目标：

- `/map` 能正常发布
- `map -> odom` 正常
- RViz 中地图能持续增长

检查命令：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /map --once --qos-durability transient_local --qos-reliability reliable
```

### 阶段 3：边建图边导航

目标：

- Nav2 能发目标点
- 底盘能执行最终的 `/cmd_vel_selected`
- 地图在移动中持续更新

### 阶段 4：评估 IMU 融合

目标：

- 先确认 IMU yaw 漂移是否已经被解决
- 再评估是否值得重新接入航向融合
- 若重新启用，目标仍然是优化走廊 / 白墙 / 静止场景的表现

---

## 5. 运行模式说明

### 5.1 手动建图模式

- 键盘控制小车
- `slam_toolbox` 建图
- Nav2 不接管控制

控制链：

```text
teleop_twist_keyboard -> /cmd_vel_manual
cmd_vel_mode_mux      -> /cmd_vel_selected
rm_serial_driver      -> 底盘执行
```

### 5.2 自动边建图边导航模式

- Nav2 发送导航速度命令
- `slam_toolbox` 一边更新地图
- 小车自动运动

控制链：

```text
Nav2              -> /cmd_vel
cmd_vel_mode_mux  -> /cmd_vel_selected
rm_serial_driver  -> 底盘执行
```

### 5.3 自动模式下人工接管

- 保持 Nav2 在运行
- 把控制模式切到 `manual`
- 手动接管底盘速度控制

---

## 6. 串口底盘控制的推荐设计

串口底盘节点 `rm_serial_driver_node` 应当只做一件事：

- **订阅最终控制话题** `/cmd_vel_selected`
- **通过串口下发到底盘**

不建议：

- 让底盘节点同时兼任键盘控制节点
- 让多个节点直接同时往底盘发速度

推荐统一结构：

```text
/cmd_vel_manual  <- 键盘
/cmd_vel         <- Nav2
/cmd_vel_selected <- 模式选择后的唯一输出
```

---

## 7. 模式切换设计

### 7.1 推荐话题

- `/control_mode`
  - `manual`
  - `auto`

### 7.2 推荐节点

- `cmd_vel_mode_mux.py`

职责：

- 订阅 `/cmd_vel_manual`
- 订阅 `/cmd_vel`
- 订阅 `/control_mode`
- 输出 `/cmd_vel_selected`

### 7.3 切换命令

切到手动模式：

```bash
ros2 topic pub --once /control_mode std_msgs/msg/String "{data: manual}"
```

切到自动模式：

```bash
ros2 topic pub --once /control_mode std_msgs/msg/String "{data: auto}"
```

---

## 8. 推荐 launch 组织

### 8.1 雷达驱动

```bash
ros2 launch lslidar_driver <你的雷达launch文件>
```

### 8.2 激光雷达里程计

```bash
ros2 launch lidar_odom_pkg lidar_odom.launch.py
```

### 8.3 在线建图

```bash
ros2 launch nav_stack_core lidar_mapping_with_lidar_odom.launch.py
```

### 8.4 边建图边导航 + 串口执行 + 模式切换

```bash
ros2 launch nav_stack_core mapping_nav_with_serial_wheel_heading.launch.py
```

说明：

- 当前推荐入口是 `mapping_nav_with_serial_wheel_heading.launch.py`
- 旧的 `mapping_nav_with_serial.launch.py` 仍可保留作 IMU 航向对比，但不作为默认方案

### 8.5 多点导航接口文档

详细说明见：

- [nav_stack_core/docs/multi_waypoint_navigation.md](nav_stack_core/docs/multi_waypoint_navigation.md)

---

## 9. `slam_toolbox` 推荐参数

建议使用如下基础配置：

```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    mode: mapping

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan

    use_map_saver: true
    debug_logging: false
    use_scan_matching: true
    use_scan_barycenter: true

    throttle_scans: 1
    minimum_time_interval: 0.05
    minimum_travel_distance: 0.03
    minimum_travel_heading: 0.03

    transform_publish_period: 0.02
    map_update_interval: 0.2
    resolution: 0.05
    max_laser_range: 20.0
    scan_queue_size: 1
    restamp_tf: true
```

说明：

- `scan_queue_size: 1` 用于降低你之前出现的队列堆积问题
- `map_update_interval: 0.2` 比 `1.0` 更新明显更快
- `minimum_travel_distance` / `minimum_travel_heading` 让插入扫描更合理

---

## 10. `lidar_odom_pkg` 调参建议

如果出现：

- 静止时 odom 还在漂
- 地图墙体重影
- yaw 抖动明显
- 位姿偶尔跳一下

建议先从这些参数入手：

```yaml
min_range: 0.18
max_range: 12.0
downsample_step: 5
icp_max_iterations: 8
icp_max_corr_dist: 0.20
min_correspondences: 30
reject_mean_err_above: 0.08
min_motion_trans: 0.004
min_motion_rot: 0.004
static_lock_trans: 0.006
static_lock_rot: 0.006
reject_trans_above: 0.08
reject_rot_above: 0.12
```

策略：

- 缩短有效量程，减少远点噪声
- 收紧 ICP 对应距离，降低错误匹配概率
- 增加匹配对应点数量要求
- 静止时做零速锁定
- 对大跳变直接拒绝

---

## 11. 当前为什么不用 IMU 航向

当前实车边建图边导航默认使用：

```bash
ros2 launch nav_stack_core mapping_nav_with_serial_wheel_heading.launch.py
```

也就是：

- 轮速负责平移速度
- 轮上角速度负责短时航向积分
- LiDAR odom 负责持续纠偏
- EKF 使用 `/wheel/odom` 和 `/lidar/odom`

当前不把 IMU yaw 接进主航向链的原因是：

- 串口 IMU 的 yaw 在实车上有明显漂移，静止时朝向也可能缓慢变化
- 如果把 IMU yaw 重新接入 `wheel_twist_odom`、EKF 和 lidar odom 先验，漂移会被多层放大
- 实际现象通常是地图慢转、规划路径摆动、原地不动时姿态也在变化
- 对当前底盘而言，轮上角速度的短时一致性比 IMU yaw 更好，更适合作为现阶段航向来源

后续如果满足下面条件，再考虑恢复 IMU 融合：

- IMU 零偏稳定
- yaw 标定与安装方向确认无误
- 静止时朝向不再持续漂移
- 实车测试能证明融合后比轮上方向版更稳

后续推荐路线：

```text
/scan -> lidar_odom_pkg -> /lidar/odom
/wheel/twist -> wheel_twist_odom -> /wheel/odom
/imu/data -> IMU（仅在漂移问题解决后再接回）
wheel odom + lidar odom + imu -> robot_localization(ekf_node)
/odometry/filtered -> 提供更稳的 odom -> base_link
slam_toolbox -> map -> odom
Nav2 -> /cmd_vel
```

说明：

- 激光雷达 odom 负责持续纠偏
- 轮上方向里程计负责当前默认的短时航向与平移先验
- IMU 若后续恢复接入，主要负责补充角速度和短时预测
- 最终推荐用 `robot_localization` 做融合，而不是把所有东西都塞进驱动节点里

---

## 12. 当前结论

当前最推荐的工作流是：

1. 保持雷达驱动不动
2. 跑通 `lidar_odom_pkg`
3. 跑通 `slam_toolbox` 在线建图
4. 串口节点只接收最终 `/cmd_vel_selected`
5. 用模式切换器在 `manual` / `auto` 之间切换
6. 当前默认使用轮上方向里程计，IMU 融合仅作为后续可选增强

一句话总结：

**先把“真实 odom + 在线建图”跑稳，再把底盘执行和模式切换接进去，最后再做边建图边导航。**
