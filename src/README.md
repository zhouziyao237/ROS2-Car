# ROS2-Car src 统一启动说明

本目录是 ROS2-Car 的主要 ROS2 工作区。当前推荐的启动方式已经从早期的“按阶段手动拼 launch”切到 `robot2_bringup` 的 YAML 统一启动器。

主入口：

```bash
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task
```

旧的 `nav_stack_core` / 单包 launch 仍可用于调试，但日常实车联调优先使用 `robot2_bringup`。

## 1. 编译与环境

在容器或 ROS2 Humble 环境中：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /host-home/install/setup.bash
```

如果只改了启动配置或 Python 节点，通常重新 source 后即可；如果改了包结构、入口点或 C++ 节点，重新 `colcon build`。

## 2. 统一启动入口

统一启动器文件：

```text
src/Navigation/robot2_bringup/launch/yaml_bringup.launch.py
```

统一配置文件：

```text
src/Navigation/robot2_bringup/config/bringup_modes.yaml
```

常用命令：

```bash
# 完整任务链：导航 + 语义货架 + OAK 目标检测/取件 + 任务行为树
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task

# 只启动底层硬件：URDF + 关节状态 + 雷达 + 底盘串口
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=hardware

# 建图链路：雷达 + 前向扫描过滤 + rf2o + slam_toolbox
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=mapping

# 导航链路：建图/定位 + Nav2 + goal bridge + 底盘串口
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=nav

# D435 辅助导航链路
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=nav_d435

# 只开 RViz
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=debug
```

## 3. 启动模式说明

| mode | 用途 | 启动内容 |
| --- | --- | --- |
| `hardware` | 最小硬件检查 | `robot_description`、`joint_state_publisher`、`lidar`、`chassis_serial` |
| `mapping` | 激光建图 | 硬件描述、雷达、`front_scan_filter`、`rf2o`、`rf2o_odom_adapter`、`cmd_vel_mux`、`slam` |
| `nav` | 基础导航 | `mapping` 相关模块 + `nav2`、`goal_pose_bridge`、`chassis_serial` |
| `nav_d435` | D435 辅助导航 | `nav` 相关模块 + `realsense`、`d435_depth_to_scan` |
| `task` | 当前完整任务入口 | 导航、底盘、D435、扫描融合、语义货架、任务行为树、OAK 检测/取件全链路 |
| `debug` | 可视化调试 | 只启动 `rviz` |

当前完整任务推荐直接跑：

```bash
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task
```

## 4. task 模式的主要链路

`task` 模式会从 `bringup_modes.yaml` 组装以下模块：

1. 机器人基础：`robot_description`、`joint_state_publisher`
2. 雷达与里程计：`lidar`、`front_scan_filter`、`rf2o`、`rf2o_odom_adapter`
3. 控制输出：`cmd_vel_mux`、`chassis_serial`
4. D435 辅助感知：`realsense`、`d435_depth_to_scan`、`scan_fusion`
5. 建图导航：`slam`、`nav2`、`goal_pose_bridge`
6. 语义货架：`apriltag_shelf_bridge`、`semantic_registrar`、`home_anchor_validator`、`semantic_marker_publisher`
7. 任务执行：`mission_bt`、`task_cancel_guard`、`bt_osd_overlay`
8. OAK 目标链路：`oak_vision_driver`、`oak_parcel_pose`、`oak_package_search`、`oak_package_servo`、`oak_guided_pickup`、`oak_package_grabber`

其中 `nav2`、`goal_pose_bridge`、部分语义模块会等待：

```text
/target_capture/oak_guided_pickup/startup_zero_done
```

这个 gating 用来避免机器人还没完成取件链路初始化时，导航和任务节点过早接管。

## 5. 常用覆盖参数

统一启动器支持临时启用或关闭模块：

```bash
# 启动 hardware，但不接雷达和底盘串口
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=hardware disable_modules:=lidar,chassis_serial

# 启动 task，但暂时不接底盘串口
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task disable_modules:=chassis_serial

# 在某个 mode 基础上额外启用模块
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=nav enable_modules:=rviz

# 使用自定义 bringup 配置
ros2 launch robot2_bringup yaml_bringup.launch.py mode:=task config_file:=/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml
```

`disable_modules` 和 `enable_modules` 都是逗号分隔列表，模块名必须来自 `bringup_modes.yaml` 的 `modules` 段。

## 6. 关键话题检查

启动后先检查基础链路：

```bash
ros2 topic list | grep -E '/scan|/scan_front|/odom|/cmd_vel_selected'
ros2 topic echo /scan --once
ros2 topic echo /scan_front --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

检查导航/任务链路：

```bash
ros2 topic echo /target_capture/oak_guided_pickup/startup_zero_done --once
ros2 topic echo /vision/semantic_observation --once
ros2 topic echo /semantic/current_shelf --once
ros2 topic echo /mission/state --once
ros2 topic echo /cmd_vel_selected --once
```

如果 `nav2` 或语义模块迟迟不启动，优先确认 `/target_capture/oak_guided_pickup/startup_zero_done` 是否已经发布。

## 7. 如何新增启动模块

推荐只改 YAML，不直接改 `yaml_bringup.launch.py`。

1. 在 `src/Navigation/robot2_bringup/config/bringup_modes.yaml` 的 `modules` 下新增模块。
2. 在需要的 `modes.<mode>.enable` 列表里加入模块名。
3. 用 `disable_modules` / `enable_modules` 做临时验证。

模块可以是 ROS 节点：

```yaml
my_node:
  type: node
  package: my_package
  executable: my_node
  name: my_node
  delay: 2.0
  respawn: true
  respawn_delay: 2.0
  parameters:
    - use_sim_time: false
```

也可以 include 另一个 launch：

```yaml
my_launch:
  type: include
  package: my_package
  launch: my_launch.launch.py
  delay: 3.0
  arguments:
    use_sim_time: 'false'
```

## 8. 目标抓取链路补充

如果只调试取件链路，可以单独使用：

```bash
ros2 launch target_pickup_bringup full_pickup_pipeline.launch.py
ros2 launch target_pickup_bringup qr_pickup_pipeline.launch.py
ros2 launch target_pickup_bringup yolo_debug_pipeline.launch.py
```

`parcel_pickup_mission` 里还有可切换相机模式的取件系统：

```bash
ros2 launch parcel_pickup_mission pickup_system_switchable.launch.py camera_mode:=single_oak start_serial:=false
ros2 launch parcel_pickup_mission pickup_system_switchable.launch.py camera_mode:=usb_wide_oak_pick wide_video_device:=/dev/video0 start_serial:=false
```

这些入口适合单独调试 OAK、二维码匹配、升降平台、底盘视觉对齐和取件状态机；完整实车流程仍建议回到 `robot2_bringup mode:=task`。

## 9. 目录速览

```text
src/
├── Diverse/              # 雷达、相机、DepthAI/OAK、串口、robot2 描述等底层包
├── Msg/                  # 自定义消息包
├── Navigation/           # robot2_bringup、nav_stack_core_v2、语义货架、任务行为树
├── target_detection/     # OAK/YOLO/目标搜索/伺服/姿态桥接
└── target_capture/       # 取件 bringup、二维码匹配、升降平台、底盘对齐、任务状态机
```

当前维护重点：

```text
Navigation/robot2_bringup/launch/yaml_bringup.launch.py
Navigation/robot2_bringup/config/bringup_modes.yaml
```

旧的单独 launch 文件可以保留作为调试入口，但 README 以后应以统一启动器为准。
