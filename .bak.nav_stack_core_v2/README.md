# Robot2 D435 + 激光雷达 SLAM + Nav2 启动教程

本教程用于启动 Robot2 实车导航系统，包括：

- 激光雷达建图与定位
- RF2O 激光里程计
- slam_toolbox 在线 SLAM
- Nav2 路径规划与控制
- D435 深度相机辅助局部避障
- `/scan_fused` 融合障碍物输入
- 串口底盘控制
- RViz2 可视化

当前稳定结构如下：

```text
/map
  ↓  slam_toolbox 发布 map -> odom
/odom
  ↓  rf2o_odom_adapter 发布 odom -> base_link
/base_link
  ├── SCAN-Link
  ├── cam1_Link / camera_link
  └── wheels
````

---

## 1. 系统结构

### 1.1 SLAM 与定位

```text
原始激光雷达：
/scan

前向过滤雷达：
/scan -> front_scan_filter -> /scan_front

RF2O：
/scan -> /odom_laser

里程计适配：
/odom_laser -> /odom
/odom_laser -> TF: odom -> base_link

SLAM：
/scan_front -> slam_toolbox
slam_toolbox -> /map
slam_toolbox -> TF: map -> odom
```

注意：

```text
RF2O 使用 /scan
slam_toolbox 使用 /scan_front
不要再让 RF2O 发布 TF
不要让多个节点同时发布 odom -> base_link
```

---

### 1.2 D435 深度相机辅助避障

```text
D435 深度图
  ↓
d435_depth_to_scan
  ↓
/d435_scan
  ↓
scan_fusion_node
  ↓
/scan_fused
  ↓
Nav2 local_costmap
```

D435 当前只参与 **局部避障**，不参与 SLAM 建图。

正确结构是：

```text
SLAM：/scan_front
Global Costmap：/map + /scan_front
Local Costmap：/scan_fused
```

---

### 1.3 Nav2 规划结构

当前规划器：

```text
Global Planner：SmacPlanner2D
Local Controller：DWBLocalPlanner
```

当前 costmap 输入：

```text
global_costmap:
  scan topic: /scan_front

local_costmap:
  scan topic: /scan_fused
```

---

## 2. 启动前检查

进入工作空间：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0
```

检查设备：

```bash
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*
```

一般情况下：

```text
激光雷达：/dev/ttyUSB0
底盘串口：/dev/ttyACM0
```

如果设备号变化，需要在启动命令中修改：

```bash
lidar_serial_port:=/dev/ttyUSB0
serial_port:=/dev/ttyACM0
```

---

## 3. 编译

每次修改 launch、config、Python 节点后，执行：

```bash
cd /host-home
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select nav_stack_core_v2

source install/setup.bash
```

如果出现 install 或 build 缓存问题，可以清理单个包：

```bash
rm -rf /host-home/build/nav_stack_core_v2 /host-home/install/nav_stack_core_v2

cd /host-home
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select nav_stack_core_v2
source install/setup.bash
```

---

## 4. 稳定版启动：激光 SLAM + Nav2 + D435 局部避障

这是当前推荐启动命令：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch nav_stack_core_v2 robot2_d435_fused_slam_semantic.launch.py \
  start_lidar:=true \
  start_serial:=true \
  start_robot_state_publisher:=true \
  start_realsense:=true \
  start_d435_scan:=true \
  start_scan_fusion:=true \
  start_nav2:=true \
  start_semantic:=false \
  robot_description_file:=/host-home/src/Diverse/robot2/urdf/robot2.urdf \
  lidar_serial_port:=/dev/ttyUSB0 \
  serial_port:=/dev/ttyACM0 \
  serial_cmd_vel_topic:=/cmd_vel_selected \
  raw_scan_topic:=/scan \
  nav_scan_topic:=/scan_front \
  d435_scan_topic:=/d435_scan \
  fused_scan_topic:=/scan_fused \
  slam_scan_topic:=/scan_front \
  use_rviz:=true
```

含义：

```text
start_lidar:=true
启动激光雷达。

start_serial:=true
启动底盘串口控制。

start_robot_state_publisher:=true
加载 robot2.urdf，发布机器人 TF。

start_realsense:=true
启动 D435 相机。

start_d435_scan:=true
把 D435 depth 转成 /d435_scan。

start_scan_fusion:=true
把 /scan_front 和 /d435_scan 融合成 /scan_fused。

start_nav2:=true
启动 Nav2。

start_semantic:=false
暂时不启动语义货架系统。

raw_scan_topic:=/scan
原始激光雷达话题。

nav_scan_topic:=/scan_front
过滤后的导航雷达话题。

slam_scan_topic:=/scan_front
slam_toolbox 使用 /scan_front 建图。

fused_scan_topic:=/scan_fused
D435 + 雷达融合后的局部避障 scan。
```

---

## 5. 纯激光导航启动

如果 D435 有问题，或只想测试基础导航，用这个版本：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch nav_stack_core_v2 robot2_d435_fused_slam_semantic.launch.py \
  start_lidar:=true \
  start_serial:=true \
  start_robot_state_publisher:=true \
  start_realsense:=false \
  start_d435_scan:=false \
  start_scan_fusion:=false \
  start_nav2:=true \
  start_semantic:=false \
  robot_description_file:=/host-home/src/Diverse/robot2/urdf/robot2.urdf \
  lidar_serial_port:=/dev/ttyUSB0 \
  serial_port:=/dev/ttyACM0 \
  serial_cmd_vel_topic:=/cmd_vel_selected \
  raw_scan_topic:=/scan \
  nav_scan_topic:=/scan_front \
  d435_scan_topic:=/d435_scan \
  fused_scan_topic:=/scan_fused \
  slam_scan_topic:=/scan_front \
  use_rviz:=true
```

这个版本不启动 D435，Nav2 主要使用 `/scan_front`。

---

## 6. 纯 SLAM 测试启动

如果需要单独确认 map 是否正常，用这个版本：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch nav_stack_core_v2 robot2_d435_fused_slam_semantic.launch.py \
  start_lidar:=true \
  start_serial:=false \
  start_robot_state_publisher:=true \
  start_realsense:=false \
  start_d435_scan:=false \
  start_scan_fusion:=false \
  start_nav2:=false \
  start_semantic:=false \
  robot_description_file:=/host-home/src/Diverse/robot2/urdf/robot2.urdf \
  lidar_serial_port:=/dev/ttyUSB0 \
  raw_scan_topic:=/scan \
  nav_scan_topic:=/scan_front \
  slam_scan_topic:=/scan_front \
  use_rviz:=true
```

这个模式用于确认：

```text
/scan 是否正常
/scan_front 是否正常
/odom 是否正常
map -> odom -> base_link 是否正常
/map 是否正常发布
```

---

## 7. 启动后检查

另开一个终端：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0
```

### 7.1 检查基础话题频率

```bash
ros2 topic hz /scan
ros2 topic hz /scan_front
ros2 topic hz /odom_laser
ros2 topic hz /odom
```

正常情况：

```text
/scan        有频率
/scan_front  约 12Hz
/odom_laser  有频率
/odom        有频率
```

---

### 7.2 检查 SLAM 和 TF

```bash
ros2 topic echo /map --once

ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

正常 TF 链：

```text
map -> odom -> base_link -> SCAN-Link
```

---

### 7.3 检查 Nav2 生命周期

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

正常应为：

```text
active [3]
```

---

### 7.4 检查 D435 是否参与局部避障

```bash
ros2 topic hz /d435_scan
ros2 topic hz /scan_fused
ros2 topic info -v /scan_fused
```

正确状态：

```text
/d435_scan 约 15Hz
/scan_fused 有频率，约 8Hz 左右
/scan_fused 有 publisher: scan_fusion_node
/scan_fused 有 subscriber: /local_costmap/local_costmap
```

如果看到：

```text
Node name: local_costmap
Node namespace: /local_costmap
```

说明 D435 已经正式参与 Nav2 局部避障。

---

### 7.5 检查 SmacPlanner2D 是否生效

```bash
ros2 param get /planner_server GridBased.plugin
```

正确输出：

```text
String value is: nav2_smac_planner/SmacPlanner2D
```

也可以检查配置文件：

```bash
grep -n "planner_server\|planner_plugins\|GridBased\|SmacPlanner2D" \
/host-home/src/Navigation/nav_stack_core_v2/config/nav2_params_min.yaml -A20 -B5
```

---

### 7.6 发目标后检查路径和速度

在 RViz2 里点击 `2D Goal Pose` 后，执行：

```bash
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel_selected --once
```

正常情况：

```text
/plan 有路径
/cmd_vel_selected 有速度输出
```

如果 `/plan` 有，但车不动，检查串口和下位机。

如果 `/cmd_vel` 有但 `/cmd_vel_selected` 没有，检查 `cmd_vel_mode_mux`。

---

## 8. RViz2 推荐显示项

建议在 RViz2 中添加：

```text
TF

RobotModel

Map
Topic: /map

Map
Topic: /global_costmap/costmap

Map
Topic: /local_costmap/costmap

LaserScan
Topic: /scan_front

LaserScan
Topic: /d435_scan

LaserScan
Topic: /scan_fused

Path
Topic: /plan

Polygon
Topic: /local_costmap/published_footprint

Polygon
Topic: /global_costmap/published_footprint
```

其中最重要的是：

```text
/map
/scan_front
/scan_fused
/local_costmap/costmap
/plan
/local_costmap/published_footprint
```

---

## 9. 当前 Nav2 参数结构

当前 `nav2_params_min.yaml` 关键配置如下：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: nav2_smac_planner/SmacPlanner2D
      tolerance: 0.80
      allow_unknown: true
      max_planning_time: 2.0
```

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      footprint: "[[0.60, 0.50], [0.60, -0.50], [-0.60, -0.50], [-0.60, 0.50]]"
      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        observation_sources: scan
        scan:
          topic: /scan_fused
```

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      footprint: "[[0.60, 0.50], [0.60, -0.50], [-0.60, -0.50], [-0.60, 0.50]]"
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      obstacle_layer:
        observation_sources: scan
        scan:
          topic: /scan_front
```

---

## 10. 车体安全范围

当前 footprint：

```yaml
footprint: "[[0.60, 0.50], [0.60, -0.50], [-0.60, -0.50], [-0.60, 0.50]]"
```

含义：

```text
以 base_link 为中心
前方 0.60 m
后方 0.60 m
左侧 0.50 m
右侧 0.50 m

车体尺寸约为：
长 1.20 m
宽 1.00 m
```

当前 inflation：

```yaml
inflation_radius: 0.20
cost_scaling_factor: 5.0
```

如果需要更安全，可以改为：

```yaml
inflation_radius: 0.35
cost_scaling_factor: 3.0
```

---

## 11. 运动规则

当前底盘运动设计：

```text
远距离导航：
Nav2 负责路径规划和避障。

近距离靠货架：
不要完全依赖 Nav2 精对齐。
应该进入视觉/语义闭环控制。

推荐动作顺序：
1. 原地旋转对正
2. 左右平移对齐
3. 缓慢前进
4. 停车取件
```

不建议：

```text
高速斜走
一边横移一边大角速度旋转
一边前进一边快速旋转
```

---

## 12. 语义货架配合方式

语义货架系统不直接修改 `/map`，也不直接干预 SLAM。

正确流程：

```text
1. 识别货架 tag / QR
2. 获得货架相对于 camera_link 或 base_link 的位置
3. 使用 TF 转换到 map 坐标系
4. 保存 shelf pose in map
5. 根据 shelf pose 生成 approach pose
6. 调用 Nav2 NavigateToPose
7. 到达货架前方后，切换视觉闭环对齐
```

语义货架必须保存为：

```text
frame_id: map
```

不要保存为：

```text
base_link
odom
camera_link
```

因为：

```text
base_link 会随车移动
odom 会被 SLAM 修正
camera_link 是相机局部坐标
map 才是全局语义坐标
```

---

## 13. 货架 approach pose

不要导航到货架中心。应该导航到货架前方一定距离：

```text
approach_distance = 0.8 ~ 1.2 m
```

计算方式：

```text
approach_x = shelf_x - approach_distance * cos(shelf_yaw)
approach_y = shelf_y - approach_distance * sin(shelf_yaw)
approach_yaw = shelf_yaw
```

推荐先用：

```text
approach_distance = 1.0 m
```

---

## 14. 常见问题

### 14.1 map 不显示

检查：

```bash
ros2 topic hz /scan
ros2 topic hz /scan_front
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /map --once
```

重点确认：

```text
rf2o_odom_adapter 必须发布 odom -> base_link
slam_toolbox 必须发布 map -> odom
```

---

### 14.2 D435 没有参与导航

检查：

```bash
ros2 topic info -v /scan_fused
```

如果：

```text
Subscription count: 0
```

说明 Nav2 没有订阅 `/scan_fused`。

正确应该有：

```text
Node name: local_costmap
Node namespace: /local_costmap
```

---

### 14.3 有路径但车不动

检查：

```bash
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /cmd_vel_selected --once
```

判断：

```text
/plan 有，/cmd_vel 没有：
controller_server 问题。

/cmd_vel 有，/cmd_vel_selected 没有：
cmd_vel_mode_mux 问题。

/cmd_vel_selected 有，车不动：
串口或下位机问题。
```

---

### 14.4 路径规划失败

检查：

```bash
ros2 topic echo /rosout | grep -E "planner_server|GridBased|Smac|No valid|failed|occupied|unknown|transform"
```

常见原因：

```text
目标点在障碍物中
目标点在未知区
footprint 太大
inflation 太大
map / TF 没有稳定
```

---

### 14.5 车体旋转漂移

检查：

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

静止时：

```text
x/y/yaw 不应该持续漂移
```

如果静止时 yaw 一直变化，优先检查：

```text
雷达安装角度
SCAN-Link TF
RF2O 输入 scan
雷达数据是否抖动
```

---

## 15. 最终定版结构

```text
定位建图：
/scan -> RF2O -> /odom_laser -> /odom -> odom->base_link
/scan -> front_scan_filter -> /scan_front -> slam_toolbox -> /map + map->odom

导航规划：
SmacPlanner2D -> /plan
DWBLocalPlanner -> /cmd_vel

底盘控制：
/cmd_vel -> cmd_vel_mode_mux -> /cmd_vel_selected -> rm_serial_driver -> 下位机

局部避障：
D435 -> /d435_scan
/scan_front + /d435_scan -> /scan_fused
/scan_fused -> local_costmap

全局地图：
/map + /scan_front -> global_costmap

语义货架：
tag/QR -> map frame shelf pose -> approach pose -> Nav2