# nav_stack_core

一个把 SLAM、Nav2、waypoint、里程计模式切换集中到一起的 ROS 2 Humble 集成包。

这版额外内置了一个最小可用的 xacro：

- `description/minimal_lidar_robot.urdf.xacro`

适合你现在这种场景：

- 激光雷达 frame = `laser`
- LaserScan topic = `/scan`
- 先只跑 2D 激光雷达建图
- 机器人本体模型暂时没有现成 urdf/xacro

## 1. 直接只跑雷达建图

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav_stack_core
source install/setup.bash

ros2 launch nav_stack_core lidar_mapping.launch.py
```

这个 launch 会启动：

- `robot_state_publisher`
- `slam_toolbox` 在线建图
- `rviz2`

默认使用内置 xacro：

- `base_link`
- `laser`
- `camera_link`

其中 `laser` 是通过固定关节挂在 `base_link` 上。

## 2. 调整激光雷达安装位姿

如果你的雷达不是安装在车体中心，可以直接传 xacro 参数：

```bash
ros2 launch nav_stack_core lidar_mapping.launch.py \
  xacro_args:="laser_x:=0.12 laser_y:=0.0 laser_z:=0.22 laser_yaw:=0.0"
```

## 3. 用你自己的 xacro 替换

之后你有正式机器人模型时，可以继续复用这个包，只需要换掉路径：

```bash
ros2 launch nav_stack_core lidar_mapping.launch.py \
  xacro_path:=/absolute/path/to/your_robot.urdf.xacro
```

## 4. 跑完整导航系统

```bash
ros2 launch nav_stack_core navigation_system.launch.py \
  xacro_path:=/absolute/path/to/your_robot.urdf.xacro \
  use_slam:=true \
  odom_mode:=lidar
```

如果你暂时没有自己的 xacro，也可以直接用内置默认模型：

```bash
ros2 launch nav_stack_core navigation_system.launch.py \
  use_slam:=true \
  odom_mode:=lidar
```

## 5. 现在这版要满足的前提

为了让 `slam_toolbox` 正常建图，你至少还需要下面这些条件成立：

- `/scan` 持续发布 `sensor_msgs/LaserScan`
- scan 消息的 frame_id 是 `laser`
- 系统里存在 `odom -> base_link` 变换，或者有等价的里程计链路
- `base_link -> laser` 由内置 xacro 提供

如果你只有 `laser` 和 `/scan`，但还没有 `odom -> base_link`，那建图通常会不稳定，甚至直接报 TF 问题。

## 6. 快速检查

```bash
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo odom base_link
```
