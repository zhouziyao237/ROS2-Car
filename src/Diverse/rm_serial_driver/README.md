# rm_serial_driver

RoboMaster 视觉系统与下位机之间的 ROS 2 串口通信节点。

<img src="docs/rm_vision.svg" alt="rm_vision" width="200" height="200">

这个包基于 [transport_drivers](https://github.com/ros-drivers/transport_drivers)，用于把 `rm_auto_aim`、底盘控制和机械臂控制的数据通过串口与电控板互通。

## 功能概览

- 接收下位机回传的云台姿态、底盘状态、机械臂状态
- 向下位机发送自瞄目标解算结果
- 向下位机发送底盘速度指令
- 向下位机发送机械臂目标位置指令
- 提供键盘遥控入口，可同时控制底盘和机械臂目标
- 发布可视化和调试用话题

## 启动

安装依赖：

```bash
sudo apt install ros-humble-serial-driver
```

启动串口节点：

```bash
ros2 launch rm_serial_driver serial_driver.launch.py
```

串口参数默认配置见 [config/serial_driver.yaml](config/serial_driver.yaml)。

## 键盘控制

如果你想像控制底盘一样，直接用键盘控制升降机构和机械臂目标，可以启动：

```bash
ros2 launch rm_serial_driver keyboard_teleop.launch.py
```

默认键位：

- 底盘移动
  - `W/S`：前进 / 后退
  - `A/D`：左移 / 右移
  - `Q/E`：左转 / 右转
  - `O` 或空格：急停
- 底盘速度
  - `+ / -`：增大 / 减小线速度
  - 上 / 下方向键：增大 / 减小角速度
- 机械臂目标
  - `R / F`：升降机构上升 / 下降
  - `T / G`：滑台前伸 / 后退
  - `Y / H`：丝杆前伸 / 回收
  - `P`：机械臂目标清零

默认情况下，键盘节点会持续发布：

- `cmd_vel`
- `/platform/target`

如果你想改话题名或步进，可以直接在 launch 时传参，例如：

```bash
ros2 launch rm_serial_driver keyboard_teleop.launch.py \
  cmd_vel_topic:=/cmd_vel_selected \
  platform_target_topic:=/platform/target \
  lift_step_mm:=20.0 \
  slide_step_mm:=10.0 \
  screw_step_mm:=5.0
```

## 串口协议对应

详细包结构见 [include/rm_serial_driver/packet.hpp](include/rm_serial_driver/packet.hpp)。

当前已接入的主要命令如下：

| cmd_id | 方向 | 说明 |
| --- | --- | --- |
| `0x0402` | ROS -> MCU | 自瞄目标角度与开火命令 |
| `0x0405` | ROS -> MCU | 底盘速度指令 |
| `0x0406` | ROS -> MCU | 机械臂目标位置 |
| `0x0502` | MCU -> ROS | 云台当前姿态 |
| `0x0503` | MCU -> ROS | 底盘当前状态 |
| `0x0505` | MCU -> ROS | 机器人状态 / 复位信息 |
| `0x0506` | MCU -> ROS | 机械臂当前状态 |

## ROS 话题

默认订阅的话题：

- `/tracker/target`
  - 类型：`auto_aim_interfaces/msg/Target`
  - 用途：自瞄节点输出，发送到下位机 `0x0402`
- `cmd_vel`
  - 类型：`geometry_msgs/msg/Twist`
  - 用途：底盘速度控制，发送到下位机 `0x0405`
- `/platform/target`
  - 类型：`auto_aim_interfaces/msg/PlatformTarget`
  - 用途：机械臂目标位置，发送到下位机 `0x0406`

默认发布的话题：

- `/serial/gimbal_joint_state`
  - 类型：`geometry_msgs/msg/Twist`
  - 用途：发布下位机回传的云台姿态
- `/wheel/twist`
  - 类型：`geometry_msgs/msg/TwistWithCovarianceStamped`
  - 用途：发布下位机回传的底盘速度
- `/chassis_control_mode`
  - 类型：`std_msgs/msg/String`
  - 用途：发布底盘控制模式，值通常为 `manual` 或 `auto`
- `/platform/current_status`
  - 类型：`auto_aim_interfaces/msg/PlatformCurrentStatus`
  - 用途：发布下位机回传的机械臂当前状态
- `/aiming_point`
  - 类型：`visualization_msgs/msg/Marker`
  - 用途：可视化弹道解算目标点
- `/latency`
  - 类型：`std_msgs/msg/Float64`
  - 用途：调试总延迟

## 机械臂消息

机械臂相关消息定义在 `auto_aim_interfaces`：

`PlatformTarget.msg`

```text
std_msgs/Header header
float32 target_lift_z_mm
float32 target_slide_x_mm
float32 target_screw_x_mm
```

`PlatformCurrentStatus.msg`

```text
std_msgs/Header header
float32 target_lift_z_mm
float32 target_slide_x_mm
float32 target_screw_x_mm
uint8 control_mode
```

这两条消息分别对应：

- `PlatformTarget` -> 串口 `0x0406`
- `PlatformCurrentStatus` <- 串口 `0x0506`

## 关键参数

默认参数见 [config/serial_driver.yaml](config/serial_driver.yaml)：

- `device_name`：串口设备名
- `baud_rate`：波特率
- `publish_tf`：是否发布云台 TF
- `wheel_twist_topic`：底盘速度反馈话题名
- `cmd_vx_scale` / `cmd_vy_scale` / `cmd_wz_scale`：底盘速度发送缩放
- `platform_target_topic`：机械臂目标位置订阅话题，默认 `/platform/target`
- `platform_current_status_topic`：机械臂状态发布话题，默认 `/platform/current_status`

`keyboard_teleop` 相关参数：

- `cmd_vel_topic`：键盘控制底盘发布话题
- `platform_target_topic`：键盘控制机械臂发布话题
- `lift_step_mm`：每次按键调整的升降量
- `slide_step_mm`：每次按键调整的滑台位移
- `screw_step_mm`：每次按键调整的丝杆位移

## 说明

- 机械臂位置字段单位按协议使用 `mm`
- `keyboard_teleop` 发布的是目标位置，不是速度，所以按一次键会在当前目标基础上累加步进
- 若你修改了 `auto_aim_interfaces` 中的机械臂消息，需要重新编译 `auto_aim_interfaces` 和 `rm_serial_driver`
- 若 launch 中重映射了 `cmd_vel` 或 `/platform/target`，请确认控制节点与本节点使用的是同一个最终话题
