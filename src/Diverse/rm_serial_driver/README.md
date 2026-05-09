# rm_serial_driver

RoboMaster 视觉系统与下位机之间的 ROS 2 串口通信节点。

<img src="docs/rm_vision.svg" alt="rm_vision" width="200" height="200">

这个包基于 [transport_drivers](https://github.com/ros-drivers/transport_drivers)，用于把 `rm_auto_aim`、底盘控制和机械臂控制的数据通过串口与电控板互通。

---

# 功能概览

- 接收下位机回传的云台姿态、底盘状态、机械臂状态
- 向下位机发送自瞄目标解算结果
- 向下位机发送底盘速度指令
- 向下位机发送机械臂目标位置指令
- 提供键盘遥控入口，可同时控制底盘和机械臂目标
- 发布可视化和调试用话题

---

# 启动

安装依赖：

```bash
sudo apt install ros-humble-serial-driver