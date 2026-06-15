# Robot2 OAK Package Servo Spec v3

## 定版原则

- Nav2 只负责导航到货架预停点。
- 扫描包裹后，无论成功失败，都保留原来的 `RequestOakPackageSearchAlign` 回正货架逻辑。
- 找到目标后，回正完成再进入 OAK 取件运动。
- 取件运动主控制为 `OAK 实时识别 + base_link/oak-cam-Link TF/URDF 变换`。
- 语义货架、`/map`、`/scan` 只做安全限制和异常校验。
- OAK 取件节点输出和 Nav2 同一套标准 ROS `cmd_vel` 定义，不直接发布 `/cmd_vel_selected`。

## 行为树增量插入

保留原行为树文件：

```text
/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml
```

只把成功分支从：

```xml
<ScanTargetPackageOnShelf/>
<RequestOakPackageSearchAlign/>
<WaitAfterOakAlign/>
<PublishTaskDone/>
```

增量改成：

```xml
<ScanTargetPackageOnShelf/>
<CacheFoundPackagePose/>
<RequestOakPackageSearchAlign/>
<WaitAfterOakAlign/>
<RequestOakServoCenterAlign/>
<RequestOakServoApproach/>
<RequestOakServoFineAlign/>
<PublishTaskDone/>
```

失败分支保持：

```xml
<RequestOakPackageSearchAlign/>
<WaitAfterOakAlign/>
<AlwaysFailure/>
```

## 主运动解算

OAK 输出：

```text
camera_xyz_mm.x = 相机右方
camera_xyz_mm.y = 相机下方
camera_xyz_mm.z = 相机前方
```

转换为 OAK 运动坐标：

```text
P_oak = [camera_z, -camera_x, -camera_y]
```

含义：

```text
P_oak.x = 包裹在 OAK 前方距离
P_oak.y = 包裹相对 OAK 左右偏差
P_oak.z = 包裹相对 OAK 高低偏差
```

通过 TF/URDF 获取：

```text
R_base_oak = rotation(base_link <- oak-cam-Link)
```

中心对齐阶段：

```text
E_oak = [0, P_oak.y, 0]
E_base = R_base_oak × E_oak
cmd.linear.x = ky * E_base.x
cmd.linear.y = ky * E_base.y
cmd.angular.z = kyaw * oak_yaw_error
```

注意：中心对齐阶段也保留 `E_base.x / E_base.y` 混合分量。这是为了适配 OAK 侧装或斜装；这里的运动方向是 OAK 横向误差在 base_link 下的投影，不是写死只走 base y。

靠近阶段：

```text
E_oak = [P_oak.x - standoff_m, P_oak.y, 0]
E_base = R_base_oak × E_oak
cmd.linear.x = kx * E_base.x
cmd.linear.y = ky * E_base.y
cmd.angular.z = kyaw * oak_yaw_error
```

最终输出到：

```text
/cmd_vel_auto
```

由现有统一启动中的：

```text
cmd_vel_mode_mux -> /cmd_vel_selected -> rm_serial_driver
```

完成真实底盘方向适配。

## 状态与误差日志

节点持续发布：

```text
/target_detection/package_servo/status
```

包含：

```text
stage
status
p_oak.x / p_oak.y
e_oak.x / e_oak.y
e_base.x / e_base.y
yaw_error
cmd_raw
cmd_safe
safety
reason
```

查看脚本：

```bash
ros2 run oak_package_servo oak_servo_error_watch
```

或：

```bash
python3 tools/watch_oak_servo_error_log.py --csv /tmp/oak_servo_error.csv
```

## 推荐初值

```yaml
standoff_m: 0.55
min_standoff_m: 0.40
oak_hard_stop_m: 0.38
shelf_hard_stop_distance_m: 0.35

kx: 0.35
ky: 0.35
kyaw: 0.8

max_vx: 0.05
max_vy: 0.05
max_wz: 0.25

x_tolerance_m: 0.035
y_tolerance_m: 0.025
fine_y_tolerance_m: 0.018
yaw_tolerance_rad: 0.05

target_lock_min_frames: 3
target_lost_timeout_sec: 0.8
```
