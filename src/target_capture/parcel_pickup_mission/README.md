# parcel_pickup_mission

自动快递取件任务状态机。这个包只负责“决策调度”，把你现有的相机、QR 识别、目标定位、底盘对齐、平台升降、两节伸出、串口控制串起来。

---

## 1. 取件码格式

用户只需要输入一个取件码：

```text
<货架列>-<货架层>-<包裹号>
```

示例：

```text
2-3-1285
```

含义：

```text
第 2 列货架
第 3 层
包裹号 1285
```

输入命令：

```bash
ros2 topic pub --once /mission/pickup_code std_msgs/msg/String "{data: '2-3-1285'}"
```

---

## 2. 三种二维码格式

### 2.1 货架列二维码

贴在每一列货架明显位置。内容使用 JSON。

第 1 列：

```json
{"col":1}
```

第 2 列：

```json
{"col":2}
```

第 3 列：

```json
{"col":3}
```

QR matcher 会标准化为：

```text
COL:1
COL:2
COL:3
```

例如取件码是：

```text
2-3-1285
```

状态机会先发布：

```text
/target_code = COL:2
```

然后寻找：

```json
{"col":2}
```

---

### 2.2 货架层二维码

贴在每一层旁边。推荐直接写该层的粗升高度。

第 1 层：

```json
{"row":1,"target_lift_mm":60}
```

第 2 层：

```json
{"row":2,"target_lift_mm":150}
```

第 3 层：

```json
{"row":3,"target_lift_mm":240}
```

第 4 层：

```json
{"row":4,"target_lift_mm":330}
```

`target_lift_mm` 是该层的粗定位起始高度，不是最终取件高度。平台先升到这个高度附近，然后再用包裹二维码做二次精对齐。

QR matcher 会标准化为：

```text
ROW:1
ROW:2
ROW:3
ROW:4
```

例如取件码为 `2-3-1285`，状态机会发布：

```text
/target_code = ROW:3
```

并读取层二维码原文：

```json
{"row":3,"target_lift_mm":240}
```

如果二维码里有 `target_lift_mm`，状态机会优先使用二维码里的高度。

如果你不想使用层二维码，把配置改成：

```yaml
use_row_qr: false
```

这时状态机会直接用 `pickup_mission.yaml` 里的 `row_heights_mm`。

---

### 2.3 包裹二维码

包裹二维码直接写纯数字，不建议第一版写 JSON。

例如：

```text
1285
```

```text
1069
```

```text
4309
```

---

## 3. 自动流程

输入：

```text
2-3-1285
```

状态机会自动执行：

```text
1. 解析取件码
   col=2, row=3, package=1285

2. 设置目标为货架列二维码
   /target_code = COL:2

3. 寻找货架列二维码
   {"col":2}

4. 设置目标为货架层二维码
   /target_code = ROW:3

5. 寻找货架层二维码
   {"row":3,"target_lift_mm":240}

6. 平台粗升到第 3 层
   /qr_lift_control/set_lift_mm = 240

7. 设置目标为包裹二维码
   /target_code = 1285

8. 寻找包裹二维码
   1285

9. 包裹二维码二次对齐
   x = 左右偏移，给底盘微调
   y = 升降偏移，给平台微调
   z = 深度距离，给伸出逻辑

10. 冻结升降

11. 第一节平台臂先伸出
    深度相机跟随第一节平台一起前进，用 z 闭环靠近包裹

12. 第一节停止后重新测 z

13. 第二节小臂伸出
    第二节伸出时深度相机不跟随，由 qr_lift_control_node 根据新的 z 计算第二节伸出距离

14. 收回第二节小臂

15. 收回第一节平台臂

16. 任务完成
```

---

## 4. 坐标定义

`/target_pose_base` 使用以下定义：

```text
x = 左右偏移
y = 升降偏移
z = 深度距离
```

控制关系：

```text
x -> 底盘左右对齐
y -> 平台升降二次对齐
z -> 第一节平台伸出和第二节小臂伸出
```

---

## 5. 统一启动

### 5.1 只启动取件系统

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash

ros2 launch parcel_pickup_mission pickup_system.launch.py
```

启动后输入取件码：

```bash
ros2 topic pub --once /mission/pickup_code std_msgs/msg/String "{data: '2-3-1285'}"
```

---

### 5.2 启动导航 + 取件系统

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash

ros2 launch parcel_pickup_mission nav_pickup_system.launch.py
```

如果你的导航 launch 文件名不是：

```text
mapping_nav_with_serial.launch.py
```

请修改：

```text
parcel_pickup_mission/launch/nav_pickup_system.launch.py
```

里的对应文件名。

---

## 6. 配置说明

配置文件：

```text
parcel_pickup_mission/config/pickup_mission.yaml
```

### 6.1 是否使用层二维码

```yaml
use_row_qr: true
```

`true` 表示流程中会寻找层二维码，例如：

```json
{"row":3,"target_lift_mm":240}
```

`false` 表示跳过层二维码，直接按配置表粗升。

---

### 6.2 层高表

```yaml
row_ids: ['1', '2', '3', '4']
row_heights_mm: [60.0, 150.0, 240.0, 330.0]
```

含义：

```text
第 1 层 -> 60mm
第 2 层 -> 150mm
第 3 层 -> 240mm
第 4 层 -> 330mm
```

如果识别到层二维码中的 `target_lift_mm`，则优先使用二维码里的高度。

---

### 6.3 对齐阈值

```yaml
x_deadband_mm: 35.0
y_deadband_mm: 30.0
stable_required_frames: 5
```

含义：

```text
x 偏差小于 35mm 连续 5 帧 -> 认为底盘左右对齐
y 偏差小于 30mm 连续 5 帧 -> 认为平台升降对齐
```

---

## 7. 调试命令

看状态机状态：

```bash
ros2 topic echo /mission/state
```

看状态机日志：

```bash
ros2 topic echo /mission/debug
```

看当前 QR matcher 状态：

```bash
ros2 topic echo /target_matcher/state
```

看当前识别到的二维码原文：

```bash
ros2 topic echo /target_matcher/raw
```

看目标 3D 坐标：

```bash
ros2 topic echo /target_pose_base
```

看平台控制输出：

```bash
ros2 topic echo /platform/target
```

看视觉 overlay：

```bash
ros2 run rqt_image_view rqt_image_view /target_position_overlay/image
```

---

## 8. 编译

```bash
cd /host-home
source /opt/ros/humble/setup.bash

colcon build --packages-select parcel_pickup_mission --symlink-install

source /host-home/install/setup.bash
```

如果你同时修改了 QR matcher 或 lift control，也编译：

```bash
colcon build --packages-select \
  parcel_pickup_mission \
  target_qr_matcher \
  qr_platform_lift_control \
  target_pickup_bringup \
  --symlink-install
```

---

## 9. 最小测试流程

准备二维码：

货架列二维码：

```json
{"col":2}
```

货架层二维码：

```json
{"row":3,"target_lift_mm":240}
```

包裹二维码：

```text
1285
```

启动：

```bash
ros2 launch parcel_pickup_mission pickup_system.launch.py
```

输入：

```bash
ros2 topic pub --once /mission/pickup_code std_msgs/msg/String "{data: '2-3-1285'}"
```

观察：

```bash
ros2 topic echo /mission/state
ros2 topic echo /mission/debug
```

---

## 10. 常见问题

### 10.1 找不到货架列二维码

确认取件码第一段和二维码内容一致。

取件码：

```text
2-3-1285
```

货架列二维码应为：

```json
{"col":2}
```

状态机会寻找：

```text
COL:2
```

---

### 10.2 找不到货架层二维码

取件码：

```text
2-3-1285
```

层二维码应为：

```json
{"row":3,"target_lift_mm":240}
```

状态机会寻找：

```text
ROW:3
```

如果不想使用层二维码，把配置改成：

```yaml
use_row_qr: false
```

---

### 10.3 粗升高度不准

修改层二维码里的：

```json
{"row":3,"target_lift_mm":240}
```

或者修改配置表：

```yaml
row_heights_mm: [60.0, 150.0, 240.0, 330.0]
```

`target_lift_mm` 是该层的粗定位起始高度，不是最终取件高度。

最终高度由包裹二维码二次识别后，通过 `/target_pose_base.point.y` 微调。

---

### 10.4 包裹二维码识别不到

包裹二维码建议直接写纯数字：

```text
1285
```

不要写 JSON。二维码尺寸要足够大。远距离识别建议使用 1920x1080 RGB 图像。

---

### 10.5 统一 launch 里底盘对齐节点启动失败

查询实际 executable 名称：

```bash
ros2 pkg executables qr_chassis_align_control
```

然后修改：

```text
parcel_pickup_mission/launch/pickup_system.launch.py
```

里的：

```python
executable='qr_chassis_align_node'
```
