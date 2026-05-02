# astra_s_cam

`astra_s_cam` 是一个简短命名的 ROS2 Humble 单包版 Astra S 深度相机包。

它**不依赖 `openni2_camera` ROS 包**，而是直接调用 Orbbec/OpenNI2 SDK：

```text
/host-home/Software/OpenNi2/sdk/libs/libOpenNI2.so
/host-home/Software/OpenNi2/sdk/libs/OpenNI2/Drivers/liborbbec.so
```

适用设备示例：

```bash
lsusb
# ID 2bc5:0402 ORBBEC ASTRA S
```

---

## 1. 功能

```text
1. 直接调用 OpenNI2 SDK 读取 Astra S
2. 发布 RGB 图像
3. 发布 Depth 深度图
4. 发布 RGB / Depth CameraInfo
5. 发布基础 TF
6. 订阅目标像素点
7. RGB-D 融合输出目标三维位置，单位 mm
8. 输出 RViz2 可视化目标点，单位 m
9. 提供深度距离标定脚本
10. 提供 RGB 标定 launch
11. 提供 RViz2 配置
```

---

## 2. 包结构

```text
astra_s_cam/
├── src/
│   └── astra_s_driver_node.cpp
├── scripts/
│   ├── rgbd_locator_node.py
│   └── depth_calib_node.py
├── launch/
│   ├── astra.launch.py
│   └── calib.launch.py
├── config/
│   └── astra_s.yaml
├── rviz/
│   └── astra.rviz
├── calib/
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 安装依赖

```bash
apt update
apt install -y \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-camera-calibration \
  ros-humble-rviz2 \
  ros-humble-tf2-ros
```

检查 OpenNI2 SDK：

```bash
find /host-home/Software/OpenNi2/sdk -name "OpenNI.h"
find /host-home/Software/OpenNi2/sdk/libs -name "libOpenNI2.so"
find /host-home/Software/OpenNi2/sdk/libs/OpenNI2/Drivers -type f
```

应该能看到：

```text
OpenNI.h
libOpenNI2.so
liborbbec.so
libOniFile.so
```

---

## 4. 编译

如果 SDK 路径是默认路径：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --packages-select astra_s_cam
source install/setup.bash
```

如果 SDK 不在默认路径：

```bash
export OPENNI2_INCLUDE=/your/OpenNi2/sdk/include
export OPENNI2_REDIST=/your/OpenNi2/sdk/libs

cd /host-home
source /opt/ros/humble/setup.bash
colcon build --packages-select astra_s_cam
source install/setup.bash
```

---

## 5. 启动相机

```bash
ros2 launch astra_s_cam astra.launch.py
```

带 RViz2：

```bash
ros2 launch astra_s_cam astra.launch.py use_rviz:=true
```

只启动相机，不启动融合节点：

```bash
ros2 launch astra_s_cam astra.launch.py use_locator:=false
```

只开深度：

```bash
ros2 launch astra_s_cam astra.launch.py enable_rgb:=false enable_depth:=true
```

---

## 6. 话题

相机话题：

```text
/camera/rgb/image_raw
/camera/depth/image_raw
/camera/rgb/camera_info
/camera/depth/camera_info
```

融合定位话题：

```text
/astra_s_cam/target_pixel
/astra_s_cam/target_position_mm
/astra_s_cam/target_point
/astra_s_cam/debug_image
```

检查频率：

```bash
ros2 topic hz /camera/rgb/image_raw --qos-reliability best_effort
ros2 topic hz /camera/depth/image_raw --qos-reliability best_effort
```

---

## 7. RGB-D 融合定位

输入目标像素点：

```bash
ros2 topic pub --once /astra_s_cam/target_pixel geometry_msgs/msg/PointStamped "
header:
  frame_id: camera_rgb_optical_frame
point:
  x: 320.0
  y: 240.0
  z: 1.0
"
```

其中：

```text
point.x = u 像素坐标
point.y = v 像素坐标
point.z = confidence，可填 1.0
```

输出毫米坐标：

```bash
ros2 topic echo /astra_s_cam/target_position_mm
```

输出 RViz2 可视化点，单位 m：

```bash
ros2 topic echo /astra_s_cam/target_point
```

计算公式：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

---

## 8. RGB 标定

建议使用 A4 打印棋盘格，不建议用电脑屏幕。

如果使用 8x11 内部角点、20mm 方格：

```bash
ros2 launch astra_s_cam calib.launch.py mode:=rgb board_size:=8x11 square:=0.020
```

或者分开跑：

```bash
ros2 launch astra_s_cam astra.launch.py use_locator:=false
```

另一个终端：

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x11 \
  --square 0.020 \
  --no-service-check \
  image:=/camera/rgb/image_raw \
  camera:=/camera/rgb
```

`CALIBRATE` 按钮灰色通常是因为：

```text
1. 棋盘格样本不够
2. 棋盘格内部角点数量设置错
3. 棋盘格太小/太远
4. 光照反光或图像模糊
5. 使用电脑屏幕导致角点检测不稳定
```

---

## 9. 深度距离标定

```bash
ros2 launch astra_s_cam calib.launch.py mode:=depth
```

或：

```bash
ros2 run astra_s_cam depth_calib_node.py
```

日志会输出：

```text
Center median raw depth = 500.0 mm
```

用真实距离和 raw depth 拟合：

```text
real = raw * depth_scale_correction + depth_offset_mm
```

然后修改：

```bash
config/astra_s.yaml
```

例如：

```yaml
rgbd_locator_node:
  ros__parameters:
    depth_scale_correction: 0.971
    depth_offset_mm: 0.0
```

---

## 10. 常见问题

### 10.1 OpenNI2 找不到设备

```bash
lsusb | grep -i -E "orbbec|2bc5"
chmod -R 777 /dev/bus/usb
```

如果在 VMware/Docker 中运行，请确认 USB 已转发，并且使用 USB 3.0/xHCI。

### 10.2 RViz2 显示 No Image

RViz2 Image Display 设置：

```text
Reliability Policy: Best Effort
Durability Policy: Volatile
History Policy: Keep Last
```

### 10.3 一会儿有深度，一会儿 No valid depth

这通常不是断流，而是中心区域没有有效深度。使用白色亚光纸板，放在画面中心，距离 500-1000mm。

### 10.4 启动后像卡住

ROS2 launch 会一直前台运行，这是正常的。另开一个终端检查 topic 或发布目标点。

---

## 11. 推荐运行顺序

```text
1. ros2 launch astra_s_cam astra.launch.py
2. 检查 RGB/Depth topic hz
3. RViz2 查看图像
4. RGB 标定
5. 深度标定
6. 检测包发布 /astra_s_cam/target_pixel
7. 使用 /astra_s_cam/target_position_mm
```
