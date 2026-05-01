# ros_astra_s_camera 中文说明

这个工作区主要包含两个 ROS 2 Humble 包：

- `ros_astra_s_camera_msgs`
  提供目标像素和三维位置的自定义消息。
- `ros_astra_s_camera`
  提供 Astra S 相机启动、RGB-D 融合节点、标定入口和 RViz 配置。

这份文档重点说明当前仓库里最常用的几个文件、如何启动、如何标定，以及出了问题该看哪里。

## 1. 目录结构

常用文件：

- `src/ros_astra_s_camera/ros_astra_s_camera/launch/astra_s_camera.launch.py`
  日常运行入口。启动相机驱动、TF、RGB-D 融合节点，可选 RViz / 点云。
- `src/ros_astra_s_camera/ros_astra_s_camera/launch/astra_s_calibration.launch.py`
  标定入口。启动相机驱动、RGB 标定工具、深度检查节点。
- `src/ros_astra_s_camera/ros_astra_s_camera/config/astra_s_camera.yaml`
  统一配置文件。OpenNI2 路径、标定文件路径、RGB-D 节点参数都在这里。
- `src/ros_astra_s_camera/ros_astra_s_camera/calib/`
  建议存放 RGB / depth 标定 YAML 的目录。
- `src/ros_astra_s_camera/ros_astra_s_camera/scripts/rgbd_position_node.py`
  把 RGB 目标像素和深度图融合成三维坐标。
- `src/ros_astra_s_camera/ros_astra_s_camera/scripts/depth_calibration_node.py`
  输出深度图中心区域的中值深度，方便做深度尺度修正。

## 2. 启动前准备

确认相机已被系统识别：

```bash
lsusb | grep -i orbbec
```

常见输出类似：

```bash
Bus XXX Device XXX: ID 2bc5:0402 ORBBEC ASTRA S
```

确认 OpenNI2 路径配置正确。当前工程优先级是：

1. 环境变量 `OPENNI2_REDIST` / `OPENNI2_DRIVERS_PATH`
2. `astra_s_camera.yaml` 里的 `openni2.redist` / `openni2.drivers`

如果两者都没配，launch 会直接报错，不再静默回退。

## 3. 编译

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select ros_astra_s_camera_msgs ros_astra_s_camera
source install/setup.bash
```

## 4. 日常运行

最常用的启动方式：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_astra_s_camera astra_s_camera.launch.py
```

带 RViz：

```bash
ros2 launch ros_astra_s_camera astra_s_camera.launch.py use_rviz:=true
```

低占用模式：

```bash
ros2 launch ros_astra_s_camera astra_s_camera.launch.py use_rviz:=true lite_mode:=true
```

这个模式会做几件事：

- 切换到轻量 RViz 配置
- 把 RViz 图像显示队列降到 `1`
- 把 RViz 刷新率降到 `15 FPS`
- 缩小 `rgbd_position_node.py` 的同步缓存
- 关闭“没有目标时自动取画面中心”的兜底计算

带点云：

```bash
ros2 launch ros_astra_s_camera astra_s_camera.launch.py use_cloud:=true use_rviz:=true
```

运行后常看这些话题：

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/rgb/image_raw
ros2 topic hz /camera/depth/image
ros2 topic echo /camera/rgb/camera_info --once
```

## 5. 两个 launch 脚本分别做什么

### `astra_s_camera.launch.py`

主要负责：

1. 从 YAML 读取 OpenNI2 路径和标定文件路径。
2. 启动 `openni2_camera` 驱动。
3. 发布标准相机 TF。
4. 启动 `rgbd_position_node.py`。
5. 按参数决定是否启用 RViz 和点云。

常用 launch 参数：

```bash
ros2 launch ros_astra_s_camera astra_s_camera.launch.py use_rviz:=true
ros2 launch ros_astra_s_camera astra_s_camera.launch.py use_cloud:=true
ros2 launch ros_astra_s_camera astra_s_camera.launch.py namespace:=camera
```

### `astra_s_calibration.launch.py`

主要负责：

1. 启动同一套相机驱动。
2. 启动 ROS 官方 `camera_calibration` GUI 做 RGB 标定。
3. 启动 `depth_calibration_node.py` 读取中心区域深度中值。

如果你是纯 SSH 终端，没有桌面/X11/VNC，这个 launch 里的 RGB 标定 GUI 不会显示。

## 6. RGB 标定

当前棋盘格参数来自：

```yaml
calibration:
  board_size: "8x11"
  square_size_m: "0.030"
```

含义：

- `board_size`
  内角点数量，不是黑白格数量。
- `square_size_m`
  单个方格的边长，单位米。

启动标定：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_astra_s_camera astra_s_calibration.launch.py
```

如果希望一边标定一边在 RViz2 里看 RGB、Depth 和 TF：

```bash
ros2 launch ros_astra_s_camera astra_s_calibration.launch.py use_rviz:=true
```

如果图形环境比较吃紧，建议用低占用标定模式：

```bash
ros2 launch ros_astra_s_camera astra_s_calibration.launch.py use_rviz:=true lite_mode:=true
```

如果只想手动打开 RViz2，也可以另开一个终端执行：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rviz2 rviz2 -d /host-home/install/ros_astra_s_camera/share/ros_astra_s_camera/rviz/astra_s_camera.rviz
```

操作建议：

1. 把棋盘格放到画面中央，确保识别稳定。
2. 逐步移动到四角、边缘、不同距离、不同倾角。
3. 在 GUI 中完成采样后执行 `CALIBRATE`。
4. 保存结果为 RGB 相机标定 YAML。

建议把 RGB 标定结果放到：

```bash
/host-home/src/ros_astra_s_camera/ros_astra_s_camera/calib/rgb_Astra_Orbbec.yaml
```

然后重新编译：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
colcon build --packages-select ros_astra_s_camera
source install/setup.bash
```

当前配置会把 `calib/rgb_Astra_Orbbec.yaml` 解析为安装后的包路径：

```bash
/host-home/install/ros_astra_s_camera/share/ros_astra_s_camera/calib/rgb_Astra_Orbbec.yaml
```

## 7. 深度标定

这个工程里的“深度标定”目前不是完整深度内参标定，而是做距离修正。

`depth_calibration_node.py` 会读取深度图中心区域，输出中值深度：

```text
Center median raw depth = 850.0 mm
```

做法：

1. 在相机前方放一块平整板子。
2. 用尺子量真实距离。
3. 记录几组 `raw_depth_mm` 和 `real_depth_mm`。
4. 根据下面公式拟合：

```text
real_depth_mm = raw_depth_mm * depth_scale_correction + depth_offset_mm
```

如果只有两组点 `(raw1, real1)`、`(raw2, real2)`，可以先这样算：

```text
scale  = (real2 - real1) / (raw2 - raw1)
offset = real1 - scale * raw1
```

再把结果写回：

```yaml
rgbd_position_node:
  ros__parameters:
    depth_scale_correction: 1.0
    depth_offset_mm: 0.0
```

## 8. RGB 像素到深度像素的对齐修正

如果 RGB 检测框中心和实际深度点不重合，可以调：

```yaml
rgb_to_depth_offset_u: 0.0
rgb_to_depth_offset_v: 0.0
rgb_to_depth_scale_u: 1.0
rgb_to_depth_scale_v: 1.0
```

映射公式：

```text
u_depth = u_rgb * rgb_to_depth_scale_u + rgb_to_depth_offset_u
v_depth = v_rgb * rgb_to_depth_scale_v + rgb_to_depth_offset_v
```

建议先把 `scale` 保持 `1.0`，只微调 `offset`。

## 9. 如何接入目标识别并输出实际三维位置

这个包当前的分工是：

- 你的检测节点负责在 `RGB` 图像里找到目标，并发布目标中心像素
- `rgbd_position_node.py` 负责把这个像素点和深度图对齐，计算三维位置

也就是说，这个包本身不限定你必须用哪种识别算法。YOLO、模板匹配、颜色分割、自定义 OpenCV 节点都可以，只要最终发布一条 `TargetPixel` 消息即可。

### 9.1 输入输出话题

检测节点要发布：

```text
/astra_s/target_pixel
```

消息类型：

```text
ros_astra_s_camera_msgs/msg/TargetPixel
```

定义如下：

```text
std_msgs/Header header
float64 u
float64 v
float64 confidence
string target_id
```

其中：

- `u`
  RGB 图像中的目标中心横坐标，单位像素
- `v`
  RGB 图像中的目标中心纵坐标，单位像素
- `confidence`
  检测置信度
- `target_id`
  目标类别或名称，比如 `person`、`bottle`、`red_blob`

融合节点会输出：

```text
/astra_s/target_position_mm
/astra_s/target_point
```

对应类型：

- `ros_astra_s_camera_msgs/msg/TargetPositionMM`
- `geometry_msgs/msg/PointStamped`

### 9.2 整体调用流程

1. 启动相机和 RGB-D 融合节点：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_astra_s_camera astra_s_camera.launch.py use_rviz:=true
```

2. 启动你自己的目标检测节点。

3. 检测节点从 `/camera/rgb/image_raw` 取图，算出目标框中心 `(u, v)`。

4. 检测节点把 `(u, v)` 连同原始 `image.header.stamp` 一起发布到 `/astra_s/target_pixel`。

5. `rgbd_position_node.py` 会先同步 `RGB` 和 `/camera/depth/image`，再按时间戳匹配对应的 `TargetPixel`，最后输出三维坐标。

### 9.3 三维坐标是怎么算出来的

`rgbd_position_node.py` 内部做了 3 件事：

1. 用 `ApproximateTimeSynchronizer` 同步 `RGB` 和 `depth` 图像
2. 按 `header.stamp` 在缓存中找到对应时刻的 `TargetPixel`
3. 在深度图中读取目标像素附近的中值深度
4. 用相机内参把像素坐标反投影成三维坐标

核心公式：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

当前输出坐标系默认是：

```text
camera_rgb_optical_frame
```

也就是“相机坐标系下的三维位置”，不是地图坐标或机器人底盘坐标。

### 9.4 最小测试方法

如果你还没有自己的检测算法，可以先手动发布一个目标像素测试整条链路：

```bash
ros2 topic pub --once /astra_s/target_pixel ros_astra_s_camera_msgs/msg/TargetPixel "{u: 320.0, v: 240.0, confidence: 1.0, target_id: 'center'}"
```

然后查看输出：

```bash
ros2 topic echo /astra_s/target_position_mm
ros2 topic echo /astra_s/target_point
```

### 9.5 示例检测节点

仓库里已经加了一个最小示例节点：

```text
ros_astra_s_camera/target_pixel_demo_node.py
```

它会：

- 订阅 `/camera/rgb/image_raw`
- 用最简单的 HSV 颜色分割找红色目标
- 取最大红色区域的中心点
- 发布到 `/astra_s/target_pixel`

启动方式：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ros_astra_s_camera target_pixel_demo_node.py
```

如果要同时看融合结果，另开一个终端启动主节点：

```bash
ros2 launch ros_astra_s_camera astra_s_camera.launch.py use_rviz:=true
```

然后把一个较明显的红色物体放到画面里，就可以在下面这些话题看到结果：

```text
/astra_s/target_pixel
/astra_s/target_position_mm
/astra_s/target_point
/astra_s/detector_debug_image
```

### 9.6 如果你用 YOLO / 自己的检测器，应该怎么接

你自己的检测节点只需要做下面这件事：

1. 订阅 RGB 图像
2. 跑目标检测
3. 取目标框中心：

```text
u = (xmin + xmax) / 2
v = (ymin + ymax) / 2
```

4. 发布 `TargetPixel`

Python 伪代码：

```python
msg = TargetPixel()
msg.header = image_msg.header
msg.u = bbox_center_x
msg.v = bbox_center_y
msg.confidence = score
msg.target_id = class_name
target_pub.publish(msg)
```

只要这一步打通，后面的深度同步和三维坐标计算都由本包完成。

为了让同步真正有效，建议你的检测节点始终复用输入 RGB 图像的 `header`，不要自己生成一个新的时间戳。

### 9.7 如何得到“实际世界坐标”

当前 `/astra_s/target_position_mm` 输出的是相机坐标系下的位置。要得到机器人或地图中的真实位置，还需要再做一步 TF 变换：

```text
camera_rgb_optical_frame -> base_link -> map
```

也就是说：

- 先保证相机安装位姿相对于机器人是已知的
- 再用 TF 把 `/astra_s/target_point` 变换到 `base_link` 或 `map`

如果只设置 `camera_to_base_x_mm / y / z`，现在只能补偿平移，不能补偿旋转。  
如果相机不是完全摆正安装，建议后续用 `tf2` 做完整坐标变换。

## 10. 标定文件路径说明

当前配置文件里有两项：

```yaml
calibration:
  rgb_info_uri: "calib/rgb_Astra_Orbbec.yaml"
  depth_info_uri: "calib/depth_Astra_Orbbec.yaml"
```

这两个值支持三种写法：

1. `file:///abs/path/to/file.yaml`
2. `/abs/path/to/file.yaml`
3. `calib/file.yaml`

第三种会被解析为“相对安装后包目录”的路径，最适合当前工程。

如果暂时没有 depth 标定文件，可以先把：

```yaml
depth_info_uri: ""
```

这样驱动就不会去找不存在的 depth YAML。

## 10. 常见问题

### 1. 为什么日志里出现 `/root/.ros/camera_info/...`

这是 `openni2_camera` 在没有拿到显式标定文件时的默认回退路径。  
如果你用 `root` 启动，默认用户目录就是 `/root`。

现在这个工程已经改成优先从 `astra_s_camera.yaml` 里显式传入标定 URI。

### 2. 为什么还是提示标定文件不存在

因为 `calib/` 目录里还没有实际的 YAML 文件。  
launch 现在找的是我们配置的新路径，不再是 `/root/.ros/...`。

### 3. 为什么 `Ctrl-C` 后 launch 里看到节点异常退出

这个问题已经在 `rgbd_position_node.py` 里修过，当前版本会 clean shutdown。

### 4. 为什么 `ros2 topic hz` 没输出

最常见原因是：

1. 命令打在正在前台运行的 `ros2 launch` 终端里，没有真正执行。
2. 应该另开一个终端再测。

例如：

```bash
# 终端1
ros2 launch ros_astra_s_camera astra_s_camera.launch.py

# 终端2
ros2 topic hz /camera/rgb/image_raw
```

## 11. 推荐调试顺序

建议按这个顺序排问题：

1. `lsusb | grep -i orbbec`
2. `ros2 launch ros_astra_s_camera astra_s_camera.launch.py`
3. `ros2 topic list | grep camera`
4. `ros2 topic echo /camera/rgb/camera_info --once`
5. `ros2 topic hz /camera/rgb/image_raw`
6. 做 RGB 标定
7. 做深度距离修正
8. 再调 RGB 到 depth 的像素偏移
