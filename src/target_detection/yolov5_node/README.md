# yolov5_node

ROS 2 Humble Python YOLOv5 ONNX detection package.

这个包负责从 RGB 图像中做目标检测，并发布 `yolo_msg/msg/YoloDetectionArray` 和可视化图像。

## 功能

- 支持 `topic` / `image` / `auto` 三种输入模式
- 支持 YOLOv5 v5 常见三输出头 ONNX
- 发布检测结果到 `/yolov5/detections`
- 发布带框图像到 `/yolov5/annotated_image`
- 输入订阅使用 `SensorDataQoS`，适合直接接相机图像

节点名：

```text
yolov5_onnx_node
```

可执行文件：

```text
yolov5_node
```

## 依赖

- ROS 2 Humble
- `cv_bridge`
- `sensor_msgs`
- `target_msgs/yolo_msg`
- Python: `onnxruntime`, `opencv-python`, `numpy`

安装常用依赖：

```bash
sudo apt update
sudo apt install -y ros-humble-cv-bridge python3-opencv
pip install onnxruntime numpy
```

## 输入输出

### 订阅

- `image_topic`，默认 `/image_raw`
- 消息类型：`sensor_msgs/msg/Image`

注意：

- 要传彩色图 topic，不要传深度图 topic
- 不要直接传 `CompressedImage`
- 常见可用 topic：
  - `/camera/color/image_raw`
  - `/camera/rgb/image_raw`
  - `/image_raw`

### 发布

- `/yolov5/detections`
  - 类型：`yolo_msg/msg/YoloDetectionArray`
- `/yolov5/annotated_image`
  - 类型：`sensor_msgs/msg/Image`

`YoloDetection` 主要字段：

- `target_id`
- `class_name`
- `class_id`
- `confidence`
- `box`
- `need_ocr`

## 模型文件

默认模型路径来自包内：

```text
$(find-pkg-share yolov5_node)/weights/best_int8.onnx
```

如果你的模型不在默认位置，可以在 launch 时覆盖：

```bash
model_path:=/your/path/best_int8.onnx
```

## 编译

在工作区根目录执行：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
colcon build --packages-select yolo_msg yolov5_node
source install/setup.bash
```

## 启动

### 1. 单张图片测试

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch yolov5_node yolov5_onnx.launch.py \
  input_mode:=image \
  image_path:=/host-home/src/yolov5_node/test/test.jpg \
  save_path:=/host-home/src/yolov5_node/test/result.jpg
```

运行后会：

- 读取 `image_path`
- 检测一次
- 保存可视化结果到 `save_path`
- 发布一次检测消息
- 默认自动退出

### 2. 相机实时检测

如果相机 RGB 图像话题是 `/camera/rgb/image_raw`：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch yolov5_node yolov5_onnx.launch.py \
  input_mode:=topic \
  image_topic:=/camera/rgb/image_raw
```

如果是 RealSense 常见话题：

```bash
ros2 launch yolov5_node yolov5_onnx.launch.py \
  input_mode:=topic \
  image_topic:=/camera/color/image_raw
```

### 3. auto 模式

如果给了有效 `image_path`，自动走单图模式；否则走 topic 模式：

```bash
ros2 launch yolov5_node yolov5_onnx.launch.py \
  input_mode:=auto \
  image_path:=/host-home/src/yolov5_node/test/test.jpg
```

## 查看结果

查看检测消息：

```bash
ros2 topic echo /yolov5/detections
```

查看带框图像：

```bash
ros2 run rqt_image_view rqt_image_view
```

然后在界面里选择：

```text
/yolov5/annotated_image
```

## 主要参数

配置文件在 [config/yolov5_onnx.yaml](/host-home/src/yolov5_node/config/yolov5_onnx.yaml:1)。

常用参数：

```yaml
model_path: $(find-pkg-share yolov5_node)/weights/best_int8.onnx
input_mode: topic
image_topic: /image_raw
image_path: ""
save_path: /mnt/yolov5/result.jpg
exit_after_image: true

detections_topic: /yolov5/detections
annotated_topic: /yolov5/annotated_image

img_size: 640
conf_thres: 0.60
iou_thres: 0.85
num_threads: 1
```

launch 文件在 [launch/yolov5_onnx.launch.py](/host-home/src/yolov5_node/launch/yolov5_onnx.launch.py:1)。

## 与 OCR 的关系

`yolov5_node` 只负责检测，不直接调用 `ocr_node`。

当前链路应该是：

```text
RGB image
  -> yolov5_node
  -> /yolov5/detections
  -> 上游桥接/融合节点
  -> /vision/ocr_requests
  -> ocr_node
```

也就是说，`ocr_node` 需要接收 `ocr_msg/msg/OcrRequestArray`，不是直接订阅 `/yolov5/detections`。

## 常见问题

### 1. `Package 'yolov5_node' not found`

先重新编译并重新 source：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
colcon build --packages-select yolo_msg yolov5_node
source install/setup.bash
```

### 2. 有相机图像，但没有检测结果

先确认：

- 传入的是 RGB 图像，不是 depth 图
- topic 类型是 `sensor_msgs/msg/Image`
- topic 名字和 launch 参数一致

### 3. `/yolov5/annotated_image` 没图

通常检查这几项：

- 相机是否真的在发图
- `image_topic` 是否写对
- 是否已经 `source /host-home/install/setup.bash`
- 模型文件是否存在
