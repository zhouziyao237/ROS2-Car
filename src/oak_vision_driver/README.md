# oak_vision_driver

`oak_vision_driver` 是一个直接调用 DepthAI SDK 的 ROS2 OAK-D Lite 驱动包。它统一发布 RGB、Depth、CameraInfo，并可选在 OAK 端加载 YOLO `.blob`，输出兼容原 `yolov5_node` 的 `/yolov5/detections`。

## 1. 设计目标

```text
mode:=original
OAK 只作为 RGB-D 相机使用，YOLO 继续用原 yolov5_node。

mode:=oak
OAK 作为 RGB-D 相机，同时在 OAK Myriad X 上运行 YOLO blob，替代原 yolov5_node。
```

下游保持不变：

```text
/yolov5/detections -> target_matcher_node -> ocr_node -> matched_depth_locator_node
```

## 2. 配置文件分离

### config/oak_vision_driver.yaml

运行资源和话题配置：

```text
RGB/depth topic
是否启用 OAK YOLO
FPS
OAK queue size
camera_info 发布频率
depth 对齐参数
```

### config/network.yaml

网络结构描述：

```text
blob_path
input_width/input_height
num_classes
anchors
anchor_masks
class_names
confidence/iou threshold
```

换模型时优先只改 `network.yaml`。

## 3. 安装依赖

```bash
sudo apt update
sudo apt install python3-pip ros-humble-cv-bridge python3-yaml
python3 -m pip install depthai
```

测试 OAK 是否连接：

```bash
python3 - <<'PY'
import depthai as dai
print(dai.Device.getAllAvailableDevices())
PY
```

## 4. 编译

```bash
cd /host-home
source /opt/ros/humble/setup.bash
colcon build --packages-select oak_vision_driver target_matcher ocr_node yolov5_node
source install/setup.bash
```

## 5. 只测试 OAK RGB-D

```bash
ros2 launch oak_vision_driver oak_vision_driver.launch.py enable_yolo:=false
```

检查：

```bash
ros2 topic list | grep oak
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/depth/image_raw
ros2 topic echo /oak/rgb/camera_info --once
```

## 6. original 模式：兼容原 yolov5_node

OAK 只发布 RGB/depth/camera_info，YOLO 仍然由原 `yolov5_node` 在 mini PC 上运行。

```bash
ros2 launch oak_vision_driver oak_target_tracking.launch.py \
  mode:=original \
  target_code:=2-3-2050 \
  yolo_num_threads:=1 \
  yolo_conf_thres:=0.35
```

## 7. oak 模式：OAK 端 YOLO blob

先把 OAK/MyriadX 可运行模型放到：

```bash
/host-home/src/oak_vision_driver/weights/best.blob
```

重新编译安装资源：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
colcon build --packages-select oak_vision_driver
source install/setup.bash
```

启动：

```bash
ros2 launch oak_vision_driver oak_target_tracking.launch.py \
  mode:=oak \
  blob_path:=/host-home/src/oak_vision_driver/weights/best.blob \
  target_code:=2-3-2050
```

检查 detection：

```bash
ros2 topic echo /yolov5/detections
ros2 topic hz /yolov5/detections
```

## 8. 资源占用建议

推荐默认：

```yaml
fps: 15.0
oak_queue_size: 2
sleep_ms: 2.0
publish_camera_info_every_frame: false
camera_info_period_sec: 1.0
set_nn_threads: 1
```

如果需要更高检测频率，再逐步提升：

```yaml
fps: 20.0
set_nn_threads: 2
```

## 9. 注意

`YoloDetectionNetwork` 需要 `.blob` 的输出格式符合 DepthAI YOLO parser。若你的 blob 不是标准 YOLO parser 格式，后续需要改成 `NeuralNetwork + host-side YOLO decode`。
