**包说明、安装、编译、单张图片测试、相机话题测试、上游节点话题测试、输出查看、常见问题**

````bash
cd /host-home/ros2_ws/src/yolov5_onnx_ros2

cat > README.md <<'MD'
# yolov5_onnx_ros2

ROS2 YOLOv5 ONNX / INT8 ONNX 目标检测包。

本包用于在 ROS2 中调用 YOLOv5 v5 导出的 ONNX 或 INT8 ONNX 模型，支持三种输入模式：

- `topic`：订阅相机或上游节点输出的图像话题进行实时检测
- `image`：读取本地单张图片，检测一次并保存结果
- `auto`：如果提供有效 `image_path`，自动进入单图检测模式；否则进入话题检测模式

当前节点适配 YOLOv5 v5 常见三检测头输出格式：

```text
[1, 3, 80, 80, 6]
[1, 3, 40, 40, 6]
[1, 3, 20, 20, 6]
````

其中最后一维 `6 = 5 + 1`，代表当前模型是 1 个类别。

---

## 1. 包功能

节点名称：

```text
yolov5_onnx_node
```

默认订阅：

```text
/image_raw
```

默认发布：

```text
/yolov5/detections
/yolov5/annotated_image
```

其中：

* `/yolov5/detections`：检测框结果，类型为 `vision_msgs/msg/Detection2DArray`
* `/yolov5/annotated_image`：带检测框的可视化图像，类型为 `sensor_msgs/msg/Image`

---

## 2. 依赖安装

### ROS2 依赖

以 ROS2 Humble 为例：

```bash
sudo apt update

sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs \
  python3-opencv
```

### Python 依赖

```bash
pip install onnxruntime opencv-python numpy
```

如果使用的是 conda 环境，需要确认 ROS2 能够找到当前 Python 环境中的 `onnxruntime`。

测试：

```bash
python3 -c "import onnxruntime; print(onnxruntime.__version__)"
python3 -c "import cv2; print(cv2.__version__)"
```

---

## 3. 模型路径

默认模型路径：

```text
/mnt/yolov5/runs/train/exp3/weights/best_int8.onnx
```

如果模型路径不同，可以在 launch 时通过参数指定：

```bash
model_path:=/your/model/path/best_int8.onnx
```

---

## 4. 编译包

```bash
cd /host-home/ros2_ws

colcon build --packages-select yolov5_onnx_ros2

source install/setup.bash
```

如果修改了 Python 节点、launch 或 config 文件，需要重新编译并 source：

```bash
cd /host-home/ros2_ws

colcon build --packages-select yolov5_onnx_ros2

source install/setup.bash
```

---

## 5. 单张图片检测测试

适合测试模型是否可以正常推理。

### 5.1 准备测试图片

例如：

```bash
cp /mnt/yolov5/calib_images/001.jpg /mnt/yolov5/test.jpg
```

确认图片存在：

```bash
ls -lh /mnt/yolov5/test.jpg
```

### 5.2 运行单图检测

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=image \
  image_path:=/mnt/yolov5/test.jpg \
  save_path:=/mnt/yolov5/result.jpg
```

运行后会：

```text
1. 读取 /mnt/yolov5/test.jpg
2. 调用 YOLOv5 ONNX / INT8 ONNX 模型
3. 输出检测日志
4. 保存可视化结果到 /mnt/yolov5/result.jpg
5. 发布一次 /yolov5/detections
6. 发布一次 /yolov5/annotated_image
7. 默认检测完成后退出
```

查看结果图：

```bash
ls -lh /mnt/yolov5/result.jpg
```

如果有桌面环境，可以用图片查看器打开。
如果在容器中，可以把图片复制到宿主机查看。

---

## 6. 相机实时检测测试

### 6.1 先启动相机节点

例如 Astra 相机：

```bash
ros2 launch astra_camera dabai_u3.launch.py
```

### 6.2 查看图像话题

```bash
ros2 topic list | grep image
```

常见 RGB 图像话题可能是：

```text
/image_raw
/camera/color/image_raw
/camera/rgb/image_raw
```

查看话题类型：

```bash
ros2 topic type /camera/color/image_raw
```

正常应该是：

```text
sensor_msgs/msg/Image
```

查看频率：

```bash
ros2 topic hz /camera/color/image_raw
```

### 6.3 启动 YOLOv5 检测节点

如果相机话题是 `/camera/color/image_raw`：

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=topic \
  image_topic:=/camera/color/image_raw
```

如果图像话题就是 `/image_raw`，可以直接运行：

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py
```

---

## 7. 接上游节点输出图像

如果前面有图像预处理节点，例如输出：

```text
/processed/image
```

先确认类型：

```bash
ros2 topic type /processed/image
```

必须是：

```text
sensor_msgs/msg/Image
```

然后运行：

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=topic \
  image_topic:=/processed/image
```

整体链路示例：

```text
camera_node
    ↓
image_preprocess_node
    ↓
/processed/image
    ↓
yolov5_onnx_node
    ↓
/yolov5/detections
/yolov5/annotated_image
```

---

## 8. auto 自动模式

### 8.1 有图片路径时自动走单图模式

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=auto \
  image_path:=/mnt/yolov5/test.jpg \
  save_path:=/mnt/yolov5/result.jpg
```

如果 `/mnt/yolov5/test.jpg` 存在，节点会自动选择 `image` 模式。

### 8.2 没有图片路径时自动走话题模式

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=auto \
  image_topic:=/camera/color/image_raw
```

如果没有提供有效 `image_path`，节点会自动选择 `topic` 模式。

---

## 9. 查看检测结果

### 9.1 查看检测框消息

```bash
ros2 topic echo /yolov5/detections
```

输出格式示例：

```yaml
detections:
- bbox:
    center:
      position:
        x: 320.0
        y: 240.0
    size_x: 100.0
    size_y: 80.0
  results:
  - hypothesis:
      class_id: '0'
      score: 0.86
```

### 9.2 查看可视化图像

```bash
rqt_image_view
```

选择：

```text
/yolov5/annotated_image
```

### 9.3 查看话题列表

```bash
ros2 topic list | grep yolov5
```

正常应该有：

```text
/yolov5/detections
/yolov5/annotated_image
```

---

## 10. 参数说明

| 参数名                | 默认值                                                  | 说明                          |
| ------------------ | ---------------------------------------------------- | --------------------------- |
| `model_path`       | `/mnt/yolov5/runs/train/exp3/weights/best_int8.onnx` | ONNX / INT8 ONNX 模型路径       |
| `input_mode`       | `topic`                                              | 输入模式：`topic`、`image`、`auto` |
| `image_topic`      | `/image_raw`                                         | 实时检测订阅的图像话题                 |
| `image_path`       | 空字符串                                                 | 单图检测图片路径                    |
| `save_path`        | `/mnt/yolov5/result.jpg`                             | 单图检测结果保存路径                  |
| `exit_after_image` | `true`                                               | 单图检测完成后是否退出                 |
| `detections_topic` | `/yolov5/detections`                                 | 检测框输出话题                     |
| `annotated_topic`  | `/yolov5/annotated_image`                            | 可视化图像输出话题                   |
| `img_size`         | `640`                                                | 模型输入尺寸                      |
| `conf_thres`       | `0.25`                                               | 置信度阈值                       |
| `iou_thres`        | `0.45`                                               | NMS IoU 阈值                  |
| `num_threads`      | `1`                                                  | ONNX Runtime CPU 线程数        |

---

## 11. 修改类别名

当前模型输出维度是：

```text
6 = 5 + 1
```

说明只有 1 个类别。

默认类别名：

```yaml
class_names:
  - "target"
```

如果你的类别是 `person`，可以在 `config/yolov5_onnx.yaml` 中改成：

```yaml
class_names:
  - "person"
```

如果模型是多类别，例如 3 类，则 ONNX 输出最后一维应该是：

```text
8 = 5 + 3
```

对应配置：

```yaml
class_names:
  - "class0"
  - "class1"
  - "class2"
```

---

## 12. 修改 anchors

默认使用 YOLOv5 原始 anchors：

```yaml
anchors:
  - 10.0
  - 13.0
  - 16.0
  - 30.0
  - 33.0
  - 23.0

  - 30.0
  - 61.0
  - 62.0
  - 45.0
  - 59.0
  - 119.0

  - 116.0
  - 90.0
  - 156.0
  - 198.0
  - 373.0
  - 326.0
```

如果训练时修改过 anchors，需要同步修改这里。

可以查看训练用的模型配置：

```bash
cat /mnt/yolov5/models/yolov5s.yaml
```

或查看你自定义的模型 yaml。

---

## 13. 常见问题

### 13.1 找不到 onnxruntime

报错示例：

```text
ModuleNotFoundError: No module named 'onnxruntime'
```

解决：

```bash
pip install onnxruntime
```

确认：

```bash
python3 -c "import onnxruntime; print(onnxruntime.__version__)"
```

---

### 13.2 找不到 cv_bridge

报错示例：

```text
ModuleNotFoundError: No module named 'cv_bridge'
```

解决：

```bash
sudo apt install ros-humble-cv-bridge
```

然后重新 source：

```bash
source /opt/ros/humble/setup.bash
source /host-home/ros2_ws/install/setup.bash
```

---

### 13.3 话题没有图像

查看图像话题：

```bash
ros2 topic list | grep image
```

查看类型：

```bash
ros2 topic type /your/image/topic
```

查看频率：

```bash
ros2 topic hz /your/image/topic
```

如果没有频率，说明上游相机或图像节点没有正常发布。

---

### 13.4 rqt_image_view 看不到图像

确认是否安装：

```bash
sudo apt install ros-humble-rqt-image-view
```

运行：

```bash
rqt_image_view
```

选择：

```text
/yolov5/annotated_image
```

如果没有图像，先确认检测节点是否正在发布：

```bash
ros2 topic hz /yolov5/annotated_image
```

---

### 13.5 检测框不准

优先检查：

```text
1. class_names 是否和训练类别一致
2. anchors 是否和训练模型一致
3. img_size 是否和导出 ONNX 时一致
4. 输入图像是不是 RGB/BGR 处理错误
5. INT8 calibration 图片是否太少
```

如果 INT8 模型效果差，建议用更多真实场景图片重新量化：

```text
建议 100–300 张 calibration images
```

---

### 13.6 INT8 ONNX 可以加载但检测效果差

可能原因：

```text
1. 校准图片太少
2. 校准图片和真实部署场景差异太大
3. INT8 量化导致小目标置信度下降
4. per-channel 量化或 QDQ 格式与部署后端兼容性问题
```

建议先用 FP32 ONNX 对比：

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=image \
  model_path:=/mnt/yolov5/runs/train/exp3/weights/best.onnx \
  image_path:=/mnt/yolov5/test.jpg \
  save_path:=/mnt/yolov5/result_fp32.jpg
```

再跑 INT8：

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=image \
  model_path:=/mnt/yolov5/runs/train/exp3/weights/best_int8.onnx \
  image_path:=/mnt/yolov5/test.jpg \
  save_path:=/mnt/yolov5/result_int8.jpg
```

对比：

```text
result_fp32.jpg
result_int8.jpg
```

---

## 14. 推荐测试流程

### 第一步：测试单图

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=image \
  image_path:=/mnt/yolov5/test.jpg \
  save_path:=/mnt/yolov5/result.jpg
```

### 第二步：看结果图

```bash
ls -lh /mnt/yolov5/result.jpg
```

### 第三步：测试相机图像话题

```bash
ros2 topic list | grep image
ros2 topic hz /camera/color/image_raw
```

### 第四步：接入相机实时检测

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=topic \
  image_topic:=/camera/color/image_raw
```

### 第五步：查看输出

```bash
ros2 topic echo /yolov5/detections
```

```bash
rqt_image_view
```

选择：

```text
/yolov5/annotated_image
```

---

## 15. 后端控制节点接入

后端控制节点只需要订阅：

```text
/yolov5/detections
```

消息类型：

```text
vision_msgs/msg/Detection2DArray
```

常用字段：

```text
detections[i].bbox.center.position.x
detections[i].bbox.center.position.y
detections[i].bbox.size_x
detections[i].bbox.size_y
detections[i].results[0].hypothesis.class_id
detections[i].results[0].hypothesis.score
```

例如图像居中控制：

```text
image_center_x = image_width / 2
target_center_x = bbox.center.position.x
error_x = target_center_x - image_center_x
```

然后根据 `error_x` 控制底盘或云台转向。

MD

````

---

# 快速测试指令汇总

## 1. 编译

```bash
cd /host-home/ros2_ws
colcon build --packages-select yolov5_onnx_ros2
source install/setup.bash
````

---

## 2. 单张图片测试

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=image \
  image_path:=/mnt/yolov5/test.jpg \
  save_path:=/mnt/yolov5/result.jpg
```

查看结果：

```bash
ls -lh /mnt/yolov5/result.jpg
```

---

## 3. 相机实时测试

先查看相机话题：

```bash
ros2 topic list | grep image
```

假设相机话题是 `/camera/color/image_raw`：

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=topic \
  image_topic:=/camera/color/image_raw
```

---

## 4. 查看检测框

```bash
ros2 topic echo /yolov5/detections
```

---

## 5. 查看可视化图像

```bash
rqt_image_view
```

选择：

```text
/yolov5/annotated_image
```

---

## 6. 接前面节点输出图像

假设前面节点输出：

```text
/processed/image
```

运行：

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=topic \
  image_topic:=/processed/image
```

---

## 7. 自动模式

有图片就单图检测：

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=auto \
  image_path:=/mnt/yolov5/test.jpg \
  save_path:=/mnt/yolov5/result.jpg
```

没图片就话题检测：

```bash
ros2 launch yolov5_onnx_ros2 yolov5_onnx.launch.py \
  input_mode:=auto \
  image_topic:=/camera/color/image_raw
```
