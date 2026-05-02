# ocr_node

ROS 2 Humble Python OCR package for PP-OCRv5 ONNXRuntime inference.

这个包不引入官方 PaddleOCR runtime，而是继续使用 ONNXRuntime 跑 PP-OCRv5 模型，并尽量复刻 PaddleOCR 的 DB 后处理、逐行输出和返回风格。

这个包只负责 OCR 文字提取，不负责 YOLO 检测、不负责正则匹配、不负责目标内容匹配，也不负责深度融合。

当前实现流程：

```text
RGB image + OcrRequestArray
        ↓
按 target_id 裁剪 YOLO ROI
        ↓
ROI 内做 OCR det
        ↓
每个文字行单独做 cls + rec
        ↓
发布 OcrResultArray
```

## 功能

- 支持 `topic` / `image` / `auto` 三种输入模式
- 订阅 RGB 图像和 `/vision/ocr_requests`
- 一个 `OcrRequest` 对应一个 ROI
- 一个 ROI 内可以输出多条 OCR 行结果
- 同一个 `target_id` 可以在 `OcrResultArray.results` 中出现多次，通过 `line_id` 区分
- 每个 `OcrResult` 只对应一个文字行，不会把多行合并成一个 `raw_text`
- 返回风格尽量接近 PaddleOCR 的 `[box, ("text", score)]`
- 支持发布或保存 OCR debug 图像
- 支持用 YAML 控制是否打印识别内容

节点名：

```text
ocr_node
```

可执行文件：

```text
ocr_node
```

## 依赖

- ROS 2 Humble
- `cv_bridge`
- `sensor_msgs`
- `target_msgs/yolo_msg`
- `target_msgs/ocr_msg`
- Python: `onnxruntime`, `opencv-python`, `numpy`, `pyyaml`, `shapely`, `pyclipper`

安装常用依赖：

```bash
sudo apt update
sudo apt install -y ros-humble-cv-bridge python3-opencv
pip install onnxruntime numpy pyyaml shapely pyclipper
```

如果只补齐 DB 后处理需要的扩框依赖，至少安装：

```bash
python3 -m pip install shapely pyclipper
```

## 权重文件

默认从包内 `weights/` 读取：

```text
weights/det_static_qdq_int8.onnx
weights/cls_static_qdq_int8.onnx
weights/rec_static_qdq_int8.onnx
weights/rec_fp32.onnx
weights/inference.yml
```

如果仓库里没有实际权重文件，需要先放进去。推荐优先使用 `det/cls` 保持 INT8，同时把 `rec_model` 切到 `rec_fp32.onnx`，再配合 batch rec 提升整体性价比。

## 输入输出

### 订阅

- `image_topic`
  - 默认：`/camera/rgb/image_raw`
  - 类型：`sensor_msgs/msg/Image`
- `request_topic`
  - 默认：`/vision/ocr_requests`
  - 类型：`ocr_msg/msg/OcrRequestArray`

`OcrRequest` 主要字段：

- `target_id`
- `class_name`
- `yolo_confidence`
- `need_ocr`
- `box`

### 发布

- `result_topic`
  - 默认：`/vision/ocr_results`
  - 类型：`ocr_msg/msg/OcrResultArray`
- `debug_image_topic`
  - 默认：`/vision/ocr_debug_image`
  - 类型：`sensor_msgs/msg/Image`

`OcrResult` 主要字段：

- `target_id`
- `line_id`
- `raw_text`
- `confidence`
- `indices`
- `has_text`
- `roi_box`
- `text_box`

这意味着同一个目标可以返回多行，例如：

```text
target_id = armor_0, line_id = 0
target_id = armor_0, line_id = 1
target_id = armor_0, line_id = 2
```

节点不会在这里做：

- 正则匹配
- 目标编号匹配
- 深度融合
- 目标内容比对

## 编译

在工作区根目录执行：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
colcon build --packages-select yolo_msg ocr_msg ocr_node
source install/setup.bash
```

## 启动

### 1. 单张图片 OCR

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ocr_node ocr.launch.py \
  input_mode:=image \
  image_path:=/host-home/src/ocr_node/test/test.png \
  save_path:=/host-home/src/ocr_node/test/result.png
```

默认行为：

- 读取整张图片
- 整图作为一个 dummy ROI
- 先做 det，再逐行做 cls + rec
- 日志按逐行格式输出，例如 `[line 0] text=05/16, score=0.9600`
- 保存 debug 图到 `save_path`
- 发布一次 `OcrResultArray`
- 默认自动退出

### 2. topic 模式

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ocr_node ocr.launch.py
```

然后确保：

- `/camera/rgb/image_raw` 正在发 RGB 图像
- `/vision/ocr_requests` 正在发 `ocr_msg/msg/OcrRequestArray`

### 3. auto 模式

```bash
ros2 launch ocr_node ocr.launch.py \
  input_mode:=auto \
  image_path:=/host-home/src/ocr_node/test/test.png
```

如果 `image_path` 有效，就走单图模式；否则走 topic 模式。

## 手工测试 OCR 请求

如果你已经有相机图像，可以运行测试脚本：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/ocr_node/scripts/test_pub_request.py
```

它会向 `/vision/ocr_requests` 发布一个整图 ROI 请求。

## 查看结果

查看 OCR 结果：

```bash
ros2 topic echo /vision/ocr_results
```

查看 OCR debug 图：

```bash
ros2 run rqt_image_view rqt_image_view
```

然后选择：

```text
/vision/ocr_debug_image
```

## 日志开关

可以通过参数 `log_recognized_text` 控制是否打印识别出的具体文本。

在 [config/ocr.yaml](/host-home/src/ocr_node/config/ocr.yaml:1) 中默认值例如：

```yaml
log_recognized_text: false
```

启动时也可以覆盖：

```bash
ros2 launch ocr_node ocr.launch.py \
  input_mode:=image \
  image_path:=/host-home/src/ocr_node/test/test.png \
  save_path:=/host-home/src/ocr_node/test/result.png \
  log_recognized_text:=true
```

`true` 时会打印每一行识别文本；`false` 时只保留简要完成信息。

## 主要参数

配置文件在 [config/ocr.yaml](/host-home/src/ocr_node/config/ocr.yaml:1)。

常用参数：

```yaml
input_mode: topic
image_path: ""
save_path: ""
exit_after_image: true

image_topic: /camera/rgb/image_raw
request_topic: /vision/ocr_requests
result_topic: /vision/ocr_results
debug_image_topic: /vision/ocr_debug_image

det_model: weights/det_static_qdq_int8.onnx
cls_model: weights/cls_static_qdq_int8.onnx
rec_model: weights/rec_fp32.onnx
rec_yml: weights/inference.yml

det_limit_side_len: 960
det_db_thresh: 0.3
det_db_box_thresh: 0.6
det_db_unclip_ratio: 1.5
det_min_size: 3
drop_score: 0.5

rec_batch_size: 6
rec_img_h: 48
rec_max_img_w: 320
rec_use_dynamic_width: false

crop_padding_ratio: 0.08
use_cls: false
cls_thresh: 0.9

det_num_threads: 1
cls_num_threads: 1
rec_num_threads: 4
publish_debug_image: false
save_debug_image: false
log_recognized_text: false

min_roi_width: 30
min_roi_height: 15

image_cache_size: 10
max_image_time_diff_ms: 80.0
use_nearest_image_if_exact_stamp_missing: true

ocr_queue_size: 8
```

推荐先把 `use_cls` 设为 `false`，这样默认不跑方向分类，每个 ROI 内的多条文字会一起做 batch rec。若现场文字可能倒置，再把 `use_cls` 改回 `true`。

launch 文件在 [launch/ocr.launch.py](/host-home/src/ocr_node/launch/ocr.launch.py:1)。

## 与 YOLO 的关系

`ocr_node` 不直接订阅 `/yolov5/detections`。

通常链路应该是：

```text
RGB image
  -> yolov5_node
  -> /yolov5/detections
  -> 桥接/融合节点
  -> /vision/ocr_requests
  -> ocr_node
  -> /vision/ocr_results
```

也就是说，中间需要一个节点把检测框转成 `OcrRequestArray`。

## 常见问题

### 1. OCR 结果结构是对的，但文字不准

这通常先从两类问题排查：

- 后处理参数和官方差异过大
- `rec` 模型精度不够，尤其是短数字串或细字

当前节点已经尽量贴近 PaddleOCR 的 DB 后处理、batch rec 和 CTC decode。如果还不够接近，下一步优先测试 `det INT8 + cls INT8 + rec FP32 ONNX`。

当前这套是通用 PP-OCR 模型，更适合常规文本，不一定适合：

- 手写数字
- 稀疏排版
- 特殊符号
- 只包含少量数字的短串

### 2. 看不到 `/vision/ocr_results`

先确认：

- 是否已经真的发了 `/vision/ocr_requests`
- 请求时间戳附近是否有对应 RGB 图像
- 是否已经 `source /host-home/install/setup.bash`

### 3. 单图模式退出时报错

当前实现已经处理了安全退出；如果你看到旧报错，通常是没有重新编译或没有重新 `source`。
