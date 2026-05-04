# target_matcher

`target_matcher` 是一个用于机器人视觉目标识别、OCR 目标码匹配、目标锁定、RGB 红框显示、深度融合定位以及到位后校准使能控制的 ROS2 包。

当前推荐的正式运行模式是：

```text
SEARCHING
低频 OCR 寻找目标码
↓
CANDIDATE_FOUND
连续稳定识别到目标码
↓
APPROACHING
锁定目标，停止重复 OCR，只继续计算位置
↓
LOCK_LOST_HOLD
目标短暂消失时保持锁定，不切换到相邻目标
↓
ARRIVED
机器人到达目标附近
↓
CALIBRATION_ENABLED
发布校准使能，允许校准节点开始运行
````

---

## 1. 功能概述

本系统用于解决以下问题：

```text
1. YOLO 检测目标框
2. 对检测框区域发送 OCR 请求
3. OCR 识别标签编号，例如 2-3-2050
4. 与目标码 target_code 进行匹配
5. 连续稳定识别后锁定目标 track_id
6. 锁定后停止重复 OCR，降低 CPU 占用
7. 目标短暂消失时保持锁定，避免被相邻相似目标抢占
8. 在 RGB 图像上将匹配目标框标红
9. 将匹配目标框与深度图融合，计算 3D 坐标
10. 机器人到位后发布 calibration_enable，允许校准流程启动
```

---

## 2. 节点组成

### 2.1 target_matcher_node

主节点，负责：

```text
- 订阅 YOLO 检测结果
- 发送 OCR ROI 请求
- 接收 OCR 识别结果
- 匹配目标码
- 管理 track slot
- 锁定目标 track_id
- 管理识别状态机
- 发布 matched_target
- 发布 calibration_enable
```

### 2.2 matched_overlay_node

图像叠加节点，负责：

```text
- 订阅 RGB 图像
- 订阅 /target_matcher/matched_target
- 匹配成功时将目标框画成红色
- 超时后自动清除旧红框
```

输出图像：

```text
/target_matcher/matched_overlay_image
```

### 2.3 matched_depth_locator_node

深度融合节点，负责：

```text
- 订阅 /target_matcher/matched_target
- 订阅深度图 /camera/depth/image_raw
- 订阅相机内参 /camera/rgb/camera_info
- 根据目标框中心附近的深度值计算 3D 坐标
- 发布 /target_matcher/matched_point
- 发布 /target_matcher/matched_depth_text
```

---

## 3. 推荐工作流

```text
1. 启动相机、YOLO、OCR、target_matcher、overlay、depth locator
2. 设置目标码 target_code，例如 2-3-2050
3. 系统进入 SEARCHING
4. YOLO 检测目标框
5. target_matcher 按限流策略发送 OCR 请求
6. OCR 返回文本
7. target_matcher 判断文本是否匹配 target_code
8. 连续稳定匹配后锁定目标
9. 系统进入 APPROACHING
10. 锁定后不再重复 OCR，只继续输出 matched_target 和 matched_point
11. 机器人根据 matched_point 移动到目标附近
12. 到位后发布 /target_matcher/arrived=true
13. 系统进入 CALIBRATION_ENABLED
14. 发布 /target_matcher/calibration_enable=true
15. 校准节点开始运行
```

---

## 4. 识别模式 recognition_mode

系统支持三种识别模式。

### 4.1 continuous

持续识别模式。

```text
特点：
- YOLO 持续运行
- OCR 按限流策略持续运行
- 适合长时间寻找目标
- CPU 占用较高
```

切换命令：

```bash
ros2 topic pub --once /target_matcher/recognition_mode std_msgs/msg/String "{data: 'continuous'}"
```

---

### 4.2 lock_then_pose

推荐正式模式。

```text
特点：
- 找目标时运行 OCR
- 连续稳定识别后锁定目标
- 锁定后停止重复 OCR
- 只继续输出目标位置和深度坐标
- 可以明显降低 CPU 占用
- 可以避免目标短暂消失时被相邻相似目标抢占
```

切换命令：

```bash
ros2 topic pub --once /target_matcher/recognition_mode std_msgs/msg/String "{data: 'lock_then_pose'}"
```

---

### 4.3 pose_only

只算位置模式。

```text
特点：
- 完全不再发送 OCR 请求
- 只保留已锁定目标的位置输出
- 适合已经确认目标后进一步降低 CPU 占用
```

切换命令：

```bash
ros2 topic pub --once /target_matcher/recognition_mode std_msgs/msg/String "{data: 'pose_only'}"
```

---

## 5. 状态机说明

### 5.1 IDLE

未启用识别，或没有目标码。

常见原因：

```text
- target_code 为空
- tracking_enabled=false
```

---

### 5.2 SEARCHING

正在寻找目标码。

此时系统会：

```text
- 接收 YOLO 检测框
- 对稳定目标发送 OCR 请求
- 等待 OCR 匹配 target_code
```

---

### 5.3 CANDIDATE_FOUND

找到候选目标，但还未满足稳定锁定条件。

例如：

```text
stable_match_required_count: 2
```

代表需要连续稳定匹配 2 次后才真正锁定。

---

### 5.4 APPROACHING

目标已经锁定。

此时系统会：

```text
- 固定 locked_track_id
- 不允许其他相似目标抢占
- 在 lock_then_pose 模式下停止重复 OCR
- 持续发布 matched_target
- 持续发布 matched_point
```

---

### 5.5 LOCK_LOST_HOLD

锁定目标暂时消失。

此时系统不会马上切换到其他目标，而是等待：

```yaml
lock_lost_timeout_sec: 5.0
```

如果目标在 5 秒内重新出现，继续跟踪原目标。

---

### 5.6 ARRIVED_NO_LOCK

机器人到位了，但当前没有锁定目标。

这种情况下不会开启校准。

---

### 5.7 CALIBRATION_ENABLED

机器人已经到位，并且已经锁定目标。

此时系统会发布：

```text
/target_matcher/calibration_enable = true
```

校准节点应该监听这个 topic，只有为 true 时才开始校准。

---

## 6. Topic 说明

### 6.1 输入 Topic

| Topic                              | 类型                                | 说明                |
| ---------------------------------- | --------------------------------- | ----------------- |
| `/yolov5/detections`               | `yolo_msg/msg/YoloDetectionArray` | YOLO 检测结果         |
| `/vision/ocr_results`              | `ocr_msg/msg/OcrResultArray`      | OCR 识别结果          |
| `/target_code`                     | `std_msgs/msg/String`             | 目标码，例如 `2-3-2050` |
| `/target_matcher/enable`           | `std_msgs/msg/Bool`               | 是否启用目标匹配          |
| `/target_matcher/arrived`          | `std_msgs/msg/Bool`               | 机器人是否已经到位         |
| `/target_matcher/reset_lock`       | `std_msgs/msg/Bool`               | 手动重置目标锁定          |
| `/target_matcher/recognition_mode` | `std_msgs/msg/String`             | 切换识别模式            |
| `/camera/depth/image_raw`          | `sensor_msgs/msg/Image`           | 深度图               |
| `/camera/rgb/camera_info`          | `sensor_msgs/msg/CameraInfo`      | RGB 相机内参          |

---

### 6.2 输出 Topic

| Topic                                   | 类型                                          | 说明                   |
| --------------------------------------- | ------------------------------------------- | -------------------- |
| `/vision/ocr_requests`                  | `ocr_msg/msg/OcrRequestArray`               | 发送给 OCR 节点的 ROI 请求   |
| `/target_matcher/matched_target`        | `target_matcher_msg/msg/MatchedTarget`      | 当前匹配目标               |
| `/target_matcher/tracked_targets`       | `target_matcher_msg/msg/TrackedTargetArray` | 当前所有 track 目标        |
| `/target_matcher/matched_index`         | `std_msgs/msg/Int32`                        | 匹配目标 index，未匹配为 `-1` |
| `/target_matcher/matched_detection`     | `yolo_msg/msg/YoloDetectionArray`           | 匹配目标检测框              |
| `/target_matcher/tracked_detections`    | `yolo_msg/msg/YoloDetectionArray`           | 所有跟踪检测框              |
| `/target_matcher/matched_overlay_image` | `sensor_msgs/msg/Image`                     | 红框叠加后的 RGB 图         |
| `/target_matcher/matched_point`         | `geometry_msgs/msg/PointStamped`            | 匹配目标 3D 坐标           |
| `/target_matcher/matched_depth_text`    | `std_msgs/msg/String`                       | 目标 3D 坐标文本           |
| `/target_matcher/state`                 | `std_msgs/msg/String`                       | 当前状态机状态              |
| `/target_matcher/calibration_enable`    | `std_msgs/msg/Bool`                         | 是否允许校准               |

---

## 7. 推荐 target_matcher.yaml

路径：

```bash
/host-home/src/target_matcher/config/target_matcher.yaml
```

完整配置：

```yaml
target_matcher_node:
  ros__parameters:
    # =====================
    # Input topics
    # =====================
    yolo_topic: /yolov5/detections
    ocr_request_topic: /vision/ocr_requests
    ocr_result_topic: /vision/ocr_results
    target_code_topic: /target_code
    enable_topic: /target_matcher/enable

    # =====================
    # Backward-compatible outputs
    # =====================
    matched_index_topic: /target_matcher/matched_index
    matched_detection_topic: /target_matcher/matched_detection
    tracked_detections_topic: /target_matcher/tracked_detections
    debug_text_topic: /target_matcher/debug_text

    # =====================
    # Main outputs
    # =====================
    tracked_targets_topic: /target_matcher/tracked_targets
    matched_target_topic: /target_matcher/matched_target

    # =====================
    # Tracking parameters
    # =====================
    track_iou_threshold: 0.12
    track_center_distance_threshold: 1.0
    max_missing_frames: 25

    # YOLO 框过滤阈值
    # 如果漏检严重，降到 0.35
    # 如果误框很多，升到 0.45
    min_yolo_confidence: 0.40

    # 调试模式才打开
    # true  = 有检测框就 OCR
    # false = 正式模式，只在有 target_code 时按策略 OCR
    only_need_ocr: false

    # =====================
    # Recognition mode
    # =====================
    # continuous:
    #   长时间持续识别模式，YOLO + OCR 低频运行
    #
    # lock_then_pose:
    #   推荐正式模式
    #   识别到目标码并锁定后，不再重复 OCR，只保留目标位置输出
    #
    # pose_only:
    #   完全关闭 OCR，只使用已经锁定的目标做位置输出
    recognition_mode: lock_then_pose
    recognition_mode_topic: /target_matcher/recognition_mode

    # =====================
    # OCR policy
    # =====================
    ocr_min_confidence: 0.35

    # OCR 重试间隔
    # 不要太小，否则 OCR 队列容易爆
    ocr_retry_interval_sec: 4.0

    # 成功 OCR 后的缓存时间
    ocr_cache_timeout_sec: 60.0

    # OCR 请求发出后，等待结果的最大时间
    # OCR 慢时不要设太小
    ocr_pending_timeout_sec: 8.0

    # 同一个 track 成功 OCR 后不重复 OCR
    avoid_repeat_ocr: true

    # =====================
    # OCR ROI
    # =====================
    # 1.10 比较稳
    # 如果只识别到一部分编号，例如只出 2050，可以改 1.15
    # 如果读到旁边文字，例如“更多”“5元”，改 1.05
    ocr_roi_expand_scale: 1.10

    image_width: 640
    image_height: 480

    # =====================
    # OCR throttling
    # =====================
    # 每帧最多发几个 OCR 请求
    max_ocr_requests_per_frame: 1

    # track 至少稳定几帧才 OCR
    # 可以过滤瞬时误检
    min_track_age_for_ocr: 4

    # =====================
    # Lock state machine
    # =====================
    state_topic: /target_matcher/state
    arrived_topic: /target_matcher/arrived
    reset_lock_topic: /target_matcher/reset_lock
    calibration_enable_topic: /target_matcher/calibration_enable

    # 连续稳定匹配几次后锁定目标
    stable_match_required_count: 2

    # 锁定目标短暂消失时，等待多久才真正释放锁
    # 这可以防止目标短暂离开视野时被旁边相似目标抢占
    lock_lost_timeout_sec: 5.0

    # 机器已经到位并开始校准后，即使视觉目标短暂丢失，也保持校准使能
    hold_calibration_after_arrived: true

    # =====================
    # Runtime control
    # =====================
    reuse_track_indices: true

    # 默认 false
    # 当 target_code 传入时，如果 auto_enable_on_target_code=true，会自动开启
    tracking_enabled: false

    auto_enable_on_target_code: true

    # 没有目标码时不发 OCR
    require_target_code_for_ocr: true

    publish_debug_text: true

    # 可以为空，推荐通过 launch 参数或 /target_code topic 设置
    target_code: ""
```

---

## 8. 推荐 ocr_realtime.yaml

路径：

```bash
/host-home/src/ocr_node/config/ocr_realtime.yaml
```

推荐配置：

```yaml
ocr_node:
  ros__parameters:
    # =====================
    # Basic mode
    # =====================
    input_mode: topic
    image_path: ""
    save_path: ""
    exit_after_image: true

    # =====================
    # Topics
    # =====================
    image_topic: /camera/rgb/image_unmirrored
    request_topic: /vision/ocr_requests
    result_topic: /vision/ocr_results
    debug_image_topic: /vision/ocr_debug_image

    # =====================
    # Models
    # =====================
    det_model: weights/det_static_qdq_int8.onnx
    cls_model: weights/cls_static_qdq_int8.onnx
    rec_model: weights/rec_fp32.onnx
    rec_yml: weights/inference.yml

    # =====================
    # Text detection
    # =====================
    # 736 性能好
    # 小字识别不稳时改成 960
    det_limit_side_len: 736
    det_db_thresh: 0.30
    det_db_box_thresh: 0.55
    det_db_unclip_ratio: 1.5
    det_min_size: 5

    # =====================
    # Result filtering
    # =====================
    drop_score: 0.35

    # =====================
    # Text recognition
    # =====================
    rec_img_h: 48
    rec_max_img_w: 320
    rec_use_dynamic_width: false
    rec_batch_size: 2

    # =====================
    # Direction classifier
    # =====================
    use_cls: false
    cls_thresh: 0.9

    # =====================
    # Crop
    # =====================
    crop_padding_ratio: 0.06

    # =====================
    # ROI preprocess
    # 如果 ocr_node.py 已经实现这些参数，可以提高小字稳定性
    # 如果还没实现，保留也不会影响核心运行
    # =====================
    enable_roi_preprocess: true
    roi_preprocess_min_short_side: 96
    roi_preprocess_max_scale: 2.0
    roi_preprocess_try_variants: true
    roi_preprocess_use_clahe: true
    roi_preprocess_use_sharpen: true
    roi_preprocess_use_binary: false
    roi_preprocess_clahe_clip_limit: 2.0
    roi_preprocess_clahe_tile_grid_size: 8
    roi_preprocess_adaptive_block_size: 31
    roi_preprocess_adaptive_c: 7

    # =====================
    # Threads
    # =====================
    ort_num_threads: 1
    det_num_threads: 1
    cls_num_threads: 1
    rec_num_threads: 1

    # =====================
    # Debug
    # =====================
    publish_debug_image: false
    save_debug_image: false
    log_recognized_text: false

    # =====================
    # ROI filter
    # =====================
    min_roi_width: 35
    min_roi_height: 15

    # =====================
    # Image sync / OCR queue
    # =====================
    image_cache_size: 30
    max_image_time_diff_ms: 300.0
    use_nearest_image_if_exact_stamp_missing: true
    ocr_queue_size: 8
```

---

## 9. 编译

普通编译：

```bash
cd /host-home
source /opt/ros/humble/setup.bash

colcon build --packages-select target_matcher yolov5_node ocr_node

source install/setup.bash
```

如果改了 Python 入口、launch 或 setup.py，建议清理重编译：

```bash
cd /host-home
source /opt/ros/humble/setup.bash

rm -rf build/target_matcher install/target_matcher build/ocr_node install/ocr_node log

colcon build --packages-select target_matcher ocr_node

source install/setup.bash
```

---

## 10. 推荐启动命令

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch target_matcher target_tracking.launch.py \
  use_camera:=true \
  use_rviz:=true \
  use_locator:=false \
  use_image_flip:=true \
  use_matched_overlay:=true \
  use_depth_locator:=true \
  raw_image_topic:=/camera/rgb/image_raw \
  image_topic:=/camera/rgb/image_unmirrored \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/rgb/camera_info \
  mirror_x_for_depth:=true \
  matched_stale_timeout_sec:=0.8 \
  target_code:=2-3-2050 \
  yolo_num_threads:=1 \
  yolo_conf_thres:=0.35 \
  ocr_queue_size:=8 \
  ocr_image_cache_size:=30 \
  ocr_max_diff_ns:=300000000
```

如果目标漏检，降低 YOLO 阈值：

```bash
yolo_conf_thres:=0.30
```

如果误框很多，提高 YOLO 阈值：

```bash
yolo_conf_thres:=0.40
```

---

## 11. 启动后检查参数

### 检查 matcher 参数

```bash
ros2 param get /target_matcher_node recognition_mode
ros2 param get /target_matcher_node only_need_ocr
ros2 param get /target_matcher_node min_yolo_confidence
ros2 param get /target_matcher_node max_ocr_requests_per_frame
ros2 param get /target_matcher_node min_track_age_for_ocr
ros2 param get /target_matcher_node ocr_retry_interval_sec
ros2 param get /target_matcher_node ocr_pending_timeout_sec
ros2 param get /target_matcher_node stable_match_required_count
ros2 param get /target_matcher_node lock_lost_timeout_sec
```

推荐结果：

```text
String value is: lock_then_pose
Boolean value is: False
Double value is: 0.4
Integer value is: 1
Integer value is: 4
Double value is: 4.0
Double value is: 8.0
Integer value is: 2
Double value is: 5.0
```

---

### 检查 OCR 参数

```bash
ros2 param get /ocr_node image_cache_size
ros2 param get /ocr_node max_image_time_diff_ms
ros2 param get /ocr_node ocr_queue_size
```

推荐结果：

```text
Integer value is: 30
Double value is: 300.0
Integer value is: 8
```

如果还是：

```text
image_cache_size=20
ocr_queue_size=3
```

说明 `ocr.launch.py` 没有正确把 launch 参数传给 OCR 节点，或者 install 里还是旧版本。

---

## 12. 运行中常用命令

### 设置目标码

```bash
ros2 topic pub --once /target_code std_msgs/msg/String "{data: '2-3-2050'}"
```

---

### 查看状态机

```bash
ros2 topic echo /target_matcher/state
```

---

### 查看最终匹配目标

```bash
ros2 topic echo /target_matcher/matched_target
```

匹配成功时会看到：

```yaml
matched: true
target_code: 2-3-2050
matched_token: 2-3-2050
```

---

### 查看 3D 坐标

```bash
ros2 topic echo /target_matcher/matched_point
```

或者：

```bash
ros2 topic echo /target_matcher/matched_depth_text
```

---

### 查看校准使能

```bash
ros2 topic echo /target_matcher/calibration_enable
```

---

### 查看红框图像

```bash
ros2 run rqt_image_view rqt_image_view
```

选择：

```text
/target_matcher/matched_overlay_image
```

---

## 13. 机器到位后开启校准

当机器人已经运行到目标附近后，发布：

```bash
ros2 topic pub --once /target_matcher/arrived std_msgs/msg/Bool "{data: true}"
```

如果当前已经锁定目标，系统会进入：

```text
CALIBRATION_ENABLED
```

并发布：

```yaml
/target_matcher/calibration_enable:
  data: true
```

校准节点应该监听：

```text
/target_matcher/calibration_enable
```

只有收到 `true` 后才开始校准。

---

## 14. 重置目标锁定

如果需要重新寻找目标：

```bash
ros2 topic pub --once /target_matcher/reset_lock std_msgs/msg/Bool "{data: true}"
```

系统会清空锁定目标并回到：

```text
SEARCHING
```

---

## 15. 切换识别模式

### 持续识别

```bash
ros2 topic pub --once /target_matcher/recognition_mode std_msgs/msg/String "{data: 'continuous'}"
```

### 锁定后只算位置

```bash
ros2 topic pub --once /target_matcher/recognition_mode std_msgs/msg/String "{data: 'lock_then_pose'}"
```

### 完全停止 OCR，只算已锁定目标位置

```bash
ros2 topic pub --once /target_matcher/recognition_mode std_msgs/msg/String "{data: 'pose_only'}"
```

---

## 16. 图像翻转与深度融合

当前推荐：

```bash
use_image_flip:=true
image_topic:=/camera/rgb/image_unmirrored
mirror_x_for_depth:=true
```

原因：

```text
YOLO 和 OCR 使用的是水平翻转后的 RGB 图像
depth 图通常仍然是原始坐标
所以深度融合时需要 mirror_x_for_depth=true
```

如果关闭图像翻转：

```bash
use_image_flip:=false
image_topic:=/camera/rgb/image_raw
```

则必须改成：

```bash
mirror_x_for_depth:=false
```

否则红框和深度取点会左右相反。

---

## 17. 常见问题

### 17.1 OCR task queue full

现象：

```text
OCR task queue full, dropping oldest request batch
```

原因：

```text
OCR 请求太多
OCR 队列太小
OCR pending timeout 太短导致反复重发
```

解决：

```yaml
max_ocr_requests_per_frame: 1
min_track_age_for_ocr: 4
ocr_retry_interval_sec: 4.0
ocr_pending_timeout_sec: 8.0
ocr_queue_size: 8
```

---

### 17.2 No synchronized image found for OCR request

现象：

```text
No synchronized image found for OCR request
```

原因：

```text
OCR 请求时间戳和缓存图像时间戳差距太大
image_cache_size 太小
max_image_time_diff_ms 太小
```

解决：

```yaml
image_cache_size: 30
max_image_time_diff_ms: 300.0
use_nearest_image_if_exact_stamp_missing: true
```

---

### 17.3 目标短暂消失后匹配到旁边目标

原因：

```text
没有锁定目标 track_id
或者锁丢失后立即重新匹配了相邻目标
```

解决：

```yaml
recognition_mode: lock_then_pose
stable_match_required_count: 2
lock_lost_timeout_sec: 5.0
```

---

### 17.4 CPU 占用过高

优先使用：

```yaml
recognition_mode: lock_then_pose
avoid_repeat_ocr: true
max_ocr_requests_per_frame: 1
min_track_age_for_ocr: 4
ocr_retry_interval_sec: 4.0
ocr_pending_timeout_sec: 8.0
```

启动时关闭 RViz：

```bash
use_rviz:=false
```

识别到目标后，可以切换为：

```bash
ros2 topic pub --once /target_matcher/recognition_mode std_msgs/msg/String "{data: 'pose_only'}"
```

---

### 17.5 红框不显示

红框只在 `/target_matcher/matched_target` 里：

```yaml
matched: true
```

时才显示。

检查：

```bash
ros2 topic echo /target_matcher/matched_target
```

如果一直是：

```yaml
matched: false
```

说明 OCR 还没有稳定匹配目标码。

---

### 17.6 深度坐标不输出

检查：

```bash
ros2 topic echo /target_matcher/matched_point
ros2 topic echo /target_matcher/matched_depth_text
```

如果没有输出，检查：

```text
1. /target_matcher/matched_target 是否 matched=true
2. /camera/depth/image_raw 是否有数据
3. /camera/rgb/camera_info 是否有数据
4. mirror_x_for_depth 是否设置正确
```

---

## 18. 推荐调试顺序

```text
1. 先看 YOLO 是否有框
2. 再看 /vision/ocr_requests 是否有请求
3. 再看 /vision/ocr_results 是否有文字
4. 再看 /target_matcher/state 是否进入 CANDIDATE_FOUND
5. 再看 /target_matcher/matched_target 是否 matched=true
6. 再看 /target_matcher/matched_overlay_image 是否有红框
7. 再看 /target_matcher/matched_point 是否有 3D 坐标
8. 最后发布 /target_matcher/arrived=true 测试校准使能
```

---

## 19. 最终推荐使用方式

正式运行建议保持：

```yaml
recognition_mode: lock_then_pose
```

因为它可以实现：

```text
找目标时允许 OCR
锁定后停止 OCR
只保留位置计算
降低 CPU
避免相邻目标抢占
到位后才开启校准
```
