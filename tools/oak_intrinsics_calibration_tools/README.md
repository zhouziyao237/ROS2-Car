# OAK Lite RGB 高清内参标定工具

这个工具用于标定 OAK Lite 的 RGB 高清图像内参，适合你当前的使用方式：

- RGB 高清图：`/oak/rgb/image_raw`，1920x1080
- Depth 图：`/oak/depth/image_raw`，通常 640x640
- `oak_parcel_pose` 使用 RGB 内参 `fx/fy/cx/cy` 和 depth 得到的 Z 反投影出 `camera_xyz_mm`

> 注意：内参必须在你实际使用的 RGB 分辨率下标定。你现在用 `video_width:=1920 video_height:=1080`，所以就要在 1920x1080 模式下采集棋盘格图片并标定。

---

## 1. 准备棋盘格

建议使用 OpenCV 标准棋盘格：

- 内角点数量：`9 x 6`
- 每个格子边长：建议 `25 mm` 或 `30 mm`
- 打印后贴在硬纸板或亚克力板上，保持完全平整

这里的 `9 x 6` 指的是**内角点数量**，不是黑白格子的数量。比如常见 10 x 7 个方格，对应 9 x 6 个内角点。

---

## 2. 启动 OAK 高清 RGB

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch oak_vision_driver oak_vision_driver.launch.py \
  enable_rgb:=true \
  enable_depth:=true \
  enable_yolo:=false \
  enable_stereo_fusion:=false \
  sensor_resolution:=1080p \
  rgb_output:=video \
  video_width:=1920 \
  video_height:=1080 \
  fps:=10.0 \
  force_usb2:=true
```

---

## 3. 采集标定图片

在另一个终端运行：

```bash
cd /host-home
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 /host-home/tools/oak_intrinsics_calibration_tools/capture_chessboard_ros2.py \
  --image-topic /oak/rgb/image_raw \
  --output-dir /host-home/calib_oak_rgb_1080p \
  --cols 9 \
  --rows 6 \
  --square-mm 25 \
  --auto \
  --max-images 40 \
  --min-interval 0.8
```

如果你的棋盘格是 30 mm，就把 `--square-mm 25` 改成 `--square-mm 30`。

采集建议：

- 至少 30 张，推荐 40–60 张
- 棋盘格要覆盖画面中心、左上、右上、左下、右下
- 有正对，也要有轻微倾斜
- 不要严重反光，不要模糊
- 不要只在画面中心采集，否则边缘畸变标不准

---

## 4. 计算内参

```bash
python3 /host-home/tools/oak_intrinsics_calibration_tools/calibrate_intrinsics.py \
  --image-dir /host-home/calib_oak_rgb_1080p \
  --cols 9 \
  --rows 6 \
  --square-mm 25 \
  --output /host-home/calib_oak_rgb_1080p/oak_rgb_1080p_intrinsics.yaml
```

输出文件里会有：

```yaml
fx: ...
fy: ...
cx: ...
cy: ...
dist_coeffs: [k1, k2, p1, p2, k3]
reprojection_error_px: ...
```

`reprojection_error_px` 建议小于 0.5–0.8 px。若大于 1.0 px，说明图片质量、棋盘格尺寸、内角点数量或采集姿态可能有问题。

---

## 5. 给 oak_parcel_pose 使用

如果你的 `oak_parcel_pose` 已经支持手动内参，可以这样传：

```bash
ros2 launch oak_parcel_pose oak_parcel_pose.launch.py \
  rgb_topic:=/oak/rgb/image_raw \
  depth_topic:=/oak/depth/image_raw \
  camera_info_topic:=/oak/rgb/camera_info \
  use_camera_info:=false \
  manual_fx:=你的fx \
  manual_fy:=你的fy \
  manual_cx:=你的cx \
  manual_cy:=你的cy
```

如果你的当前版本还不支持 `use_camera_info/manual_fx/manual_fy/manual_cx/manual_cy`，需要把这些参数加进 `oak_parcel_pose_node.py` 和 launch 文件。

---

## 6. 为什么要标 RGB 内参

你的 Data Matrix 中心点来自 RGB 高清图，所以 X/Y 反投影应该使用 RGB 相机内参：

```text
X = (u_rgb - cx) * Z / fx
Y = (v_rgb - cy) * Z / fy
Z = depth_mm
```

这里的 `Z` 来自 depth 图，但 `u_rgb/v_rgb/fx/fy/cx/cy` 都属于 RGB 相机。

