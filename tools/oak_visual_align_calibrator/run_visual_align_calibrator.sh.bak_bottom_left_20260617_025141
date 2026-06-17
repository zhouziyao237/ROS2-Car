#!/usr/bin/env bash
set -eo pipefail

# 这个脚本用于启动 OAK RGB 对齐点标定工具。
# 它会读取 OAK RGB 图像，在画面中显示 align_u_px / align_v_px 十字线，
# 支持鼠标点击、滑条微调，并可按 s 保存到 oak_guided_pickup.yaml。

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f /host-home/install/setup.bash ]; then
  source /host-home/install/setup.bash
fi

ros2 run oak_guided_pickup calibrate_visual_align_point --ros-args \
  -p rgb_image_topic:=/oak/rgb/image_raw \
  -p datamatrix_topic:=/oak/datamatrix_3d_result_json \
  -p visual_align_point_topic:=/oak/visual_align_point \
  -p config_yaml_path:=/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml
