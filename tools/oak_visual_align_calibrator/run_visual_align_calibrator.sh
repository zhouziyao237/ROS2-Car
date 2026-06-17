#!/usr/bin/env bash
set -eo pipefail

# 启动 OAK 升降视觉对齐标定工具。
# 注意：OAK/OpenCV 原始图像仍是左上角原点；本工具把竖直标定保存为 bottom-left：
#   align_v_origin: bottom_left
#   align_v_from_bottom_px: 距离画面底部的像素
# 同时保存 align_v_px = image_height_px - align_v_from_bottom_px，兼容旧字段。

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
  -p config_yaml_path:=/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml \
  -p install_config_yaml_path:=/host-home/install/oak_guided_pickup/share/oak_guided_pickup/config/oak_guided_pickup.yaml \
  -p image_height_px:=1080.0 \
  -p align_v_from_bottom_px:=300.0
