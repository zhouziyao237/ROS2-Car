#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash

ros2 launch oak_vision_driver oak_vision_driver.launch.py \
  params_file:=/host-home/src/oak_vision_driver/config/oak_yolo_runtime_ros_params.yaml \
  enable_depth:=true \
  blob_path:=/host-home/src/oak_vision_driver/weights/best_fp16.blob \
  conf:=0.14 \
  iou:=0.25
