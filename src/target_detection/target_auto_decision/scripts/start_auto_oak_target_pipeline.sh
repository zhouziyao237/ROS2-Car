#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash

ros2 launch target_auto_decision auto_oak_target_pipeline.launch.py \
  start_oak:=true \
  start_matcher:=false \
  start_overlay:=true \
  start_decision:=true \
  target_frame:=oak_camera_link \
  detections_topic:=/yolov5/detections \
  depth_topic:=/oak/depth/image_raw \
  camera_info_topic:=/oak/rgb/camera_info
