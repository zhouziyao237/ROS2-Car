#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash

echo "===== topic list ====="
ros2 topic list | grep -E "oak|depth|rgb|camera_info|yolo|target|pose" || true

echo ""
echo "===== depth hz ====="
timeout 5 ros2 topic hz /oak/depth/image_raw || true

echo ""
echo "===== rgb hz ====="
timeout 5 ros2 topic hz /oak/rgb/image_raw || true

echo ""
echo "===== detections info ====="
ros2 topic info /yolov5/detections || true

echo ""
echo "===== matcher info ====="
ros2 topic info /target_matcher/matched_target || true

echo ""
echo "===== target pose info ====="
ros2 topic info /target_pose_base || true
