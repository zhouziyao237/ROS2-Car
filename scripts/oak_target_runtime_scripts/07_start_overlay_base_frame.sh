#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash

# Use this only after running 06_start_temp_base_tf.sh or after URDF TF exists.
ros2 launch target_pose_bridge single_camera_target_overlay.launch.py \
  image_topic:=/oak/rgb/image_raw \
  detections_topic:=/target_matcher/matched_target \
  depth_topic:=/oak/depth/image_raw \
  camera_info_topic:=/oak/rgb/camera_info \
  target_frame:=base_link \
  publish_optical_tf:=true \
  camera_link_frame:=oak_camera_link \
  camera_optical_frame:=oak_rgb_camera_optical_frame
