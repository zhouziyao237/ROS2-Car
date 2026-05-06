#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash

# Temporary static TF when you do not have URDF.
# Change x y z according to current lift/camera pose relative to base_link.
# Format: x y z roll pitch yaw parent child
ros2 run tf2_ros static_transform_publisher \
  0.100 0.000 0.300 \
  0.000 0.000 0.000 \
  base_link oak_camera_link
