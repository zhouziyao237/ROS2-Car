#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash

echo "Echo /target_pose_base ..."
echo "Current mode: target_frame is expected to be oak_camera_link if using 03_start_overlay_camera_frame.sh"
ros2 topic echo /target_pose_base
