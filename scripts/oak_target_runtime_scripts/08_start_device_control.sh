#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash

ros2 launch target_device_control target_device_control.launch.py \
  target_pose_topic:=/target_pose_base \
  desired_x_mm:=350.0 \
  pusher_extend_mm:=120.0
