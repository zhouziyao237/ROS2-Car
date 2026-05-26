#!/usr/bin/env bash
set -e

CODE=$1

if [ -z "$CODE" ]; then
  echo "Usage: ./change_target_code.sh <target_code>"
  exit 1
fi

source /opt/ros/humble/setup.bash
source /host-home/install/setup.sh

echo "Switching target code to: ${CODE}"

echo "[1/8] disable matcher"
ros2 topic pub --once /target_matcher/enable std_msgs/msg/Bool "{data: false}" || true
sleep 0.5

echo "[2/8] reset old lock"
ros2 topic pub --once /target_matcher/reset_lock std_msgs/msg/Bool "{data: true}" || true
sleep 0.5

echo "[3/8] clear downstream matched_detection"
ros2 topic pub --once /target_matcher/matched_detection yolo_msg/msg/YoloDetectionArray "{
header:
  frame_id: 'oak_rgb_camera_optical_frame',
detections: []
}" || true
sleep 0.5

echo "[4/8] set lock_then_pose mode"
ros2 topic pub --once /target_matcher/recognition_mode std_msgs/msg/String "{data: 'lock_then_pose'}" || true
sleep 0.5

echo "[5/8] publish new target code"
ros2 topic pub --once /target_code std_msgs/msg/String "{data: '${CODE}'}"
sleep 1.0

echo "[6/8] arrived=true"
ros2 topic pub --once /target_matcher/arrived std_msgs/msg/Bool "{data: true}" || true
sleep 0.5

echo "[7/8] enable matcher"
ros2 topic pub --once /target_matcher/enable std_msgs/msg/Bool "{data: true}" || true
sleep 0.5

echo "[8/8] publish target code again after enable"
ros2 topic pub --once /target_code std_msgs/msg/String "{data: '${CODE}'}"

echo "Target code switched to: ${CODE}"
