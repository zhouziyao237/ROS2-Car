#!/usr/bin/env bash
set -e

TARGET_FRAME=${1:-oak_camera_link}

source /opt/ros/humble/setup.bash
source /host-home/install/setup.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${SCRIPT_DIR}/create_oak_runtime_params.sh"

OAK_PARAMS=/host-home/src/target_detection/oak_vision_driver/config/oak_yolo_runtime_ros_params.yaml
OAK_BLOB=/host-home/src/target_detection/oak_vision_driver/weights/best_fp16.blob

echo "Starting YOLO debug pipeline"
echo "  target frame: ${TARGET_FRAME}"
echo "  OAK params:   ${OAK_PARAMS}"
echo "  OAK blob:     ${OAK_BLOB}"

ros2 launch target_pickup_bringup yolo_debug_pipeline.launch.py \
  target_frame:=${TARGET_FRAME} \
  publish_optical_tf:=true \
  oak_params_file:=${OAK_PARAMS} \
  oak_blob_path:=${OAK_BLOB}
