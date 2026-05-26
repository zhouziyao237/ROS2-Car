#!/usr/bin/env bash
set -e

CODE=${1:-3412}
MATCHED_TOPIC=${2:-/target_matcher/matched_detection}
TARGET_FRAME=${3:-oak_camera_link}

source /opt/ros/humble/setup.bash
source /host-home/install/setup.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${SCRIPT_DIR}/create_oak_runtime_params.sh"

OAK_PARAMS=/host-home/src/target_detection/oak_vision_driver/config/oak_yolo_runtime_ros_params.yaml
OAK_BLOB=/host-home/src/target_detection/oak_vision_driver/weights/best_fp16.blob

echo "Starting full pickup pipeline"
echo "  initial target code: ${CODE}"
echo "  matched topic:       ${MATCHED_TOPIC}"
echo "  target frame:        ${TARGET_FRAME}"
echo "  OAK params:          ${OAK_PARAMS}"
echo "  OAK blob:            ${OAK_BLOB}"

ros2 launch target_pickup_bringup full_pickup_pipeline.launch.py \
  target_code:=${CODE} \
  auto_publish_target_code:=false \
  start_oak:=true \
  start_ocr:=true \
  start_matcher:=true \
  start_overlay:=true \
  start_decision:=true \
  detections_topic:=${MATCHED_TOPIC} \
  target_frame:=${TARGET_FRAME} \
  publish_optical_tf:=true \
  oak_params_file:=${OAK_PARAMS} \
  oak_blob_path:=${OAK_BLOB}
