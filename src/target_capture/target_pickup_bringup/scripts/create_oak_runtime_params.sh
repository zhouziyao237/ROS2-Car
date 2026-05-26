#!/usr/bin/env bash
set -e

OAK_CONFIG=${OAK_CONFIG:-/host-home/src/target_detection/oak_vision_driver/config/yolov5_oak.yaml}
OUT_FILE=${OUT_FILE:-/host-home/src/target_detection/oak_vision_driver/config/oak_yolo_runtime_ros_params.yaml}

mkdir -p "$(dirname "$OUT_FILE")"

cat > "$OUT_FILE" <<YAML
oak_vision_driver_node:
  ros__parameters:
    config_path: $OAK_CONFIG
YAML

echo "Created: $OUT_FILE"
echo "Using OAK config: $OAK_CONFIG"
cat "$OUT_FILE"
