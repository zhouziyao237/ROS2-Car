#!/usr/bin/env bash
set -e
TARGET_CODE="${1:-4309}"
TARGET_FRAME="${2:-oak_camera_link}"
cd /host-home
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash
OAK_RUNTIME_PARAMS="/host-home/src/target_detection/oak_vision_driver/config/oak_yolo_runtime_ros_params.yaml"
OAK_CONFIG="/host-home/src/target_detection/oak_vision_driver/config/yolov5_oak.yaml"
OAK_BLOB="/host-home/src/target_detection/oak_vision_driver/weights/best_fp16.blob"
cat > "${OAK_RUNTIME_PARAMS}" <<EOF
oak_vision_driver_node:
  ros__parameters:
    config_path: ${OAK_CONFIG}
EOF
echo "Starting QR pickup pipeline"
echo "  target code: ${TARGET_CODE}"
echo "  target frame: ${TARGET_FRAME}"
ros2 launch target_pickup_bringup qr_pickup_pipeline.launch.py \
  target_code:="${TARGET_CODE}" \
  target_frame:="${TARGET_FRAME}" \
  oak_params_file:="${OAK_RUNTIME_PARAMS}" \
  oak_blob_path:="${OAK_BLOB}"
