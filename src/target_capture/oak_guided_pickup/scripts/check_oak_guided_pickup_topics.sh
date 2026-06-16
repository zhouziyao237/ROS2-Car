#!/usr/bin/env bash
set -euo pipefail

echo "[check] OAK guided pickup required topics"
ros2 topic list | grep -E '^/oak/datamatrix_3d_result_json$|^/platform/current_status$|^/platform/target$|^/target_capture/oak_guided_pickup/request$|^/target_capture/oak_guided_pickup/status$' || true

echo

echo "[hint] status echo:"
echo "  ros2 topic echo /target_capture/oak_guided_pickup/status"

echo "[hint] platform status echo:"
echo "  ros2 topic echo /platform/current_status"
