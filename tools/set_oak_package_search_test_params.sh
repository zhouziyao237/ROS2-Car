#!/usr/bin/env bash
set -e

MODE="${1:-dry}"

echo "============================================================"
echo "[OAK PACKAGE SEARCH PARAM SETUP]"
echo "mode=$MODE"
echo "============================================================"

until ros2 node list | grep -qx "/oak_package_search_node"; do
  echo "waiting for /oak_package_search_node..."
  sleep 0.5
done

if [ "$MODE" = "real" ]; then
  ENABLE_MOTION=true
else
  ENABLE_MOTION=false
fi

ros2 param set /oak_package_search_node enable_motion "$ENABLE_MOTION"

# Same policy as future main BT integration:
# request -> align -> scan start -> scan -> match.
ros2 param set /oak_package_search_node match_only_in_scan true
ros2 param set /oak_package_search_node ignore_detection_before_request true
ros2 param set /oak_package_search_node min_confirm_matches 2
ros2 param set /oak_package_search_node min_search_time_before_match_sec 0.5

# Search timing.
ros2 param set /oak_package_search_node scan_timeout_sec 25.0
ros2 param set /oak_package_search_node scan_range_deg 25.0
ros2 param set /oak_package_search_node scan_speed_wz 0.10

# OAK yaw alignment control.
ros2 param set /oak_package_search_node align_yaw_tolerance_deg 3.0
ros2 param set /oak_package_search_node kp_wz 1.0
ros2 param set /oak_package_search_node min_wz 0.06
ros2 param set /oak_package_search_node max_wz 0.25

echo "============================================================"
echo "[DONE]"
echo "enable_motion=$ENABLE_MOTION"
echo "============================================================"
