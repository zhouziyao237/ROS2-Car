#!/usr/bin/env bash
set +e

echo "============================================================"
echo "[CHECK TF] map -> oak_camera_link"
echo "============================================================"
timeout 3s ros2 run tf2_ros tf2_echo map oak_camera_link

echo ""
echo "============================================================"
echo "[CHECK TF] map -> oak_camera_forward_link"
echo "============================================================"
timeout 3s ros2 run tf2_ros tf2_echo map oak_camera_forward_link

echo ""
echo "============================================================"
echo "[CHECK NODES]"
echo "============================================================"
ros2 node list | grep -E "oak_vision|oak_parcel|oak_package|robot_state|rf2o|bt_navigator|slam|amcl" || true

echo ""
echo "============================================================"
echo "[CHECK TOPICS]"
echo "============================================================"
ros2 topic list | grep -E "oak|datamatrix|package_search|cmd_vel" || true
