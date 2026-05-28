#!/usr/bin/env bash
set -e

echo "=== /scan ==="
timeout 5 ros2 topic hz /scan || true

echo "=== /scan_front ==="
timeout 5 ros2 topic hz /scan_front || true

echo "=== /odom ==="
timeout 5 ros2 topic hz /odom || true

echo "=== /map ==="
timeout 5 ros2 topic hz /map || true

echo "=== TF odom -> base_link ==="
timeout 5 ros2 run tf2_ros tf2_echo odom base_link || true

echo "=== TF map -> base_link ==="
timeout 5 ros2 run tf2_ros tf2_echo map base_link || true
