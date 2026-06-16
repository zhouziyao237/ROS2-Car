#!/usr/bin/env bash
set +e

echo "============================================================"
echo "[STEP 1] Stop chassis command"
echo "============================================================"

for i in 1 2 3; do
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
  sleep 0.1
done

echo "[OK] Published zero cmd_vel"

echo ""
echo "============================================================"
echo "[STEP 2] Try lifecycle shutdown for Nav2"
echo "============================================================"

for node in \
  /lifecycle_manager_navigation \
  /bt_navigator \
  /behavior_server \
  /smoother_server
do
  echo "[TRY] ros2 lifecycle set $node shutdown"
  ros2 lifecycle set "$node" shutdown >/dev/null 2>&1 || true
done

echo ""
echo "============================================================"
echo "[STEP 3] Stop known robot ROS processes"
echo "============================================================"

PATTERN="mission_btcpp_node|semantic_marker_publisher_node|semantic_registrar_node|bt_osd_overlay|goal_pose_bridge|home_anchor_validator_node|task_cancel_guard_node|rf2o_odom_adapter|lslidar_driver_node|robot_state_publisher|bt_navigator|behavior_server|smoother_server|lifecycle_manager_navigation|apriltag_shelf_bridge_node|cmd_vel_mode_mux|frontier_explorer_node|vision_qr_depth_node|depth_align_node|nav2|rviz2"

echo "[INFO] Sending SIGINT..."
pkill -INT -f "$PATTERN" || true
sleep 2

echo "[INFO] Sending SIGTERM to remaining..."
pkill -TERM -f "$PATTERN" || true
sleep 1

echo ""
echo "============================================================"
echo "[STEP 4] Stop ros2 launch parents if still alive"
echo "============================================================"

pkill -INT -f "ros2 launch" || true
sleep 1
pkill -TERM -f "ros2 launch" || true

echo ""
echo "============================================================"
echo "[STEP 5] Current ROS2 nodes"
echo "============================================================"

ros2 node list || true

echo ""
echo "[DONE] ROS nodes cleanup command finished."
