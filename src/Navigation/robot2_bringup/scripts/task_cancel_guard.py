#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from typing import List

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path


class TaskCancelGuard(Node):
    def __init__(self):
        super().__init__("task_cancel_guard_node")

        self.declare_parameter("task_goal_topic", "/task/shelf_goal")
        self.declare_parameter("task_cancel_topic", "/task/cancel")
        self.declare_parameter("task_status_topic", "/task/status")

        self.declare_parameter("cmd_vel_topics", [
            "/cmd_vel_selected",
            "/cmd_vel",
            "/cmd_vel_nav",
        ])

        self.declare_parameter("path_clear_topics", [
            "/plan",
            "/local_plan",
            "/received_global_plan",
            "/transformed_global_plan",
            "/unsmoothed_plan",
        ])

        # 安全改动：
        # 默认不再接受 /task/shelf_goal 里面的 cancel JSON。
        # /task/shelf_goal 只应该放任务目标，取消只走 /task/cancel。
        self.declare_parameter("allow_cancel_on_task_goal", False)

        # 允许明确的 raw cancel 字符串，例如 "cancel"、"stop"。
        # 空字符串和未知字符串不会再被当成 cancel。
        self.declare_parameter("allow_raw_cancel", True)

        self.declare_parameter("cancel_repeat_sec", 2.5)
        self.declare_parameter("zero_vel_hz", 20.0)
        self.declare_parameter("clear_path_repeat_sec", 3.0)
        self.declare_parameter("clear_path_hz", 20.0)

        # 只取消 Nav2 相关 action，不再全场扫描取消 camera calibration 等无关 action。
        self.declare_parameter("cancel_action_services", [
            "/navigate_to_pose/_action/cancel_goal",
            "/navigate_through_poses/_action/cancel_goal",
            "/follow_waypoints/_action/cancel_goal",
            "/follow_path/_action/cancel_goal",
            "/compute_path_to_pose/_action/cancel_goal",
            "/compute_path_through_poses/_action/cancel_goal",
            "/smooth_path/_action/cancel_goal",
            "/backup/_action/cancel_goal",
            "/wait/_action/cancel_goal",
        ])

        self.goal_sub = self.create_subscription(
            String,
            self.get_parameter("task_goal_topic").value,
            self.on_goal_or_cancel,
            20,
        )

        self.cancel_sub = self.create_subscription(
            String,
            self.get_parameter("task_cancel_topic").value,
            self.on_cancel,
            20,
        )

        self.status_pub = self.create_publisher(
            String,
            self.get_parameter("task_status_topic").value,
            20,
        )

        self.cmd_pubs = []
        for topic in self.get_parameter("cmd_vel_topics").value:
            self.cmd_pubs.append(self.create_publisher(Twist, str(topic), 10))

        self.path_pubs = []
        for topic in self.get_parameter("path_clear_topics").value:
            self.path_pubs.append(self.create_publisher(Path, str(topic), 10))

        self.canceling = False
        self.last_cancel_time = 0.0

        self.get_logger().info("task_cancel_guard_node started")
        self.get_logger().info("Listening explicit cancel on /task/cancel")
        self.get_logger().info("Cancel JSON on /task/shelf_goal is ignored unless allow_cancel_on_task_goal=true")

    def parse_json(self, text: str):
        try:
            data = json.loads(text)
            if isinstance(data, dict):
                return data
        except Exception:
            pass
        return {}

    def is_cancel_payload(self, data: dict):
        task_type = str(data.get("task_type", "")).lower()
        command = str(data.get("command", "")).lower()
        status = str(data.get("status", "")).lower()

        return (
            task_type == "cancel"
            or command == "cancel"
            or status in ["cancel_requested", "canceled", "cancelled"]
        )

    def raw_cancel_payload(self, text: str):
        raw = str(text).strip().lower()
        if not raw:
            return None

        allowed = {
            "cancel",
            "c",
            "stop",
            "interrupt",
            "operator_interrupt",
            "emergency_stop",
        }

        if raw in allowed:
            return {
                "task_type": "cancel",
                "command": "cancel",
                "reason": raw,
                "raw": raw,
            }

        return None

    def on_goal_or_cancel(self, msg: String):
        data = self.parse_json(msg.data)

        if self.is_cancel_payload(data):
            if bool(self.get_parameter("allow_cancel_on_task_goal").value):
                self.handle_cancel(data, source="/task/shelf_goal")
            else:
                self.get_logger().warn(
                    "Ignored cancel-like payload on /task/shelf_goal. "
                    "Use /task/cancel for cancel."
                )

    def on_cancel(self, msg: String):
        text = msg.data
        data = self.parse_json(text)

        if data:
            if self.is_cancel_payload(data):
                self.handle_cancel(data, source="/task/cancel")
            else:
                self.get_logger().warn(f"Ignored non-cancel JSON on /task/cancel: {text}")
            return

        if bool(self.get_parameter("allow_raw_cancel").value):
            raw_data = self.raw_cancel_payload(text)
            if raw_data is not None:
                self.handle_cancel(raw_data, source="/task/cancel")
                return

        self.get_logger().warn(f"Ignored invalid/raw non-cancel message on /task/cancel: {text!r}")

    def publish_status(self, state: str, status: str, message: str, extra=None):
        payload = {
            "state": state,
            "status": status,
            "message": message,
            "stamp": time.time(),
            "source": "task_cancel_guard_node",
        }

        if isinstance(extra, dict):
            payload.update(extra)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def publish_zero_velocity_once(self):
        zero = Twist()
        for pub in self.cmd_pubs:
            pub.publish(zero)

    def publish_zero_velocity_for(self, duration_sec: float):
        hz = float(self.get_parameter("zero_vel_hz").value)
        period = 1.0 / max(hz, 1.0)
        end_time = time.time() + duration_sec

        while time.time() < end_time and rclpy.ok():
            self.publish_zero_velocity_once()
            time.sleep(period)

    def publish_empty_paths_once(self):
        stamp = self.get_clock().now().to_msg()

        empty_path = Path()
        empty_path.header.stamp = stamp
        empty_path.header.frame_id = "map"
        empty_path.poses = []

        for pub in self.path_pubs:
            pub.publish(empty_path)

    def clear_rviz_paths_for(self, duration_sec: float):
        hz = float(self.get_parameter("clear_path_hz").value)
        period = 1.0 / max(hz, 1.0)
        end_time = time.time() + duration_sec

        self.get_logger().info("publishing empty Path messages to clear RViz path displays")

        while time.time() < end_time and rclpy.ok():
            self.publish_empty_paths_once()
            time.sleep(period)

    def discover_cancel_services(self) -> List[str]:
        services = self.get_service_names_and_types()
        available = set()

        for name, types in services:
            if name.endswith("/_action/cancel_goal"):
                if "action_msgs/srv/CancelGoal" in types:
                    available.add(name)

        configured = [str(x) for x in self.get_parameter("cancel_action_services").value]

        ordered = []
        for name in configured:
            if name in available and name not in ordered:
                ordered.append(name)

        return ordered

    def call_cancel_service(self, service_name: str) -> bool:
        client = self.create_client(CancelGoal, service_name)

        if not client.wait_for_service(timeout_sec=0.25):
            self.get_logger().warn(f"cancel service not available: {service_name}")
            return False

        req = CancelGoal.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

        if future.done() and future.result() is not None:
            self.get_logger().info(
                f"cancel action sent: {service_name}, return_code={future.result().return_code}"
            )
            return True

        self.get_logger().warn(f"cancel action timeout: {service_name}")
        return False

    def cancel_all_actions(self):
        services = self.discover_cancel_services()

        if not services:
            self.get_logger().warn("no configured Nav2 action cancel service found")
            return 0

        self.get_logger().info("found Nav2 action cancel services:")
        for s in services:
            self.get_logger().info(f"  - {s}")

        ok = 0
        for s in services:
            if self.call_cancel_service(s):
                ok += 1

        return ok

    def clear_costmap(self, service_name: str) -> bool:
        client = self.create_client(ClearEntireCostmap, service_name)

        if not client.wait_for_service(timeout_sec=0.25):
            return False

        req = ClearEntireCostmap.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)

        return future.done() and future.result() is not None

    def clear_costmaps(self):
        services = [
            "/global_costmap/clear_entirely_global_costmap",
            "/local_costmap/clear_entirely_local_costmap",
        ]

        ok = 0
        for s in services:
            if self.clear_costmap(s):
                self.get_logger().info(f"cleared costmap: {s}")
                ok += 1
            else:
                self.get_logger().warn(f"costmap clear unavailable or timeout: {s}")

        return ok

    def handle_cancel(self, data: dict, source: str):
        now = time.time()

        if self.canceling and now - self.last_cancel_time < 0.5:
            return

        self.canceling = True
        self.last_cancel_time = now

        reason = str(data.get("reason", "operator_interrupt"))

        self.publish_status(
            state="TASK_CANCELING",
            status="RUNNING",
            message=f"cancel requested from {source}",
            extra={"reason": reason},
        )

        self.publish_zero_velocity_for(0.5)
        self.clear_rviz_paths_for(0.5)

        ok_cancel = self.cancel_all_actions()

        duration = float(self.get_parameter("cancel_repeat_sec").value)
        self.publish_zero_velocity_for(duration)

        ok_clear = self.clear_costmaps()

        clear_path_duration = float(self.get_parameter("clear_path_repeat_sec").value)
        self.clear_rviz_paths_for(clear_path_duration)

        self.publish_status(
            state="TASK_CANCELED",
            status="CANCELED",
            message="task canceled by operator",
            extra={
                "reason": reason,
                "cancel_action_count": ok_cancel,
                "clear_costmap_count": ok_clear,
            },
        )

        self.canceling = False


def main(args=None):
    rclpy.init(args=args)
    node = TaskCancelGuard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
