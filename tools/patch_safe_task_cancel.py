#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import os
import subprocess

STAMP = time.strftime("%Y%m%d_%H%M%S")

PICKUP_CODE_INPUT = r'''#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import re
import readline
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


try:
    readline.parse_and_bind('"\\C-?": backward-delete-char')
    readline.parse_and_bind('"\\C-h": backward-delete-char')
    readline.parse_and_bind('"\\e[3~": delete-char')
except Exception:
    pass


class PickupCodeInputNode(Node):
    def __init__(self):
        super().__init__("pickup_code_input_node")

        self.declare_parameter("task_goal_topic", "/task/shelf_goal")
        self.declare_parameter("task_cancel_topic", "/task/cancel")
        self.declare_parameter("task_status_topic", "/task/status")

        # 默认不再把 cancel 同时发到 /task/shelf_goal，避免误触发任务节点。
        self.declare_parameter("publish_cancel_to_goal_topic", False)

        self.goal_pub = self.create_publisher(
            String,
            self.get_parameter("task_goal_topic").value,
            10,
        )

        self.cancel_pub = self.create_publisher(
            String,
            self.get_parameter("task_cancel_topic").value,
            10,
        )

        self.status_sub = self.create_subscription(
            String,
            self.get_parameter("task_status_topic").value,
            self.on_status,
            20,
        )

        self.latest_status = ""
        self.running = True

        self.get_logger().info("pickup_code_input_node started")
        self.get_logger().info("Input example: S03-L1-0301")
        self.get_logger().info("Cancel command: cancel / c / stop")
        self.get_logger().info("Ctrl+C now exits this input tool only, and will NOT cancel task.")

    def on_status(self, msg: String):
        self.latest_status = msg.data
        print()
        print(f"[task/status] {msg.data}")
        print("> ", end="", flush=True)

    def parse_pickup_code(self, text: str):
        raw = str(text).strip().upper()

        norm = raw.replace("_", "-").replace("/", "-")
        norm = re.sub(r"\s+", "-", norm)
        norm = re.sub(r"[^A-Z0-9-]+", "-", norm)
        norm = re.sub(r"-+", "-", norm).strip("-")

        m = re.search(r"([A-Z0-9]{1,4})-(L[0-9]{1,3})-([A-Z0-9]{4})", norm)
        if not m:
            raise ValueError(
                f"取件码格式错误：{text}，应为类似 S03-L1-0301 的完整取件码"
            )

        shelf_id = m.group(1)
        layer_id = m.group(2)
        package_code = m.group(3)
        pickup_code = f"{shelf_id}-{layer_id}-{package_code}"

        try:
            row = int(layer_id[1:])
        except Exception:
            row = 0

        return {
            "task_type": "pickup",
            "pickup_code": pickup_code,
            "shelf_id": shelf_id,
            "cell_id": pickup_code,
            "layer_id": layer_id,
            "package_code": package_code,
            "row": row,
            "col": package_code,
        }

    def publish_pickup_task(self, text: str):
        try:
            task = self.parse_pickup_code(text)
        except Exception as e:
            print(f"[ERROR] {e}")
            print("[提示] 示例：S03-L1-0301；输入 cancel/c/stop 才会中断任务。")
            return

        msg = String()
        msg.data = json.dumps(task, ensure_ascii=False)
        self.goal_pub.publish(msg)

        print("[OK] 已发布取件任务到 /task/shelf_goal：")
        print(json.dumps(task, ensure_ascii=False, indent=2))

    def publish_cancel(self, reason: str = "operator_interrupt"):
        payload = {
            "command": "cancel",
            "task_type": "cancel",
            "status": "CANCEL_REQUESTED",
            "reason": reason,
            "stamp": time.time(),
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)

        # 只发布到 /task/cancel。
        # 不再默认发布到 /task/shelf_goal，避免任务目标 topic 被 cancel JSON 污染。
        self.cancel_pub.publish(msg)

        if bool(self.get_parameter("publish_cancel_to_goal_topic").value):
            self.goal_pub.publish(msg)

        print("[OK] 已发布取消请求到 /task/cancel：")
        print(json.dumps(payload, ensure_ascii=False, indent=2))

    def print_help(self):
        print()
        print("======== 取件码输入工具 ========")
        print("完整取件码格式：<货架编号>-<层号>-<4位取件码>，例如 S03-L1-0301")
        print()
        print("输入格式：")
        print("  S03-L1-0301      发布取件任务")
        print("  S03-L2-0311      发布取件任务")
        print()
        print("控制命令：")
        print("  cancel / c    中断当前任务")
        print("  stop          中断任务并停止底盘")
        print("  status        显示最近一次 /task/status")
        print("  help          显示帮助")
        print("  q / quit      退出输入工具，不取消任务")
        print()
        print("重要：Ctrl+C 现在只退出输入工具，不会取消任务。")
        print("================================")
        print()

    def input_loop(self):
        self.print_help()

        while self.running and rclpy.ok():
            try:
                text = input("> ").strip()
            except KeyboardInterrupt:
                print()
                print("[EXIT] Ctrl+C 只退出输入工具，不发送 /task/cancel。")
                self.running = False
                break
            except EOFError:
                print()
                print("[EXIT] EOF 退出输入工具，不发送 /task/cancel。")
                self.running = False
                break

            if not text:
                continue

            cmd = text.lower()

            if cmd in ["q", "quit", "exit"]:
                print("[EXIT] 退出输入工具，不取消任务。")
                self.running = False
                break

            if cmd in ["help", "h", "?"]:
                self.print_help()
                continue

            if cmd in ["cancel", "c", "stop", "interrupt"]:
                self.publish_cancel("operator_interrupt")
                continue

            if cmd in ["status", "s"]:
                print("[latest /task/status]")
                print(self.latest_status if self.latest_status else "(暂无状态)")
                continue

            self.publish_pickup_task(text)

        print("pickup_code_input_node exited")


def main(args=None):
    rclpy.init(args=args)
    node = PickupCodeInputNode()

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spin_thread.start()

    try:
        node.input_loop()
    finally:
        node.running = False
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
'''

TASK_CANCEL_GUARD = r'''#!/usr/bin/env python3
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
'''


def write_file(path: Path, content: str):
    if path.exists():
        bak = path.with_suffix(path.suffix + f".bak_safe_cancel_{STAMP}")
        shutil.copy2(path, bak)
        print(f"[BAK] {path} -> {bak}")
    else:
        path.parent.mkdir(parents=True, exist_ok=True)

    path.write_text(content, encoding="utf-8")
    os.chmod(path, 0o755)
    print(f"[OK] wrote {path}")


targets = [
    (Path("/host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py"), PICKUP_CODE_INPUT),
    (Path("/host-home/install/robot2_bringup/lib/robot2_bringup/pickup_code_input.py"), PICKUP_CODE_INPUT),
    (Path("/host-home/src/Navigation/robot2_bringup/scripts/task_cancel_guard.py"), TASK_CANCEL_GUARD),
    (Path("/host-home/install/robot2_bringup/lib/robot2_bringup/task_cancel_guard.py"), TASK_CANCEL_GUARD),
]

for path, content in targets:
    write_file(path, content)

print("")
print("[CHECK] py_compile")
for path, _ in targets:
    r = subprocess.run(["python3", "-m", "py_compile", str(path)])
    if r.returncode != 0:
        raise SystemExit(f"[FAIL] py_compile failed: {path}")
    print(f"[OK] {path}")

print("")
print("[DONE] safe cancel patch applied.")
