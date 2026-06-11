#!/usr/bin/env python3
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
        # AUTO_RESET_CONTROL_MODE_BEFORE_PICKUP_TASK_20260611_MULTILINE
        self.control_mode_pub = self.create_publisher(
            String,
            '/control_mode',
            10
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
        # AUTO_RESET_CONTROL_MODE_BEFORE_PICKUP_TASK_20260611_MULTILINE
        try:
            auto_msg = String()
            auto_msg.data = 'auto'
            self.control_mode_pub.publish(auto_msg)
        except Exception:
            pass
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
