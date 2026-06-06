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


PICKUP_CODE_RE = re.compile(r"^(S\d+)-L(\d+)-([A-Za-z0-9]+)$")


class PickupCodeInputNode(Node):
    def __init__(self):
        super().__init__("pickup_code_input_node")

        self.declare_parameter("task_goal_topic", "/task/shelf_goal")
        self.declare_parameter("task_cancel_topic", "/task/cancel")
        self.declare_parameter("task_status_topic", "/task/status")

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
        self.get_logger().info("Input example: S03-L1-A")
        self.get_logger().info("Cancel command: cancel / c / stop")

    def on_status(self, msg: String):
        self.latest_status = msg.data
        print()
        print(f"[task/status] {msg.data}")
        print("> ", end="", flush=True)

    def parse_pickup_code(self, text: str):
        code = text.strip().upper()
        match = PICKUP_CODE_RE.match(code)

        if not match:
            return None

        shelf_id = match.group(1)
        row = int(match.group(2))
        col = match.group(3)
        layer_id = f"L{row}"

        return {
            "task_type": "pickup",
            "pickup_code": code,
            "shelf_id": shelf_id,
            "cell_id": code,
            "layer_id": layer_id,
            "row": row,
            "col": col,
        }

    def publish_pickup_task(self, text: str):
        task = self.parse_pickup_code(text)

        if task is None:
            print("[ERROR] 取件码格式错误，请输入类似：S03-L1-A")
            print("[提示] Backspace 删除；Ctrl+U 清空整行；c/cancel 中断任务。")
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

        self.cancel_pub.publish(msg)
        self.goal_pub.publish(msg)

        print("[OK] 已发布取消请求到 /task/cancel 和 /task/shelf_goal：")
        print(json.dumps(payload, ensure_ascii=False, indent=2))

    def print_help(self):
        print()
        print("======== 取件码输入工具 ========")
        print("输入格式：")
        print("  S03-L1-A      发布取件任务")
        print("  S02-L2-B      发布取件任务")
        print()
        print("控制命令：")
        print("  cancel / c    中断当前任务")
        print("  stop          中断任务并停止底盘")
        print("  status        显示最近一次 /task/status")
        print("  help          显示帮助")
        print("  q / quit      退出脚本")
        print()
        print("编辑按键：")
        print("  Backspace     删除前一个字符")
        print("  Ctrl+U        清空整行")
        print("================================")
        print()

    def input_loop(self):
        self.print_help()

        while self.running and rclpy.ok():
            try:
                text = input("> ").strip()
            except KeyboardInterrupt:
                print()
                self.publish_cancel("keyboard_interrupt")
                break
            except EOFError:
                break

            if not text:
                continue

            cmd = text.lower()

            if cmd in ["q", "quit", "exit"]:
                self.running = False
                break

            if cmd in ["help", "h", "?"]:
                self.print_help()
                continue

            if cmd in ["cancel", "c", "stop", "interrupt", "clear", "delete"]:
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
