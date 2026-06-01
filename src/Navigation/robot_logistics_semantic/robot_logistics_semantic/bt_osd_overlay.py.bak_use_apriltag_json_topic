#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from typing import Dict, Any, Optional

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


def parse_json(text: str) -> Dict[str, Any]:
    try:
        v = json.loads(text)
        if isinstance(v, dict):
            return v
    except Exception:
        pass
    return {}


def short_json(text: str, max_len: int = 110) -> str:
    d = parse_json(text)
    if d:
        parts = []
        for k in ["state", "status", "message", "shelf_id", "layer_id", "pickup_code"]:
            if k in d and d[k] not in ["", None]:
                parts.append(f"{k}={d[k]}")
        s = " | ".join(parts) if parts else text
    else:
        s = text

    if len(s) > max_len:
        s = s[:max_len - 3] + "..."
    return s


class BtOsdOverlay(Node):
    """
    统一 OSD：
      - 原图：/camera/camera/color/image_raw
      - 输出：/task/bt_osd/image_raw
      - 显示：任务状态、front_qr 写入状态、HOME 状态、AprilTag 识别结果

    只使用旧文件 bt_osd_overlay.py，不新增节点文件。
    """

    def __init__(self):
        super().__init__("bt_osd_overlay")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_image_topic", "/task/bt_osd/image_raw")

        self.declare_parameter("task_status_topic", "/task/status")
        self.declare_parameter("front_writer_status_topic", "/semantic/front_qr_writer_status")
        self.declare_parameter("home_status_topic", "/semantic/home_anchor_status")
        self.declare_parameter("apriltag_observation_topic", "/vision/apriltag_semantic_observation")

        self.declare_parameter("keep_sec", 3.0)
        self.declare_parameter("draw_apriltag", True)
        self.declare_parameter("draw_status", True)

        self.bridge = CvBridge()

        self.latest_task = ("", 0.0)
        self.latest_front = ("", 0.0)
        self.latest_home = ("", 0.0)
        self.latest_tag = ({}, 0.0)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            sensor_qos,
        )

        self.task_sub = self.create_subscription(
            String,
            str(self.get_parameter("task_status_topic").value),
            self.on_task_status,
            20,
        )

        self.front_sub = self.create_subscription(
            String,
            str(self.get_parameter("front_writer_status_topic").value),
            self.on_front_status,
            20,
        )

        self.home_sub = self.create_subscription(
            String,
            str(self.get_parameter("home_status_topic").value),
            self.on_home_status,
            20,
        )

        self.tag_sub = self.create_subscription(
            String,
            str(self.get_parameter("apriltag_observation_topic").value),
            self.on_tag_obs,
            50,
        )

        # 关键：输出图像用 Reliable，RViz 默认 Reliable 可以直接看到。
        self.image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("output_image_topic").value),
            10,
        )

        self.get_logger().info(
            "bt_osd_overlay started: "
            f"{self.get_parameter('image_topic').value} -> {self.get_parameter('output_image_topic').value}"
        )

    def now(self) -> float:
        return time.time()

    def on_task_status(self, msg: String):
        self.latest_task = (msg.data, self.now())

    def on_front_status(self, msg: String):
        self.latest_front = (msg.data, self.now())

    def on_home_status(self, msg: String):
        self.latest_home = (msg.data, self.now())

    def on_tag_obs(self, msg: String):
        d = parse_json(msg.data)
        if d:
            self.latest_tag = (d, self.now())

    def draw_box_text(self, img, x: int, y: int, text: str, color=(255, 255, 255)):
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.5
        thickness = 1

        h, w = img.shape[:2]
        text = str(text)

        (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
        x = max(0, min(x, w - tw - 8))
        y = max(th + 8, min(y, h - 8))

        cv2.rectangle(img, (x, y - th - 7), (x + tw + 8, y + 5), (0, 0, 0), -1)
        cv2.putText(img, text, (x + 4, y), font, scale, color, thickness, cv2.LINE_AA)

    def draw_status_lines(self, img):
        if not bool(self.get_parameter("draw_status").value):
            return

        keep = float(self.get_parameter("keep_sec").value)
        now = self.now()

        lines = []

        if self.latest_task[0] and now - self.latest_task[1] <= keep:
            lines.append(("TASK " + short_json(self.latest_task[0]), (255, 255, 255)))

        if self.latest_front[0] and now - self.latest_front[1] <= keep:
            lines.append(("FRONT " + short_json(self.latest_front[0]), (0, 255, 0)))

        if self.latest_home[0] and now - self.latest_home[1] <= keep:
            lines.append(("HOME " + short_json(self.latest_home[0]), (255, 0, 255)))

        y = 24
        for text, color in lines[:5]:
            self.draw_box_text(img, 10, y, text, color)
            y += 24

    def draw_tag(self, img):
        if not bool(self.get_parameter("draw_apriltag").value):
            return

        keep = float(self.get_parameter("keep_sec").value)
        tag, t = self.latest_tag

        if not tag or self.now() - t > keep:
            return

        raw = tag.get("raw", tag.get("display_code", ""))
        tag_id = tag.get("tag_id", "")
        typ = tag.get("type", tag.get("obs_type", ""))
        shelf = tag.get("shelf_id", "")
        layer = tag.get("layer_id", "")
        face = tag.get("face", "")

        label = f"TAG {raw} id={tag_id} {typ} {shelf} {layer} {face}"

        corners = tag.get("corners", None)
        if isinstance(corners, list) and len(corners) >= 4:
            try:
                pts = []
                for p in corners:
                    pts.append((int(float(p[0])), int(float(p[1]))))

                color = (0, 255, 255)
                for i in range(len(pts)):
                    cv2.line(img, pts[i], pts[(i + 1) % len(pts)], color, 2)

                cx = int(sum(p[0] for p in pts) / len(pts))
                cy = int(sum(p[1] for p in pts) / len(pts))
                cv2.drawMarker(img, (cx, cy), color, cv2.MARKER_CROSS, 16, 2)
                self.draw_box_text(img, cx + 8, cy - 8, label, color)
                return
            except Exception:
                pass

        # 没有 corners 就在右上角显示文字
        h, w = img.shape[:2]
        self.draw_box_text(img, 10, h - 20, label, (0, 255, 255))

    def on_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {repr(e)}")
            return

        self.draw_status_lines(img)
        self.draw_tag(img)

        try:
            out = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            out.header = msg.header
            self.image_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"osd publish failed: {repr(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = BtOsdOverlay()
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
