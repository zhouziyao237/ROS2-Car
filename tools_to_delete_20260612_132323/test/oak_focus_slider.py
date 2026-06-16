#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import re
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


BRINGUP = Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml")


class OakFocusSlider(Node):
    def __init__(self):
        super().__init__("oak_focus_slider")

        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("focus_cmd_topic", "/oak/rgb/focus_set")
        self.declare_parameter("initial_focus", 145)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.focus_cmd_topic = str(self.get_parameter("focus_cmd_topic").value)
        self.focus = int(self.get_parameter("initial_focus").value)
        self.focus = max(0, min(255, self.focus))

        self.bridge = CvBridge()
        self.latest = None
        self.last_pub_focus = None
        self.last_pub_time = 0.0
        self.mode = "manual"

        self.pub = self.create_publisher(String, self.focus_cmd_topic, 10)
        self.sub = self.create_subscription(Image, self.image_topic, self.image_cb, 1)

        cv2.namedWindow("OAK manual focus slider", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("OAK manual focus slider", 1280, 720)
        cv2.createTrackbar("manual_focus 0-255", "OAK manual focus slider", self.focus, 255, self.on_focus)

        self.timer = self.create_timer(0.03, self.loop)

        self.get_logger().info("OAK focus slider started")
        self.get_logger().info("Keys: s=save to bringup_modes.yaml, a=auto focus, m=manual focus, q=quit")
        self.publish_focus(force=True)

    def on_focus(self, val):
        self.focus = int(val)
        self.mode = "manual"
        self.publish_focus(force=True)

    def image_cb(self, msg):
        try:
            self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"image convert failed: {e}")

    def publish_focus(self, force=False):
        now = time.time()
        if not force:
            if self.last_pub_focus == self.focus and now - self.last_pub_time < 0.3:
                return

        msg = String()
        msg.data = json.dumps(
            {
                "mode": self.mode,
                "manual_focus": int(self.focus),
                "autofocus_mode": "continuous_video",
            }
        )
        self.pub.publish(msg)
        self.last_pub_focus = self.focus
        self.last_pub_time = now

    def save_focus(self):
        if not BRINGUP.exists():
            self.get_logger().error(f"bringup file not found: {BRINGUP}")
            return

        text = BRINGUP.read_text()

        if "focus_mode:" not in text:
            text = text.replace(
                "      force_usb2: 'true'\n",
                "      force_usb2: 'true'\n"
                "      focus_mode: manual\n"
                f"      manual_focus: '{self.focus}'\n"
                "      autofocus_mode: continuous_video\n"
            )
        else:
            text = re.sub(r"(^\s*focus_mode:\s*).*$", r"\g<1>manual", text, flags=re.M)
            text = re.sub(r"(^\s*manual_focus:\s*).*$", rf"\g<1>'{self.focus}'", text, flags=re.M)
            if "autofocus_mode:" not in text:
                text = text.replace(
                    re.search(r"^\s*manual_focus:.*$", text, flags=re.M).group(0) + "\n",
                    re.search(r"^\s*manual_focus:.*$", text, flags=re.M).group(0) + "\n"
                    "      autofocus_mode: continuous_video\n",
                )

        BRINGUP.write_text(text)
        self.get_logger().info(f"SAVED manual_focus={self.focus} to {BRINGUP}")

    def loop(self):
        self.publish_focus(force=False)

        if self.latest is None:
            img = None
        else:
            img = self.latest.copy()

        if img is None:
            img = 255 * (cv2.UMat(480, 800, cv2.CV_8UC3).get())
            cv2.putText(
                img,
                f"Waiting image: {self.image_topic}",
                (30, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                img,
                f"mode={self.mode} manual_focus={self.focus} | s=save a=auto m=manual q=quit",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow("OAK manual focus slider", img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            self.save_focus()
        elif key == ord("a"):
            self.mode = "auto"
            self.publish_focus(force=True)
            self.get_logger().info("Switched to AUTO focus")
        elif key == ord("m"):
            self.mode = "manual"
            self.publish_focus(force=True)
            self.get_logger().info(f"Switched to MANUAL focus={self.focus}")
        elif key == ord("q"):
            rclpy.shutdown()


def main():
    rclpy.init()
    node = OakFocusSlider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
