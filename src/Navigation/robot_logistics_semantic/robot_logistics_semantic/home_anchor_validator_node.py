#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from pathlib import Path
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_json(text: str) -> Dict[str, Any]:
    try:
        value = json.loads(text)
        if isinstance(value, dict):
            return value
    except Exception:
        pass
    return {}


class HomeAnchorValidatorNode(Node):
    """
    HOME AprilTag 语义锚点节点。

    输入：
      /vision/semantic_observation

    输出：
      /semantic/home_anchor_status
      /host-home/maps/home_anchor.json

    作用：
      1. 如果看到 HOME tag_id=0，则保存 HOME 观测。
      2. 如果没有 HOME，不干预原来的 map TF。
      3. 如果 HOME 已存在，再看到货架标签，则发布校验状态。
      4. 第一版不直接改 map->odom TF，避免 Nav2 / SLAM 地图跳变。
    """

    def __init__(self):
        super().__init__("home_anchor_validator_node")

        self.declare_parameter("observation_topic", "/vision/semantic_observation")
        self.declare_parameter("home_anchor_path", "/host-home/maps/home_anchor.json")
        self.declare_parameter("status_topic", "/semantic/home_anchor_status")
        self.declare_parameter("debug_log", True)

        self.home_anchor_path = Path(str(self.get_parameter("home_anchor_path").value))
        self.home_anchor_path.parent.mkdir(parents=True, exist_ok=True)

        self.home_obs: Optional[Dict[str, Any]] = None
        self.load_home_anchor()

        self.sub = self.create_subscription(
            String,
            str(self.get_parameter("observation_topic").value),
            self.on_observation,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )

        self.get_logger().info(
            f"home_anchor_validator_node started: "
            f"observation={self.get_parameter('observation_topic').value}, "
            f"home_anchor_path={self.home_anchor_path}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def debug(self, text: str):
        if bool(self.get_parameter("debug_log").value):
            self.get_logger().info(text)

    def load_home_anchor(self):
        if self.home_anchor_path.exists():
            try:
                self.home_obs = json.loads(self.home_anchor_path.read_text())
            except Exception:
                self.home_obs = None

    def publish_status(self, state: str, message: str, extra: Optional[Dict[str, Any]] = None):
        data = {
            "state": state,
            "message": message,
            "has_home": self.home_obs is not None,
            "stamp": self.now_sec(),
        }

        if extra:
            data.update(extra)

        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.status_pub.publish(msg)

    def save_home_anchor(self, obs: Dict[str, Any]):
        data = {
            "type": "home_anchor",
            "raw": obs.get("raw", "HOME"),
            "tag_id": obs.get("tag_id", 0),
            "camera_frame": obs.get("camera_frame", obs.get("frame_id", "")),
            "camera_xyz": obs.get("camera_xyz", []),
            "normal": obs.get("normal", []),
            "corners": obs.get("corners", []),
            "source": obs.get("source", ""),
            "stamp": obs.get("stamp", self.now_sec()),
            "saved_at": self.now_sec(),
            "note": "HOME is a semantic origin/check anchor. It does not directly overwrite map->odom TF."
        }

        self.home_obs = data
        self.home_anchor_path.write_text(json.dumps(data, ensure_ascii=False, indent=2))
        self.publish_status("HOME_UPDATED", "home anchor saved", data)
        self.debug("HOME anchor saved/updated")

    def validate_shelf(self, obs: Dict[str, Any]):
        shelf_id = obs.get("shelf_id", "")
        tag_id = obs.get("tag_id", None)

        if self.home_obs is None:
            self.publish_status(
                "NO_HOME_FALLBACK_MAP_TF",
                "HOME tag not available; keep using current map TF",
                {
                    "shelf_id": shelf_id,
                    "tag_id": tag_id,
                    "raw": obs.get("raw", ""),
                },
            )
            return

        self.publish_status(
            "HOME_AVAILABLE_SHELF_OBSERVED",
            "HOME exists; shelf observation can be checked against semantic map",
            {
                "shelf_id": shelf_id,
                "layer_id": obs.get("layer_id", ""),
                "tag_id": tag_id,
                "raw": obs.get("raw", ""),
                "shelf_camera_xyz": obs.get("camera_xyz", []),
                "home_camera_xyz_at_last_seen": self.home_obs.get("camera_xyz", []),
            },
        )

    def on_observation(self, msg: String):
        obs = parse_json(msg.data)
        if not obs:
            return

        obs_type = str(obs.get("obs_type", obs.get("type", "")))
        tag_type = str(obs.get("tag_type", ""))
        tag_id = obs.get("tag_id", None)

        if obs_type == "home_tag" or tag_type == "home_tag" or tag_id == 0:
            self.save_home_anchor(obs)
            return

        if obs_type in ["layer_qr", "column_qr"] or tag_type in ["row_tag", "column_tag"]:
            self.validate_shelf(obs)


def main(args=None):
    rclpy.init(args=args)
    node = HomeAnchorValidatorNode()

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
