#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
from pathlib import Path
from typing import Dict, Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener


def parse_json(text: str) -> Dict[str, Any]:
    try:
        v = json.loads(text)
        if isinstance(v, dict):
            return v
    except Exception:
        pass
    return {}


def norm_yaw(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class FrontQRSemanticWriterNode(Node):
    """
    旧 front_qr_semantic_writer_node 的 AprilTag 兼容版。

    现在货架标签已经从 QR 换成 AprilTag，但输出给旧语义系统时仍然保留：
      type / obs_type / qr_type = layer_qr
      face = front
      shelf_id = Sxx
      layer_id = Lx

    本节点负责把正面标签写进：
      /host-home/maps/semantic_library.json

    修复点：
      1. 同时兼容 type / obs_type / qr_type。
      2. 自动尝试多个 camera frame。
      3. TF camera->map 失败时，退回用 map->base_link + optical近似转换。
      4. 每次失败都会在 /semantic/front_qr_writer_status 里说明原因。
    """

    def __init__(self):
        super().__init__("front_qr_semantic_writer_node")

        self.declare_parameter("input_topic", "/vision/semantic_observation")
        self.declare_parameter("semantic_map_path", "/host-home/maps/semantic_library.json")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("camera_frame_fallback", "camera_color_optical_frame")
        self.declare_parameter("tf_timeout_sec", 0.35)

        self.declare_parameter("min_confidence", 0.2)

        self.declare_parameter("shelf_width", 1.85)
        self.declare_parameter("shelf_depth", 0.35)
        self.declare_parameter("shelf_height", 1.5)
        self.declare_parameter("face_to_center_offset", 0.175)
        self.declare_parameter("approach_distance", 0.60)

        self.declare_parameter("front_qr_yaw_offset_deg", 0.0)
        self.declare_parameter("normal_points_to_camera", True)

        # 若 camera TF 不通，是否用 base_link 近似估计 map 坐标。
        self.declare_parameter("fallback_use_base_pose", True)

        # D435/OAK optical 坐标常见定义：
        # camera optical: x右, y下, z前
        # base_link: x前, y左, z上
        # 所以近似转换：
        # base_x = camera_z
        # base_y = -camera_x
        # base_z = -camera_y
        self.declare_parameter("optical_to_base_x_from", "z")
        self.declare_parameter("optical_to_base_y_from", "-x")

        self.declare_parameter("debug_log", True)

        self.map_path = Path(str(self.get_parameter("semantic_map_path").value))
        self.map_path.parent.mkdir(parents=True, exist_ok=True)
        if not self.map_path.exists():
            self.map_path.write_text(json.dumps({"shelves": {}}, ensure_ascii=False, indent=2))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            String,
            str(self.get_parameter("input_topic").value),
            self.on_observation,
            30,
        )

        self.status_pub = self.create_publisher(
            String,
            "/semantic/front_qr_writer_status",
            10,
        )

        self.get_logger().info(
            "front_qr_semantic_writer_node started in robust AprilTag-compatible mode: "
            f"input={self.get_parameter('input_topic').value}, map={self.map_path}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def debug(self, text: str):
        if bool(self.get_parameter("debug_log").value):
            self.get_logger().info(text)

    def publish_status(self, state: str, message: str, extra: Optional[Dict[str, Any]] = None):
        data = {
            "state": state,
            "message": message,
            "stamp": self.now_sec(),
        }
        if extra:
            data.update(extra)
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.status_pub.publish(msg)

    def should_accept(self, obs: Dict[str, Any]) -> bool:
        typ = str(obs.get("obs_type", obs.get("type", obs.get("qr_type", ""))))
        face = str(obs.get("face", "front"))
        shelf_id = str(obs.get("shelf_id", ""))
        layer_id = str(obs.get("layer_id", ""))

        if typ != "layer_qr":
            return False
        if face and face != "front":
            return False
        if not shelf_id.startswith("S"):
            return False
        if not layer_id.startswith("L"):
            return False

        try:
            conf = float(obs.get("confidence", 1.0))
        except Exception:
            conf = 1.0

        if conf < float(self.get_parameter("min_confidence").value):
            return False

        return True

    def load_library(self) -> Dict[str, Any]:
        try:
            data = json.loads(self.map_path.read_text())
            if isinstance(data, dict):
                data.setdefault("shelves", {})
                return data
        except Exception:
            pass
        return {"shelves": {}}

    def save_library(self, data: Dict[str, Any]):
        self.map_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.map_path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2))
        tmp.replace(self.map_path)

    def candidate_frames(self, obs: Dict[str, Any]):
        frames = []
        for k in ["camera_frame", "frame_id", "header_frame_id"]:
            v = obs.get(k, "")
            if v:
                frames.append(str(v))

        frames.append(str(self.get_parameter("camera_frame_fallback").value))
        frames += [
            "camera_color_optical_frame",
            "camera_camera_color_optical_frame",
            "camera_color_frame",
            "camera_link",
            "camera_depth_optical_frame",
            "oak_camera_link",
            "oak_rgb_camera_optical_frame",
        ]

        out = []
        for f in frames:
            if f and f not in out:
                out.append(f)
        return out

    def transform_point_from_frame(self, frame_id: str, x: float, y: float, z: float) -> Optional[Tuple[float, float, float]]:
        ps = PointStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = Time().to_msg()
        ps.point.x = float(x)
        ps.point.y = float(y)
        ps.point.z = float(z)

        try:
            out = self.tf_buffer.transform(
                ps,
                str(self.get_parameter("map_frame").value),
                timeout=Duration(seconds=float(self.get_parameter("tf_timeout_sec").value)),
            )
            return float(out.point.x), float(out.point.y), float(out.point.z)
        except Exception:
            return None

    def transform_point_auto(self, obs: Dict[str, Any], x: float, y: float, z: float):
        errors = []
        for frame in self.candidate_frames(obs):
            p = self.transform_point_from_frame(frame, x, y, z)
            if p is not None:
                return p, frame, "camera_tf"
            errors.append(frame)

        if bool(self.get_parameter("fallback_use_base_pose").value):
            p = self.transform_point_via_base_fallback(x, y, z)
            if p is not None:
                return p, str(self.get_parameter("robot_frame").value), "base_fallback"

        self.publish_status(
            "FAILED_TF",
            "cannot transform camera point to map",
            {"tried_frames": errors, "camera_xyz": [x, y, z]},
        )
        return None, "", "failed"

    def transform_point_via_base_fallback(self, cx: float, cy: float, cz: float) -> Optional[Tuple[float, float, float]]:
        """
        TF 相机链断开时，用 map->base_link + optical近似转换。
        只为先恢复语义地图显示和任务发布，不代替精确外参。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("robot_frame").value),
                Time(),
                timeout=Duration(seconds=float(self.get_parameter("tf_timeout_sec").value)),
            )
        except Exception as e:
            self.publish_status("FAILED_TF", f"cannot lookup map->base_link: {repr(e)}")
            return None

        bx = float(cz)
        by = -float(cx)

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        tz = float(tf.transform.translation.z)

        q = tf.transform.rotation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        mx = tx + bx * math.cos(yaw) - by * math.sin(yaw)
        my = ty + bx * math.sin(yaw) + by * math.cos(yaw)
        mz = tz

        return mx, my, mz

    def normal_map_auto(self, obs: Dict[str, Any], xyz, normal):
        nx, ny, nz = float(normal[0]), float(normal[1]), float(normal[2])
        nlen = math.sqrt(nx * nx + ny * ny + nz * nz)
        if nlen < 1e-6:
            return None

        nx, ny, nz = nx / nlen, ny / nlen, nz / nlen

        p0, frame0, mode0 = self.transform_point_auto(obs, float(xyz[0]), float(xyz[1]), float(xyz[2]))
        if p0 is None:
            return None

        p1, frame1, mode1 = self.transform_point_auto(
            obs,
            float(xyz[0]) + nx,
            float(xyz[1]) + ny,
            float(xyz[2]) + nz,
        )
        if p1 is None:
            return None

        mx = p1[0] - p0[0]
        my = p1[1] - p0[1]
        mlen = math.sqrt(mx * mx + my * my)

        # 如果 normal 主要朝 z，投影到地图可能很小。用机器人到 tag 的方向兜底。
        if mlen < 1e-4:
            try:
                tf = self.tf_buffer.lookup_transform(
                    str(self.get_parameter("map_frame").value),
                    str(self.get_parameter("robot_frame").value),
                    Time(),
                    timeout=Duration(seconds=float(self.get_parameter("tf_timeout_sec").value)),
                )
                rx = float(tf.transform.translation.x)
                ry = float(tf.transform.translation.y)
                mx = rx - p0[0]
                my = ry - p0[1]
                mlen = math.sqrt(mx * mx + my * my)
            except Exception:
                pass

        if mlen < 1e-4:
            self.publish_status("BAD_NORMAL", "map normal projection too small")
            return None

        mx /= mlen
        my /= mlen

        if not bool(self.get_parameter("normal_points_to_camera").value):
            mx = -mx
            my = -my

        return p0, (mx, my), frame0, mode0

    def compute_pose(self, obs: Dict[str, Any]) -> Optional[Dict[str, float]]:
        xyz = obs.get("camera_xyz", None)
        normal = obs.get("normal", None)

        if not isinstance(xyz, list) or len(xyz) < 3:
            self.publish_status("BAD_OBS", "camera_xyz missing", {"obs_keys": list(obs.keys())})
            return None

        if not isinstance(normal, list) or len(normal) < 3:
            self.publish_status("BAD_OBS", "normal missing", {"obs_keys": list(obs.keys())})
            return None

        ret = self.normal_map_auto(obs, xyz, normal)
        if ret is None:
            return None

        tag_map, normal_map, frame_used, mode_used = ret
        tx, ty, tz = tag_map
        mx, my = normal_map

        shelf_yaw = math.atan2(my, mx)
        shelf_yaw += math.radians(float(self.get_parameter("front_qr_yaw_offset_deg").value))
        shelf_yaw = norm_yaw(shelf_yaw)

        face_to_center = float(self.get_parameter("face_to_center_offset").value)
        approach_distance = float(self.get_parameter("approach_distance").value)

        shelf_x = tx - mx * face_to_center
        shelf_y = ty - my * face_to_center

        approach_x = shelf_x + mx * approach_distance
        approach_y = shelf_y + my * approach_distance
        approach_yaw = norm_yaw(shelf_yaw + math.pi)

        return {
            "tag_x": tx,
            "tag_y": ty,
            "tag_z": tz,
            "normal_x": mx,
            "normal_y": my,
            "shelf_x": shelf_x,
            "shelf_y": shelf_y,
            "shelf_yaw": shelf_yaw,
            "approach_x": approach_x,
            "approach_y": approach_y,
            "approach_yaw": approach_yaw,
            "frame_used": frame_used,
            "mode_used": mode_used,
        }

    def on_observation(self, msg: String):
        obs = parse_json(msg.data)
        if not obs:
            return

        if not self.should_accept(obs):
            return

        pose = self.compute_pose(obs)
        if pose is None:
            return

        shelf_id = str(obs.get("shelf_id", ""))
        layer_id = str(obs.get("layer_id", ""))

        try:
            row = int(obs.get("row", 0) or 0)
        except Exception:
            row = 0

        try:
            stamp = float(obs.get("stamp", self.now_sec()) or self.now_sec())
        except Exception:
            stamp = self.now_sec()

        data = self.load_library()
        shelves = data.setdefault("shelves", {})

        shelf = shelves.setdefault(shelf_id, {})
        shelf["shelf_id"] = shelf_id
        shelf["status"] = "active"
        shelf["last_seen"] = self.now_sec()

        shelf["current_map_pose"] = {
            "valid": True,
            "frame": str(self.get_parameter("map_frame").value),
            "x": pose["shelf_x"],
            "y": pose["shelf_y"],
            "yaw": pose["shelf_yaw"],
            "confidence": float(obs.get("confidence", 1.0)),
            "stamp": stamp,
            "source": "front_qr_semantic_writer_node",
            "mode": pose["mode_used"],
            "frame_used": pose["frame_used"],
        }

        shelf["approach_pose"] = {
            "valid": True,
            "frame": str(self.get_parameter("map_frame").value),
            "x": pose["approach_x"],
            "y": pose["approach_y"],
            "yaw": pose["approach_yaw"],
            "stamp": stamp,
            "source": "front_qr_semantic_writer_node",
        }

        layers = shelf.setdefault("layers", {})
        layers[layer_id] = {
            "seen": True,
            "row": row,
            "stamp": stamp,
            "source": "front_qr_semantic_writer_node",
        }

        shelf["front_anchor"] = {
            "seen": True,
            "x": pose["tag_x"],
            "y": pose["tag_y"],
            "yaw": pose["shelf_yaw"],
            "stamp": stamp,
            "obs_type": "layer_qr",
            "face": "front",
            "frame": str(self.get_parameter("map_frame").value),
            "source": "front_qr_semantic_writer_node",
        }

        shelf["calibration"] = {
            "shelf_width": float(self.get_parameter("shelf_width").value),
            "shelf_depth": float(self.get_parameter("shelf_depth").value),
            "shelf_height": float(self.get_parameter("shelf_height").value),
            "approach_distance": float(self.get_parameter("approach_distance").value),
            "face_to_center_offset": float(self.get_parameter("face_to_center_offset").value),
        }

        self.save_library(data)

        self.publish_status(
            "UPDATED",
            f"updated {shelf_id} {layer_id}",
            {
                "shelf_id": shelf_id,
                "layer_id": layer_id,
                "x": pose["shelf_x"],
                "y": pose["shelf_y"],
                "yaw": pose["shelf_yaw"],
                "approach_x": pose["approach_x"],
                "approach_y": pose["approach_y"],
                "approach_yaw": pose["approach_yaw"],
                "mode": pose["mode_used"],
                "frame_used": pose["frame_used"],
            },
        )

        self.debug(
            f"updated shelf semantic {shelf_id} {layer_id}: "
            f"shelf=({pose['shelf_x']:.2f},{pose['shelf_y']:.2f},{pose['shelf_yaw']:.2f}) "
            f"approach=({pose['approach_x']:.2f},{pose['approach_y']:.2f},{pose['approach_yaw']:.2f}) "
            f"mode={pose['mode_used']} frame={pose['frame_used']}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontQRSemanticWriterNode()

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
