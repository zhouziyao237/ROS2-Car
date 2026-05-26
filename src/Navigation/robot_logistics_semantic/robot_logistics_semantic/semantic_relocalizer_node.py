#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SemanticRelocalizerNode(Node):
    """
    语义重定位节点。

    逻辑：
      1. 每次启动几何 map 都是新的。
      2. 只有语义库长期保存。
      3. 当真实视觉看到旧库中已有 shelf，且它有 current_map_pose 和 semantic_pose，
         就估计 T_map_semantic。
      4. 有 T_map_semantic 后，把其它 shelf 的 semantic_pose 投影成 current_map_pose。
    """

    def __init__(self):
        super().__init__('semantic_relocalizer_node')

        self.declare_parameter('semantic_map_path', '/host-home/maps/semantic_library.json')
        self.declare_parameter('period_sec', 1.0)

        self.semantic_map_path = Path(self.get_parameter('semantic_map_path').value)
        self.period_sec = float(self.get_parameter('period_sec').value)

        self.status_pub = self.create_publisher(String, '/semantic_map/relocalization_status', 10)
        self.timer = self.create_timer(self.period_sec, self.timer_cb)

        self.get_logger().info('semantic_relocalizer_node started')

    def timer_cb(self):
        data = self.load_library()
        shelves = data.get('shelves', {})

        anchors = []

        for sid, shelf in shelves.items():
            sem = shelf.get('semantic_pose')
            cur = shelf.get('current_map_pose')

            if not isinstance(sem, dict) or not isinstance(cur, dict):
                continue

            if not cur.get('valid', False):
                continue

            anchors.append((sid, sem, cur))

        if not anchors:
            self.publish_status(False, [], 0.0)
            return

        # 第一版：用第一个已知锚点估计平移和 yaw。
        # 后续可以升级为多锚点 SVD/ICP。
        sid, sem, cur = anchors[0]

        yaw = float(cur.get('yaw', 0.0)) - float(sem.get('yaw', 0.0))
        sx = float(sem.get('x', 0.0))
        sy = float(sem.get('y', 0.0))
        cx = float(cur.get('x', 0.0))
        cy = float(cur.get('y', 0.0))

        # cur = R * sem + t
        rx = math.cos(yaw) * sx - math.sin(yaw) * sy
        ry = math.sin(yaw) * sx + math.cos(yaw) * sy

        tx = cx - rx
        ty = cy - ry

        data.setdefault('relocalization', {})
        data['relocalization']['has_transform'] = True
        data['relocalization']['T_map_semantic'] = {
            'x': float(tx),
            'y': float(ty),
            'yaw': float(yaw),
        }
        data['relocalization']['matched_anchors'] = [a[0] for a in anchors]
        data['relocalization']['confidence'] = min(1.0, 0.5 + 0.2 * len(anchors))

        # 投影其它货架到当前 map
        for other_sid, shelf in shelves.items():
            sem2 = shelf.get('semantic_pose')
            if not isinstance(sem2, dict):
                continue

            mx, my, myaw = self.transform_pose(
                float(sem2.get('x', 0.0)),
                float(sem2.get('y', 0.0)),
                float(sem2.get('yaw', 0.0)),
                tx, ty, yaw
            )

            # 如果当前已经被真实视觉更新过，就不要覆盖它。
            cur2 = shelf.get('current_map_pose', {})
            if isinstance(cur2, dict) and cur2.get('valid', False):
                continue

            shelf['current_map_pose'] = {
                'valid': True,
                'x': float(mx),
                'y': float(my),
                'yaw': float(myaw),
                'source': 'semantic_projection',
            }

        self.save_library(data)
        self.publish_status(True, [a[0] for a in anchors], data['relocalization']['confidence'])

        self.get_logger().info(
            f'semantic relocalized by anchors={[a[0] for a in anchors]}, '
            f'T_map_semantic=({tx:.2f},{ty:.2f},{yaw:.2f})'
        )

    def transform_pose(self, x, y, yaw0, tx, ty, dyaw):
        mx = math.cos(dyaw) * x - math.sin(dyaw) * y + tx
        my = math.sin(dyaw) * x + math.cos(dyaw) * y + ty
        myaw = self.normalize_angle(yaw0 + dyaw)
        return mx, my, myaw

    def publish_status(self, ok, anchors, confidence):
        msg = String()
        msg.data = json.dumps({
            'has_transform': bool(ok),
            'anchors': anchors,
            'confidence': float(confidence),
        }, ensure_ascii=False)
        self.status_pub.publish(msg)

    def load_library(self):
        if not self.semantic_map_path.exists():
            return {'version': 1, 'semantic_frame': 'semantic_world', 'shelves': {}}
        try:
            return json.loads(self.semantic_map_path.read_text())
        except Exception:
            return {'version': 1, 'semantic_frame': 'semantic_world', 'shelves': {}}

    def save_library(self, data):
        self.semantic_map_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.semantic_map_path.with_suffix('.json.tmp')
        tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False))
        tmp.replace(self.semantic_map_path)

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = SemanticRelocalizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
