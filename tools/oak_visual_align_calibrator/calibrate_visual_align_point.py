#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import re
from pathlib import Path
from typing import Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

try:
    import cv2
    from cv_bridge import CvBridge
except Exception:
    cv2 = None
    CvBridge = None


def parse_center_px(item: Any) -> Optional[Tuple[float, float]]:
    if not isinstance(item, dict):
        return None
    for key in ('center_px', 'code_center_px', 'pixel_center', 'center'):
        p = item.get(key)
        if isinstance(p, dict):
            u = p.get('u', p.get('x', p.get('cx')))
            v = p.get('v', p.get('y', p.get('cy')))
            if u is not None and v is not None:
                try:
                    return float(u), float(v)
                except Exception:
                    pass
        if isinstance(p, (list, tuple)) and len(p) >= 2:
            try:
                return float(p[0]), float(p[1])
            except Exception:
                pass
    bbox = item.get('bbox') or item.get('box') or item.get('rect')
    if isinstance(bbox, dict):
        try:
            if all(k in bbox for k in ('x', 'y', 'w', 'h')):
                return float(bbox['x']) + float(bbox['w']) * 0.5, float(bbox['y']) + float(bbox['h']) * 0.5
            if all(k in bbox for k in ('xmin', 'ymin', 'xmax', 'ymax')):
                return (float(bbox['xmin']) + float(bbox['xmax'])) * 0.5, (float(bbox['ymin']) + float(bbox['ymax'])) * 0.5
        except Exception:
            pass
    pts = item.get('corners') or item.get('points') or item.get('polygon')
    if isinstance(pts, list) and pts:
        us, vs = [], []
        for p in pts:
            try:
                if isinstance(p, dict):
                    u = p.get('u', p.get('x'))
                    v = p.get('v', p.get('y'))
                    if u is not None and v is not None:
                        us.append(float(u)); vs.append(float(v))
                elif isinstance(p, (list, tuple)) and len(p) >= 2:
                    us.append(float(p[0])); vs.append(float(p[1]))
            except Exception:
                pass
        if us and vs:
            return sum(us)/len(us), sum(vs)/len(vs)
    return None


def iter_items(payload: Any):
    if isinstance(payload, dict):
        for key in ('items', 'results', 'detections', 'codes'):
            if isinstance(payload.get(key), list):
                for item in payload[key]:
                    if isinstance(item, dict):
                        yield item
        if any(k in payload for k in ('code', 'text', 'data')):
            yield payload
    elif isinstance(payload, list):
        for item in payload:
            if isinstance(item, dict):
                yield item


class Calibrator(Node):
    def __init__(self):
        super().__init__('oak_visual_align_point_calibrator')
        if cv2 is None or CvBridge is None:
            raise RuntimeError('cv2/cv_bridge missing; install ros-${ROS_DISTRO}-cv-bridge and python3-opencv')
        self.declare_parameter('rgb_image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('datamatrix_topic', '/oak/datamatrix_3d_result_json')
        self.declare_parameter('visual_align_point_topic', '/oak/visual_align_point')
        self.declare_parameter('config_yaml_path', '/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml')
        self.declare_parameter('align_u_px', 38.0)
        self.declare_parameter('align_v_px', 100.0)
        self.rgb_topic = str(self.get_parameter('rgb_image_topic').value)
        self.dm_topic = str(self.get_parameter('datamatrix_topic').value)
        self.point_topic = str(self.get_parameter('visual_align_point_topic').value)
        self.yaml_path = Path(str(self.get_parameter('config_yaml_path').value))
        self.align_u = float(self.get_parameter('align_u_px').value)
        self.align_v = float(self.get_parameter('align_v_px').value)
        self.bridge = CvBridge()
        self.image = None
        self.code_center = None
        self.pub = self.create_publisher(String, self.point_topic, 10)
        self.create_subscription(Image, self.rgb_topic, self.on_image, 10)
        self.create_subscription(String, self.dm_topic, self.on_dm, 10)
        self.win = 'oak_visual_align_point'
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.win, self.on_mouse)
        cv2.createTrackbar('align_u_px', self.win, int(self.align_u), 1919, self.on_u)
        cv2.createTrackbar('align_v_px', self.win, int(self.align_v), 1079, self.on_v)
        self.publish_point('startup')
        self.get_logger().info(f'calibrator started: image={self.rgb_topic}, output={self.point_topic}')

    def publish_point(self, source):
        msg = String()
        msg.data = json.dumps({'align_u_px': float(self.align_u), 'align_v_px': float(self.align_v), 'source': source}, ensure_ascii=False)
        self.pub.publish(msg)

    def on_image(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')

    def on_dm(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        for item in iter_items(payload):
            c = parse_center_px(item)
            if c:
                self.code_center = c
                return

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.align_u, self.align_v = float(x), float(y)
            cv2.setTrackbarPos('align_u_px', self.win, int(self.align_u))
            cv2.setTrackbarPos('align_v_px', self.win, int(self.align_v))
            self.publish_point('mouse_click')

    def on_u(self, v):
        self.align_u = float(v); self.publish_point('slider')

    def on_v(self, v):
        self.align_v = float(v); self.publish_point('slider')

    def save_yaml(self):
        if not self.yaml_path.exists():
            self.get_logger().error(f'yaml not found: {self.yaml_path}')
            return
        text = self.yaml_path.read_text(encoding='utf-8')
        def set_key(src, key, val):
            pat = rf'(^\s*{key}\s*:\s*)([-+0-9.]+)(.*$)'
            if re.search(pat, src, flags=re.M):
                return re.sub(pat, rf'\g<1>{val:.1f}\g<3>', src, flags=re.M)
            return src + f'    {key}: {val:.1f}\n'
        text = set_key(text, 'align_u_px', self.align_u)
        text = set_key(text, 'align_v_px', self.align_v)
        self.yaml_path.write_text(text, encoding='utf-8')
        self.get_logger().info(f'saved align point: u={self.align_u:.1f}, v={self.align_v:.1f}')

    def reload_yaml(self):
        if not self.yaml_path.exists():
            return
        text = self.yaml_path.read_text(encoding='utf-8')
        mu = re.search(r'^\s*align_u_px\s*:\s*([-+0-9.]+)', text, flags=re.M)
        mv = re.search(r'^\s*align_v_px\s*:\s*([-+0-9.]+)', text, flags=re.M)
        if mu: self.align_u = float(mu.group(1))
        if mv: self.align_v = float(mv.group(1))
        cv2.setTrackbarPos('align_u_px', self.win, int(self.align_u))
        cv2.setTrackbarPos('align_v_px', self.win, int(self.align_v))
        self.publish_point('reload_yaml')

    def draw_once(self):
        if self.image is None:
            return True
        img = self.image.copy()
        h, w = img.shape[:2]
        try:
            cv2.setTrackbarMax('align_u_px', self.win, max(1, w-1))
            cv2.setTrackbarMax('align_v_px', self.win, max(1, h-1))
        except Exception:
            pass
        u, v = int(round(self.align_u)), int(round(self.align_v))
        cv2.line(img, (u, 0), (u, h-1), (0,255,0), 1)
        cv2.line(img, (0, v), (w-1, v), (0,255,0), 1)
        cv2.circle(img, (u, v), 6, (0,255,0), 2)
        text = f'align=({self.align_u:.1f},{self.align_v:.1f})'
        if self.code_center:
            cu, cv = self.code_center
            cv2.circle(img, (int(cu), int(cv)), 7, (0,0,255), 2)
            text += f' code=({cu:.1f},{cv:.1f}) err=({cu-self.align_u:.1f},{cv-self.align_v:.1f})'
        cv2.putText(img, text, (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv2.putText(img, 'mouse=set  c=use code  s=save  p=publish  r=reload  q/ESC=quit', (20,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
        cv2.imshow(self.win, img)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27): return False
        if key == ord('s'):
            self.save_yaml(); self.publish_point('save')
        elif key == ord('p'):
            self.publish_point('manual_publish')
        elif key == ord('r'):
            self.reload_yaml()
        elif key == ord('c') and self.code_center:
            self.align_u, self.align_v = self.code_center
            cv2.setTrackbarPos('align_u_px', self.win, int(self.align_u))
            cv2.setTrackbarPos('align_v_px', self.win, int(self.align_v))
            self.publish_point('use_code_center')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = Calibrator()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if not node.draw_once():
                break
    finally:
        node.destroy_node()
        if cv2 is not None:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
