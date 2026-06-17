#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import re
from pathlib import Path
from typing import Any, Optional, Tuple, List

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
            return sum(us) / len(us), sum(vs) / len(vs)
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


def _read_key(text: str, key: str) -> Optional[str]:
    m = re.search(rf'^\s*{re.escape(key)}\s*:\s*([^#\n\r]+)', text, flags=re.M)
    if not m:
        return None
    return m.group(1).strip().strip('"\'')


def _as_float(v: Optional[str], default: float) -> float:
    if v is None:
        return default
    try:
        return float(v)
    except Exception:
        return default


def _set_number_key(src: str, key: str, val: float) -> str:
    pat = rf'(^\s*{re.escape(key)}\s*:\s*)([-+0-9.eE]+)(.*$)'
    if re.search(pat, src, flags=re.M):
        return re.sub(pat, rf'\g<1>{val:.1f}\g<3>', src, flags=re.M)
    return src.rstrip() + f'\n    {key}: {val:.1f}\n'


def _set_string_key(src: str, key: str, val: str) -> str:
    pat = rf'(^\s*{re.escape(key)}\s*:\s*)([^#\n\r]+)(.*$)'
    if re.search(pat, src, flags=re.M):
        return re.sub(pat, rf'\g<1>{val}\g<3>', src, flags=re.M)
    return src.rstrip() + f'\n    {key}: {val}\n'


class Calibrator(Node):
    def __init__(self):
        super().__init__('oak_visual_align_point_calibrator')
        if cv2 is None or CvBridge is None:
            raise RuntimeError('cv2/cv_bridge missing; install ros-${ROS_DISTRO}-cv-bridge and python3-opencv')

        self.declare_parameter('rgb_image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('datamatrix_topic', '/oak/datamatrix_3d_result_json')
        self.declare_parameter('visual_align_point_topic', '/oak/visual_align_point')
        self.declare_parameter('config_yaml_path', '/host-home/src/target_capture/oak_guided_pickup/config/oak_guided_pickup.yaml')
        self.declare_parameter('install_config_yaml_path', '/host-home/install/oak_guided_pickup/share/oak_guided_pickup/config/oak_guided_pickup.yaml')
        self.declare_parameter('align_u_px', 960.0)
        self.declare_parameter('align_v_from_bottom_px', 300.0)
        self.declare_parameter('image_height_px', 1080.0)

        self.rgb_topic = str(self.get_parameter('rgb_image_topic').value)
        self.dm_topic = str(self.get_parameter('datamatrix_topic').value)
        self.point_topic = str(self.get_parameter('visual_align_point_topic').value)
        self.yaml_path = Path(str(self.get_parameter('config_yaml_path').value))
        self.install_yaml_path = Path(str(self.get_parameter('install_config_yaml_path').value))
        self.align_u = float(self.get_parameter('align_u_px').value)
        self.image_height = float(self.get_parameter('image_height_px').value)
        self.align_v_bottom = float(self.get_parameter('align_v_from_bottom_px').value)

        self._load_initial_yaml()

        self.bridge = CvBridge()
        self.image = None
        self.code_center = None
        self.pub = self.create_publisher(String, self.point_topic, 10)
        self.create_subscription(Image, self.rgb_topic, self.on_image, 10)
        self.create_subscription(String, self.dm_topic, self.on_dm, 10)

        self.win = 'oak_lift_align_bottom_left_calibrator'
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.win, self.on_mouse)
        cv2.createTrackbar('align_u_px', self.win, int(round(self.align_u)), 1919, self.on_u)
        cv2.createTrackbar('align_v_bottom_px', self.win, int(round(self.align_v_bottom)), int(round(self.image_height)), self.on_v_bottom)
        self.publish_point('startup')
        self.get_logger().info(
            f'bottom-left calibrator started: image={self.rgb_topic}, output={self.point_topic}, '
            f'align_u={self.align_u:.1f}, align_v_bottom={self.align_v_bottom:.1f}, image_h={self.image_height:.1f}'
        )

    def _load_initial_yaml(self):
        if not self.yaml_path.exists():
            return
        try:
            text = self.yaml_path.read_text(encoding='utf-8')
        except Exception:
            return
        self.align_u = _as_float(_read_key(text, 'align_u_px'), self.align_u)
        self.image_height = _as_float(_read_key(text, 'image_height_px'), self.image_height)

        origin = (_read_key(text, 'align_v_origin') or '').lower()
        bottom = _read_key(text, 'align_v_from_bottom_px')
        top = _read_key(text, 'align_v_px')
        if origin == 'bottom_left' and bottom is not None:
            self.align_v_bottom = _as_float(bottom, self.align_v_bottom)
        elif top is not None:
            top_v = _as_float(top, self.image_height - self.align_v_bottom)
            self.align_v_bottom = max(0.0, self.image_height - top_v)

    def current_align_top(self) -> float:
        # OAK/OpenCV display coordinate, top-left origin, v downward.
        return self.image_height - self.align_v_bottom

    def publish_point(self, source: str):
        align_top = self.current_align_top()
        msg = String()
        msg.data = json.dumps({
            'align_u_px': float(self.align_u),
            'align_v_origin': 'bottom_left',
            'align_v_from_bottom_px': float(self.align_v_bottom),
            'align_v_px': float(align_top),
            'image_height_px': float(self.image_height),
            'source': source,
        }, ensure_ascii=False)
        self.pub.publish(msg)

    def on_image(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, _ = self.image.shape[:2]
            if abs(float(h) - float(self.image_height)) > 1.0:
                self.image_height = float(h)
                try:
                    cv2.setTrackbarMax('align_v_bottom_px', self.win, int(round(self.image_height)))
                except Exception:
                    pass
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
            self.align_u = float(x)
            # Convert clicked top-left y to bottom-left distance.
            self.align_v_bottom = max(0.0, float(self.image_height) - float(y))
            cv2.setTrackbarPos('align_u_px', self.win, int(round(self.align_u)))
            cv2.setTrackbarPos('align_v_bottom_px', self.win, int(round(self.align_v_bottom)))
            self.publish_point('mouse_click_bottom_left')

    def on_u(self, v):
        self.align_u = float(v)
        self.publish_point('slider_u')

    def on_v_bottom(self, v):
        self.align_v_bottom = float(v)
        self.publish_point('slider_v_bottom')

    def _config_paths(self) -> List[Path]:
        paths = []
        for p in [self.yaml_path, self.install_yaml_path]:
            if p and p not in paths:
                paths.append(p)
        return paths

    def save_yaml(self):
        align_top = self.current_align_top()
        for path in self._config_paths():
            if not path.exists():
                self.get_logger().warn(f'yaml not found, skip: {path}')
                continue
            text = path.read_text(encoding='utf-8')
            text = _set_number_key(text, 'align_u_px', self.align_u)
            text = _set_string_key(text, 'align_v_origin', 'bottom_left')
            text = _set_number_key(text, 'align_v_from_bottom_px', self.align_v_bottom)
            text = _set_number_key(text, 'image_height_px', self.image_height)
            text = _set_number_key(text, 'align_v_px', align_top)
            path.write_text(text, encoding='utf-8')
            self.get_logger().info(
                f'saved {path}: u={self.align_u:.1f}, v_bottom={self.align_v_bottom:.1f}, '
                f'v_top_equiv={align_top:.1f}, image_h={self.image_height:.1f}'
            )

    def reload_yaml(self):
        self._load_initial_yaml()
        cv2.setTrackbarPos('align_u_px', self.win, int(round(self.align_u)))
        cv2.setTrackbarPos('align_v_bottom_px', self.win, int(round(self.align_v_bottom)))
        self.publish_point('reload_yaml')

    def draw_once(self):
        if self.image is None:
            return True
        img = self.image.copy()
        h, w = img.shape[:2]
        self.image_height = float(h)
        try:
            cv2.setTrackbarMax('align_u_px', self.win, max(1, w - 1))
            cv2.setTrackbarMax('align_v_bottom_px', self.win, max(1, h))
        except Exception:
            pass

        u = int(round(self.align_u))
        v_top_float = self.current_align_top()
        v_top = int(round(max(0.0, min(float(h - 1), v_top_float))))

        cv2.line(img, (u, 0), (u, h - 1), (0, 255, 0), 1)
        cv2.line(img, (0, v_top), (w - 1, v_top), (0, 255, 0), 1)
        cv2.circle(img, (u, v_top), 6, (0, 255, 0), 2)

        text = (
            f'ALIGN bottom-left: u={self.align_u:.1f}, v_bottom={self.align_v_bottom:.1f} '
            f'(top-equiv={v_top_float:.1f}, image_h={h})'
        )
        if self.code_center:
            cu, cv = self.code_center
            code_bottom = float(h) - float(cv)
            err_up = code_bottom - self.align_v_bottom
            cv2.circle(img, (int(round(cu)), int(round(cv))), 7, (0, 0, 255), 2)
            text2 = (
                f'CODE top=({cu:.1f},{cv:.1f}), bottom_v={code_bottom:.1f}, '
                f'err_up={err_up:.1f}  [err_up>0 target above align -> lift up]'
            )
            cv2.putText(img, text2, (20, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        cv2.putText(img, text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(img, 'mouse=set desired align point  c=use code center  s=save  p=publish  r=reload  q/ESC=quit',
                    (20, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.imshow(self.win, img)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            return False
        if key == ord('s'):
            self.save_yaml()
            self.publish_point('save')
        elif key == ord('p'):
            self.publish_point('manual_publish')
        elif key == ord('r'):
            self.reload_yaml()
        elif key == ord('c') and self.code_center:
            cu, cv = self.code_center
            self.align_u = float(cu)
            self.align_v_bottom = max(0.0, float(h) - float(cv))
            cv2.setTrackbarPos('align_u_px', self.win, int(round(self.align_u)))
            cv2.setTrackbarPos('align_v_bottom_px', self.win, int(round(self.align_v_bottom)))
            self.publish_point('use_code_center_bottom_left')
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
