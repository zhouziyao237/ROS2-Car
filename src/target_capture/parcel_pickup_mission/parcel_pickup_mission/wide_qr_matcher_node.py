#!/usr/bin/env python3
import json
import re
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32


class WideQrMatcherNode(Node):
    def __init__(self):
        super().__init__('wide_qr_matcher_node')

        self.declare_parameter('image_topic', '/wide/image_raw')
        self.declare_parameter('target_code_topic', '/wide_target_code')
        self.declare_parameter('initial_target_code', 'COL:1')

        self.declare_parameter('state_topic', '/wide_qr/state')
        self.declare_parameter('raw_topic', '/wide_qr/raw')
        self.declare_parameter('center_error_x_topic', '/wide_qr/center_error_x')
        self.declare_parameter('debug_image_topic', '/wide_qr/debug_image')

        self.declare_parameter('decode_scale', 0.75)
        self.declare_parameter('publish_rate_hz', 8.0)
        self.declare_parameter('min_qr_size_px', 25)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('use_pyzbar_fallback', True)

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        self.target_code = self.norm_code(self.get_parameter('initial_target_code').value)
        self.last_decode_time = 0.0
        self.last_state = ''

        self.state_pub = self.create_publisher(
            String,
            self.get_parameter('state_topic').value,
            10
        )

        self.raw_pub = self.create_publisher(
            String,
            self.get_parameter('raw_topic').value,
            10
        )

        self.center_err_pub = self.create_publisher(
            Float32,
            self.get_parameter('center_error_x_topic').value,
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            self.get_parameter('debug_image_topic').value,
            10
        )

        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_cb,
            10
        )

        self.create_subscription(
            String,
            self.get_parameter('target_code_topic').value,
            self.target_cb,
            10
        )

        self.get_logger().info('wide_qr_matcher_node started')
        self.get_logger().info(f'image_topic={self.get_parameter("image_topic").value}')
        self.get_logger().info(f'target_code={self.target_code}')

    def norm_code(self, value):
        text = str(value).strip()

        if text == '':
            return ''

        try:
            data = json.loads(text)
            if isinstance(data, dict):
                if 'col' in data:
                    return f"COL:{int(data['col'])}"
                if 'row' in data:
                    return f"ROW:{int(data['row'])}"
                if 'pkg' in data:
                    return str(data['pkg']).strip()
                if 'package' in data:
                    return str(data['package']).strip()
                if 'id' in data:
                    return str(data['id']).strip()
        except Exception:
            pass

        upper = text.upper()

        if upper.startswith('COL:'):
            return upper

        if upper.startswith('ROW:'):
            return upper

        if upper.startswith('PKG:'):
            return upper.split(':', 1)[1].strip()

        parts = re.findall(r'\d+', text)
        if parts and text.strip().isdigit():
            return ''.join(parts)

        return text

    def target_cb(self, msg):
        self.target_code = self.norm_code(msg.data)
        self.publish_state('TARGET_CHANGED')
        self.get_logger().info(f'wide target_code changed to {self.target_code}')

    def publish_state(self, state):
        if state == self.last_state:
            return

        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.last_state = state

    def publish_raw(self, raw):
        msg = String()
        msg.data = str(raw)
        self.raw_pub.publish(msg)

    def publish_center_error(self, err):
        msg = Float32()
        msg.data = float(err)
        self.center_err_pub.publish(msg)

    def box_from_points(self, points):
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)

        return [
            int(np.min(pts[:, 0])),
            int(np.min(pts[:, 1])),
            int(np.max(pts[:, 0])),
            int(np.max(pts[:, 1])),
        ]

    def decode_opencv(self, img):
        results = []

        try:
            ok, decoded_info, points, _ = self.detector.detectAndDecodeMulti(img)

            if ok and points is not None:
                for raw, pts in zip(decoded_info, points):
                    if raw:
                        results.append((raw, self.box_from_points(pts), 'ocv_multi'))
        except Exception:
            pass

        try:
            raw, points, _ = self.detector.detectAndDecode(img)

            if raw and points is not None:
                results.append((raw, self.box_from_points(points), 'ocv_single'))
        except Exception:
            pass

        return results

    def decode_pyzbar(self, img):
        if not bool(self.get_parameter('use_pyzbar_fallback').value):
            return []

        try:
            from pyzbar.pyzbar import decode
        except Exception:
            return []

        results = []

        try:
            for obj in decode(img):
                raw = obj.data.decode('utf-8', errors='ignore')
                x, y, w, h = obj.rect
                results.append(
                    (raw, [int(x), int(y), int(x + w), int(y + h)], 'pyzbar')
                )
        except Exception:
            pass

        return results

    def run_decode(self, img):
        scale = float(self.get_parameter('decode_scale').value)
        decode_img = img

        if abs(scale - 1.0) > 1e-3:
            decode_img = cv2.resize(
                img,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_AREA
            )

        results = self.decode_opencv(decode_img)
        results.extend(self.decode_pyzbar(decode_img))

        out = []
        min_size = int(self.get_parameter('min_qr_size_px').value)

        for raw, box, method in results:
            if abs(scale - 1.0) > 1e-3:
                box = [int(v / scale) for v in box]

            x1, y1, x2, y2 = box

            if x2 - x1 < min_size or y2 - y1 < min_size:
                continue

            out.append((raw, box, method))

        return out

    def image_cb(self, msg):
        now = time.time()
        rate = float(self.get_parameter('publish_rate_hz').value)

        if now - self.last_decode_time < 1.0 / max(1.0, rate):
            return

        self.last_decode_time = now

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'image convert failed: {e}')
            return

        h, w = img.shape[:2]
        results = self.run_decode(img)

        matched = False
        matched_raw = ''
        matched_box = None

        for raw, box, method in results:
            norm = self.norm_code(raw)
            ok = norm == self.target_code

            self.get_logger().info(
                f'WIDE QR raw="{raw}", norm="{norm}", '
                f'target="{self.target_code}", method={method}, matched={ok}'
            )

            if ok:
                matched = True
                matched_raw = raw
                matched_box = box
                break

        if matched and matched_box is not None:
            x1, y1, x2, y2 = matched_box
            cx = 0.5 * (x1 + x2)
            err_norm = (cx - w * 0.5) / max(1.0, w * 0.5)

            self.publish_state('QR_TRACKING')
            self.publish_raw(matched_raw)
            self.publish_center_error(err_norm)
        else:
            self.publish_state('QR_SEARCHING')

        if bool(self.get_parameter('publish_debug_image').value):
            debug = img.copy()

            cv2.line(debug, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)
            cv2.putText(
                debug,
                f'WIDE TARGET: {self.target_code}',
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            for raw, box, method in results:
                x1, y1, x2, y2 = box
                norm = self.norm_code(raw)
                color = (0, 255, 0) if norm == self.target_code else (0, 165, 255)

                cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    debug,
                    f'{norm}',
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2
                )

            out = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            out.header = msg.header
            self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WideQrMatcherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
