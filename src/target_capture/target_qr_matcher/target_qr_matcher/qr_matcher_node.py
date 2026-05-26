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
from std_msgs.msg import Bool, String

from yolo_msg.msg import YoloDetectionArray, YoloDetection, Box2D


class QrMatcherNode(Node):
    def __init__(self):
        super().__init__('target_matcher_node')

        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('target_code', '1069')
        self.declare_parameter('target_code_topic', '/target_code')

        self.declare_parameter('enable_topic', '/target_matcher/enable')
        self.declare_parameter('reset_lock_topic', '/target_matcher/reset_lock')
        self.declare_parameter('arrived_topic', '/target_matcher/arrived')

        self.declare_parameter('matched_detection_topic', '/target_matcher/matched_detection')
        self.declare_parameter('state_topic', '/target_matcher/state')
        self.declare_parameter('debug_image_topic', '/target_qr_matcher/debug_image')

        # 远距离 1920x1080 推荐：
        # decode_scale=0.75，表示 1920x1080 缩到 1440x810 后再解码
        self.declare_parameter('decode_scale', 0.75)
        self.declare_parameter('publish_rate_hz', 6.0)

        # 不保留旧框，目标消失就清空
        self.declare_parameter('lock_hold_sec', 0.0)

        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('min_qr_size_px', 30)

        # JSON QR 必须 false
        self.declare_parameter('match_digits_only', False)

        # 远距离小 QR，pyzbar 通常比 OpenCV 稳，但会慢，所以频率不要太高
        self.declare_parameter('use_pyzbar_fallback', True)

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        self.enabled = True
        self.target_code = self.norm_code(self.get_parameter('target_code').value)

        self.last_pub_time = 0.0
        self.last_state = ''

        self.det_pub = self.create_publisher(
            YoloDetectionArray,
            self.get_parameter('matched_detection_topic').value,
            10
        )

        self.state_pub = self.create_publisher(
            String,
            self.get_parameter('state_topic').value,
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
            self.target_code_cb,
            10
        )

        self.create_subscription(
            Bool,
            self.get_parameter('enable_topic').value,
            self.enable_cb,
            10
        )

        self.create_subscription(
            Bool,
            self.get_parameter('reset_lock_topic').value,
            self.reset_cb,
            10
        )

        self.create_subscription(
            Bool,
            self.get_parameter('arrived_topic').value,
            self.arrived_cb,
            10
        )

        self.get_logger().info('QR matcher node started WITHOUT tracker')
        self.get_logger().info(f'image_topic={self.get_parameter("image_topic").value}')
        self.get_logger().info(f'target_code={self.target_code}')
        self.get_logger().info(f'decode_scale={self.get_parameter("decode_scale").value}')
        self.get_logger().info(f'publish_rate_hz={self.get_parameter("publish_rate_hz").value}')
        self.get_logger().info(
            f'matched_detection_topic={self.get_parameter("matched_detection_topic").value}'
        )

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

        if bool(self.get_parameter('match_digits_only').value):
            parts = re.findall(r'\d+', text)
            if parts:
                return ''.join(parts)

        return text.strip()

    def publish_state(self, state):
        if state == self.last_state:
            return

        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.last_state = state

    def target_code_cb(self, msg):
        self.target_code = self.norm_code(msg.data)
        self.publish_empty_detection()
        self.publish_state('TARGET_CHANGED')
        self.get_logger().info(f'target_code changed to {self.target_code}')

    def enable_cb(self, msg):
        self.enabled = bool(msg.data)

        if not self.enabled:
            self.publish_empty_detection()
            self.publish_state('DISABLED')
        else:
            self.publish_state('ENABLED')

        self.get_logger().info(f'enabled={self.enabled}')

    def reset_cb(self, msg):
        if bool(msg.data):
            self.publish_empty_detection()
            self.publish_state('RESET_LOCK')
            self.get_logger().info('QR matcher reset')

    def arrived_cb(self, msg):
        if bool(msg.data):
            self.publish_state('ARRIVED')

    def box_from_points(self, points):
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)

        x1 = int(np.min(pts[:, 0]))
        y1 = int(np.min(pts[:, 1]))
        x2 = int(np.max(pts[:, 0]))
        y2 = int(np.max(pts[:, 1]))

        return [x1, y1, x2, y2]

    def decode_opencv(self, img):
        results = []

        try:
            ok, decoded_info, points, _ = self.detector.detectAndDecodeMulti(img)

            if ok and points is not None:
                for raw_text, pts in zip(decoded_info, points):
                    if raw_text:
                        results.append(
                            (raw_text, self.box_from_points(pts), 'ocv_multi')
                        )
        except Exception as e:
            self.get_logger().debug(f'OpenCV multi QR failed: {e}')

        try:
            raw_text, points, _ = self.detector.detectAndDecode(img)

            if raw_text and points is not None:
                results.append(
                    (raw_text, self.box_from_points(points), 'ocv_single')
                )
        except Exception as e:
            self.get_logger().debug(f'OpenCV single QR failed: {e}')

        return results

    def decode_pyzbar(self, img):
        if not bool(self.get_parameter('use_pyzbar_fallback').value):
            return []

        try:
            from pyzbar.pyzbar import decode
        except Exception as e:
            self.get_logger().warn(f'pyzbar not available: {e}')
            return []

        results = []

        try:
            decoded = decode(img)

            for obj in decoded:
                raw_text = obj.data.decode('utf-8', errors='ignore')
                x, y, w, h = obj.rect
                box = [int(x), int(y), int(x + w), int(y + h)]
                results.append((raw_text, box, 'pyzbar'))
        except Exception as e:
            self.get_logger().debug(f'pyzbar QR failed: {e}')

        return results

    def filter_results(self, results, scale):
        filtered = []
        min_size = int(self.get_parameter('min_qr_size_px').value)

        for raw_text, box, method in results:
            if abs(scale - 1.0) > 1e-3:
                box = [int(v / scale) for v in box]

            x1, y1, x2, y2 = box
            w = x2 - x1
            h = y2 - y1

            if w < min_size or h < min_size:
                continue

            filtered.append((raw_text, box, method))

        return filtered

    def run_qr_decode(self, img):
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

        results = []
        results.extend(self.decode_opencv(decode_img))
        results.extend(self.decode_pyzbar(decode_img))

        return self.filter_results(results, scale)

    def make_detection(self, header, box, text, confidence=1.0):
        x1, y1, x2, y2 = [int(v) for v in box]

        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = max(x1 + 1, x2)
        y2 = max(y1 + 1, y2)

        b = Box2D()
        b.x1 = x1
        b.y1 = y1
        b.x2 = x2
        b.y2 = y2
        b.center_u = int((x1 + x2) / 2)
        b.center_v = int((y1 + y2) / 2)
        b.width = int(x2 - x1)
        b.height = int(y2 - y1)

        det = YoloDetection()
        det.header = header
        det.target_id = text
        det.class_name = 'target'
        det.class_id = 0
        det.confidence = float(confidence)
        det.box = b
        det.need_ocr = False

        return det

    def publish_detection(self, header, box, text, confidence=1.0):
        arr = YoloDetectionArray()
        arr.header = header
        arr.detections.append(self.make_detection(header, box, text, confidence))
        self.det_pub.publish(arr)

    def publish_empty_detection(self, header=None):
        arr = YoloDetectionArray()

        if header is not None:
            arr.header = header

        self.det_pub.publish(arr)

    def draw_debug(self, img, all_results, matched_box=None, matched_text=''):
        debug = img.copy()

        for raw_text, box, method in all_results:
            x1, y1, x2, y2 = box
            norm = self.norm_code(raw_text)

            color = (0, 255, 255)
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                debug,
                f'{norm}:{method}',
                (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1
            )

        if matched_box is not None:
            x1, y1, x2, y2 = matched_box
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(
                debug,
                f'MATCH {matched_text}',
                (x1, max(25, y1 - 28)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

        cv2.putText(
            debug,
            f'target={self.target_code}',
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2
        )

        return debug

    def image_cb(self, msg):
        if not self.enabled:
            return

        now = time.time()
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        min_dt = 1.0 / max(1.0, publish_rate)

        if now - self.last_pub_time < min_dt:
            return

        self.last_pub_time = now

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
            return

        results = self.run_qr_decode(img)

        matched = False
        matched_box = None
        matched_text = ''

        for raw_text, box, method in results:
            norm = self.norm_code(raw_text)
            is_match = (norm == self.target_code)

            self.get_logger().info(
                f'QR decoded raw="{raw_text}", norm="{norm}", '
                f'target="{self.target_code}", method={method}, matched={is_match}'
            )

            if is_match:
                matched = True
                matched_box = box
                matched_text = norm
                break

        if matched and matched_box is not None:
            self.publish_detection(msg.header, matched_box, matched_text, 1.0)
            self.publish_state('QR_TRACKING')
        else:
            self.publish_empty_detection(msg.header)
            self.publish_state('QR_SEARCHING')

        if bool(self.get_parameter('publish_debug_image').value):
            debug = self.draw_debug(
                img,
                results,
                matched_box if matched else None,
                matched_text if matched else ''
            )

            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            )


def main(args=None):
    rclpy.init(args=args)
    node = QrMatcherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publish_empty_detection()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
