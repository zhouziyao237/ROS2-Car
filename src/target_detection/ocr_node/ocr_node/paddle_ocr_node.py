#!/usr/bin/env python3
import queue
import re
import threading
from collections import OrderedDict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from paddleocr import PaddleOCR
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ocr_msg.msg import OcrRequestArray, OcrResult, OcrResultArray
from yolo_msg.msg import Box2D


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PaddleOcrNode(Node):
    def __init__(self):
        super().__init__('ocr_node')

        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('request_topic', '/vision/ocr_requests')
        self.declare_parameter('result_topic', '/vision/ocr_results')
        self.declare_parameter('debug_image_topic', '/vision/ocr_debug_image')

        self.declare_parameter('lang', 'en')
        self.declare_parameter('use_angle_cls', False)
        self.declare_parameter('use_gpu', False)
        self.declare_parameter('show_log', False)

        self.declare_parameter('image_cache_size', 60)
        self.declare_parameter('max_image_time_diff_ms', 3000.0)
        self.declare_parameter('ocr_queue_size', 4)

        self.declare_parameter('min_roi_width', 8)
        self.declare_parameter('min_roi_height', 8)
        self.declare_parameter('roi_padding_ratio', 0.05)

        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('log_recognized_text', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.request_topic = self.get_parameter('request_topic').value
        self.result_topic = self.get_parameter('result_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value

        self.image_cache_size = int(self.get_parameter('image_cache_size').value)
        self.max_image_time_diff_ns = int(float(self.get_parameter('max_image_time_diff_ms').value) * 1e6)
        self.ocr_queue_size = int(self.get_parameter('ocr_queue_size').value)

        self.min_roi_width = int(self.get_parameter('min_roi_width').value)
        self.min_roi_height = int(self.get_parameter('min_roi_height').value)
        self.roi_padding_ratio = float(self.get_parameter('roi_padding_ratio').value)

        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.log_recognized_text = bool(self.get_parameter('log_recognized_text').value)

        self.bridge = CvBridge()
        self.image_cache = OrderedDict()
        self.image_cache_lock = threading.Lock()

        self.result_pub = self.create_publisher(OcrResultArray, self.result_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.request_sub = self.create_subscription(
            OcrRequestArray,
            self.request_topic,
            self.request_callback,
            10,
        )

        lang = self.get_parameter('lang').value
        use_angle_cls = bool(self.get_parameter('use_angle_cls').value)
        use_gpu = bool(self.get_parameter('use_gpu').value)
        show_log = bool(self.get_parameter('show_log').value)

        self.get_logger().info(
            f'Loading PaddleOCR 2.x: lang={lang}, use_angle_cls={use_angle_cls}, use_gpu={use_gpu}'
        )

        self.ocr = PaddleOCR(
            lang=lang,
            use_angle_cls=use_angle_cls,
            use_gpu=use_gpu,
            show_log=show_log,
        )

        self.request_queue = queue.Queue(maxsize=max(1, self.ocr_queue_size))
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.get_logger().info('PaddleOCR node started')
        self.get_logger().info(f'image_topic={self.image_topic}')
        self.get_logger().info(f'request_topic={self.request_topic}')
        self.get_logger().info(f'result_topic={self.result_topic}')
        self.get_logger().info(f'debug_image_topic={self.debug_image_topic}')

    def destroy_node(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=0.5)
        return super().destroy_node()

    @staticmethod
    def stamp_to_ns(stamp):
        return int(stamp.sec) * 1000000000 + int(stamp.nanosec)

    def image_callback(self, msg):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return

        stamp_ns = self.stamp_to_ns(msg.header.stamp)

        with self.image_cache_lock:
            self.image_cache[stamp_ns] = (msg.header, image_bgr)
            while len(self.image_cache) > self.image_cache_size:
                self.image_cache.popitem(last=False)

    def get_image_by_stamp(self, stamp):
        target_ns = self.stamp_to_ns(stamp)

        with self.image_cache_lock:
            if not self.image_cache:
                return None, None, None

            nearest_ns = min(self.image_cache.keys(), key=lambda k: abs(k - target_ns))
            diff_ns = abs(nearest_ns - target_ns)

            if diff_ns > self.max_image_time_diff_ns:
                return None, None, diff_ns

            header, img = self.image_cache[nearest_ns]
            return header, img.copy(), diff_ns

    def request_callback(self, msg):
        header, image_bgr, diff_ns = self.get_image_by_stamp(msg.header.stamp)

        if image_bgr is None:
            self.get_logger().warn('No synchronized image found for PaddleOCR request')
            return

        if not msg.requests:
            return

        task = {
            'header': msg.header,
            'image_bgr': image_bgr,
            'requests': list(msg.requests),
        }

        if self.request_queue.full():
            try:
                self.request_queue.get_nowait()
                self.request_queue.task_done()
            except queue.Empty:
                pass

        try:
            self.request_queue.put_nowait(task)
        except queue.Full:
            self.get_logger().warn('OCR queue full, drop request')

    def worker_loop(self):
        while not self.stop_event.is_set():
            try:
                task = self.request_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                self.process_task(task)
            except Exception as exc:
                self.get_logger().error(f'PaddleOCR worker failed: {exc}')
            finally:
                self.request_queue.task_done()

    def process_task(self, task):
        header = task['header']
        image_bgr = task['image_bgr']
        requests = task['requests']

        result_array = OcrResultArray()
        result_array.header = header

        debug_image = image_bgr.copy() if self.publish_debug_image else None

        for req in requests:
            if not req.need_ocr:
                continue

            h, w = image_bgr.shape[:2]

            x1 = int(req.box.x1)
            y1 = int(req.box.y1)
            x2 = int(req.box.x2)
            y2 = int(req.box.y2)

            bw = max(1, x2 - x1)
            bh = max(1, y2 - y1)

            pad_x = int(bw * self.roi_padding_ratio)
            pad_y = int(bh * self.roi_padding_ratio)

            x1 = clamp(x1 - pad_x, 0, w - 1)
            y1 = clamp(y1 - pad_y, 0, h - 1)
            x2 = clamp(x2 + pad_x, 0, w)
            y2 = clamp(y2 + pad_y, 0, h)

            if x2 <= x1 or y2 <= y1:
                continue

            if (x2 - x1) < self.min_roi_width or (y2 - y1) < self.min_roi_height:
                continue

            roi = image_bgr[y1:y2, x1:x2]

            results = self.run_ocr_on_roi(
                roi=roi,
                req=req,
                offset_x=x1,
                offset_y=y1,
            )

            result_array.results.extend(results)

            if debug_image is not None:
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), (255, 128, 0), 2)
                cv2.putText(
                    debug_image,
                    req.target_id,
                    (x1, max(15, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 128, 0),
                    1,
                )

                for res in results:
                    cv2.rectangle(
                        debug_image,
                        (res.text_box.x1, res.text_box.y1),
                        (res.text_box.x2, res.text_box.y2),
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        debug_image,
                        f'{res.raw_text} {res.confidence:.2f}',
                        (res.text_box.x1, max(15, res.text_box.y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (0, 255, 0),
                        1,
                    )

        self.result_pub.publish(result_array)

        if debug_image is not None:
            try:
                msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                msg.header = header
                self.debug_pub.publish(msg)
            except Exception as exc:
                self.get_logger().warn(f'publish debug image failed: {exc}')

    def run_ocr_on_roi(self, roi, req, offset_x, offset_y):
        results = []

        try:
            ocr_out = self.ocr.ocr(roi, cls=False)
        except TypeError:
            ocr_out = self.ocr.ocr(roi)

        lines = self.normalize_paddle_output(ocr_out)

        if self.log_recognized_text:
            self.get_logger().info(
                f'target_id={req.target_id}, PaddleOCR lines={len(lines)}'
            )

        for line_id, item in enumerate(lines):
            points, text, score = item

            if not text or not str(text).strip():
                continue

            text = str(text).strip()

            digits = ''.join(re.findall(r'\d+', text))
            raw_text = digits if digits else text

            box = self.make_box(points, offset_x, offset_y)

            res = OcrResult()
            res.header = req.header
            res.target_id = req.target_id
            res.line_id = int(line_id)
            res.raw_text = raw_text
            res.confidence = float(score)
            res.indices = []
            res.has_text = True
            res.roi_box = req.box
            res.text_box = box

            results.append(res)

            if self.log_recognized_text:
                self.get_logger().info(
                    f'[{req.target_id} line {line_id}] text={raw_text}, score={score:.4f}'
                )

        return results

    def normalize_paddle_output(self, ocr_out):
        lines = []

        if ocr_out is None:
            return lines

        if isinstance(ocr_out, list) and len(ocr_out) == 1 and isinstance(ocr_out[0], list):
            candidate = ocr_out[0]
        else:
            candidate = ocr_out

        if not isinstance(candidate, list):
            return lines

        for item in candidate:
            try:
                box = item[0]
                text_score = item[1]
                text = text_score[0]
                score = float(text_score[1])
                lines.append((box, text, score))
            except Exception:
                continue

        return lines

    def make_box(self, points, offset_x, offset_y):
        pts = np.array(points, dtype=np.float32)
        x1 = int(np.min(pts[:, 0]) + offset_x)
        y1 = int(np.min(pts[:, 1]) + offset_y)
        x2 = int(np.max(pts[:, 0]) + offset_x)
        y2 = int(np.max(pts[:, 1]) + offset_y)

        box = Box2D()
        box.x1 = x1
        box.y1 = y1
        box.x2 = x2
        box.y2 = y2
        box.center_u = int((x1 + x2) / 2)
        box.center_v = int((y1 + y2) / 2)
        box.width = int(x2 - x1)
        box.height = int(y2 - y1)
        return box


def main(args=None):
    rclpy.init(args=args)
    node = PaddleOcrNode()
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