#!/usr/bin/env python3
import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import zxingcpp
    HAS_ZXING = True
except Exception:
    zxingcpp = None
    HAS_ZXING = False


class OakMultiDataMatrixDetector(Node):
    """
    Multi Data Matrix detector.

    Input:
      /oak/rgb/image_raw

    Output:
      /oak/datamatrix_result          单个/第一个码，方便旧状态机兼容
      /oak/datamatrix_result_json     多个码 JSON
      /oak/datamatrix_debug_image     带多个画框的调试图
    """

    def __init__(self):
        super().__init__("oak_multi_datamatrix_detector")

        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("result_topic", "/oak/datamatrix_result")
        self.declare_parameter("result_json_topic", "/oak/datamatrix_result_json")
        self.declare_parameter("debug_image_topic", "/oak/datamatrix_debug_image")

        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("show_window", False)

        self.declare_parameter("roi_ratio", 1.0)
        self.declare_parameter("resize_scale", 2.0)
        self.declare_parameter("decode_interval", 0.15)

        # 是否只识别 Data Matrix。true 时 QR / Code128 会被忽略。
        self.declare_parameter("datamatrix_only", True)

        # 同一帧内去重：不同预处理方法可能识别到同一个码。
        self.declare_parameter("deduplicate", True)

        # 重复发布间隔，避免同一组码疯狂刷屏。
        self.declare_parameter("repeat_publish_interval", 0.8)

        self.declare_parameter("enable_sharpen", True)
        self.declare_parameter("enable_binary", False)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.result_json_topic = str(self.get_parameter("result_json_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)

        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.show_window = bool(self.get_parameter("show_window").value)

        self.roi_ratio = float(self.get_parameter("roi_ratio").value)
        self.resize_scale = float(self.get_parameter("resize_scale").value)
        self.decode_interval = float(self.get_parameter("decode_interval").value)

        self.datamatrix_only = bool(self.get_parameter("datamatrix_only").value)
        self.deduplicate = bool(self.get_parameter("deduplicate").value)
        self.repeat_publish_interval = float(self.get_parameter("repeat_publish_interval").value)

        self.enable_sharpen = bool(self.get_parameter("enable_sharpen").value)
        self.enable_binary = bool(self.get_parameter("enable_binary").value)

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.result_json_pub = self.create_publisher(String, self.result_json_topic, 10)

        self.debug_image_pub = (
            self.create_publisher(Image, self.debug_image_topic, 10)
            if self.publish_debug_image
            else None
        )

        self.last_decode_time = 0.0
        self.last_publish_time = 0.0
        self.last_signature = ""

        self.get_logger().info("OakMultiDataMatrixDetector started")
        self.get_logger().info(f"image_topic: {self.image_topic}")
        self.get_logger().info(f"debug_image_topic: {self.debug_image_topic}")

        if not HAS_ZXING:
            self.get_logger().error("zxing-cpp not installed. Run: pip install zxing-cpp")

    # =========================
    # ROI
    # =========================
    def crop_center_roi(self, frame):
        h, w = frame.shape[:2]

        ratio = max(0.1, min(1.0, self.roi_ratio))
        roi_w = int(w * ratio)
        roi_h = int(h * ratio)

        x1 = max((w - roi_w) // 2, 0)
        y1 = max((h - roi_h) // 2, 0)
        x2 = min(x1 + roi_w, w)
        y2 = min(y1 + roi_h, h)

        return frame[y1:y2, x1:x2], (x1, y1, x2, y2)

    # =========================
    # Preprocess
    # =========================
    def make_candidates(self, roi):
        candidates = []

        candidates.append(("raw", roi, 1.0))

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        candidates.append(("gray", gray, 1.0))

        if self.resize_scale > 1.0:
            gray_big = cv2.resize(
                gray,
                None,
                fx=self.resize_scale,
                fy=self.resize_scale,
                interpolation=cv2.INTER_CUBIC,
            )
            candidates.append(("gray_big", gray_big, self.resize_scale))
        else:
            gray_big = gray

        if self.enable_sharpen:
            blur = cv2.GaussianBlur(gray_big, (0, 0), 1.0)
            sharp = cv2.addWeighted(gray_big, 1.6, blur, -0.6, 0)
            candidates.append(("sharp_big", sharp, self.resize_scale))
        else:
            sharp = gray_big

        if self.enable_binary:
            try:
                binary = cv2.adaptiveThreshold(
                    sharp,
                    255,
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY,
                    31,
                    5,
                )
                candidates.append(("binary", binary, self.resize_scale))
            except Exception:
                pass

        return candidates

    # =========================
    # ZXing helpers
    # =========================
    def is_datamatrix(self, fmt):
        f = str(fmt).lower()
        return "datamatrix" in f or "data_matrix" in f or "data matrix" in f

    def extract_points_from_zxing(self, result, roi_offset=(0, 0), scale=1.0):
        ox, oy = roi_offset

        pos = getattr(result, "position", None)
        if pos is None:
            return None

        raw_points = []

        names = ["top_left", "top_right", "bottom_right", "bottom_left"]
        if all(hasattr(pos, name) for name in names):
            for name in names:
                raw_points.append(getattr(pos, name))
        elif hasattr(pos, "__iter__"):
            try:
                raw_points = list(pos)
            except Exception:
                raw_points = []

        points = []
        for p in raw_points:
            try:
                x = getattr(p, "x", None)
                y = getattr(p, "y", None)

                if x is None or y is None:
                    if isinstance(p, (tuple, list)) and len(p) >= 2:
                        x, y = p[0], p[1]

                if x is None or y is None:
                    continue

                x = int(float(x) / scale + ox)
                y = int(float(y) / scale + oy)
                points.append((x, y))
            except Exception:
                continue

        if len(points) >= 4:
            return points[:4]

        return None

    def result_center_key(self, item):
        """
        用文本 + 中心点近似去重。
        如果没有角点，就只按文本去重。
        """
        text = item.get("text", "")

        pts = item.get("points", None)
        if pts is None or len(pts) < 4:
            return (text, "no_points")

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]

        cx = int(sum(xs) / len(xs) / 20) * 20
        cy = int(sum(ys) / len(ys) / 20) * 20

        return (text, cx, cy)

    def decode_all_in_roi(self, roi, roi_box):
        """
        返回同一帧内识别到的所有 Data Matrix。
        """
        x1, y1, _, _ = roi_box
        all_items = []

        if not HAS_ZXING:
            return []

        for method, img, scale in self.make_candidates(roi):
            try:
                results = zxingcpp.read_barcodes(img)
            except Exception as e:
                self.get_logger().warn(f"ZXing decode error: {e}")
                continue

            for r in results:
                text = getattr(r, "text", "")
                fmt = str(getattr(r, "format", "UNKNOWN"))

                if not text:
                    continue

                if self.datamatrix_only and not self.is_datamatrix(fmt):
                    continue

                points = self.extract_points_from_zxing(
                    r,
                    roi_offset=(x1, y1),
                    scale=scale,
                )

                all_items.append(
                    {
                        "text": text,
                        "format": fmt,
                        "method": method,
                        "points": points,
                    }
                )

        if not self.deduplicate:
            return all_items

        unique = {}
        for item in all_items:
            key = self.result_center_key(item)
            if key not in unique:
                unique[key] = item

        return list(unique.values())

    # =========================
    # Draw
    # =========================
    def draw_results(self, frame, results, roi_box):
        out = frame.copy()

        x1, y1, x2, y2 = roi_box
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 255, 0), 2)

        if not results:
            cv2.putText(
                out,
                "DataMatrix: NOT DECODED",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            return out

        cv2.putText(
            out,
            f"DataMatrix count: {len(results)}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        for idx, item in enumerate(results):
            text = item.get("text", "")
            fmt = item.get("format", "")
            method = item.get("method", "")
            points = item.get("points", None)

            color = (0, 255, 0)

            if points is not None and len(points) >= 4:
                pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(out, [pts], True, color, 3)

                tx, ty = points[0]
                text_pos = (tx, max(30, ty - 12))
            else:
                # 如果 ZXing 没返回角点，先在左上角列出。
                text_pos = (20, 80 + idx * 35)

            cv2.putText(
                out,
                f"{idx + 1}: {text}",
                text_pos,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.85,
                color,
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                out,
                f"{fmt} | {method}",
                (20, out.shape[0] - 30 - idx * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2,
                cv2.LINE_AA,
            )

        return out

    # =========================
    # Publish
    # =========================
    def publish_results(self, results):
        if not results:
            return

        now = time.time()

        codes = [r["text"] for r in results]
        signature = "|".join(sorted(codes))

        if signature == self.last_signature and now - self.last_publish_time < self.repeat_publish_interval:
            return

        # 兼容旧逻辑：第一个码发到 /oak/datamatrix_result
        first_msg = String()
        first_msg.data = codes[0]
        self.result_pub.publish(first_msg)

        json_msg = String()
        json_msg.data = json.dumps(
            {
                "count": len(results),
                "codes": codes,
                "items": [
                    {
                        "code": r.get("text", ""),
                        "format": r.get("format", ""),
                        "method": r.get("method", ""),
                        "points": r.get("points", None),
                    }
                    for r in results
                ],
                "source_topic": self.image_topic,
                "timestamp": now,
            },
            ensure_ascii=False,
        )
        self.result_json_pub.publish(json_msg)

        self.last_signature = signature
        self.last_publish_time = now

        self.get_logger().info(f"DataMatrix count={len(results)} codes={codes}")

    # =========================
    # Callback
    # =========================
    def image_callback(self, msg):
        now = time.time()

        if now - self.last_decode_time < self.decode_interval:
            return

        self.last_decode_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge convert failed: {e}")
            return

        roi, roi_box = self.crop_center_roi(frame)
        results = self.decode_all_in_roi(roi, roi_box)

        debug = self.draw_results(frame, results, roi_box)

        if results:
            self.publish_results(results)

        if self.debug_image_pub is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                debug_msg.header.stamp = msg.header.stamp
                debug_msg.header.frame_id = msg.header.frame_id
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().warn(f"debug image publish failed: {e}")

        if self.show_window:
            cv2.imshow("Multi DataMatrix Detection", debug)
            key = cv2.waitKey(1)
            if key == ord("q"):
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = OakMultiDataMatrixDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()