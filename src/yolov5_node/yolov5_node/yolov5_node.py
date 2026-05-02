import os
import time
import cv2
import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo_msg.msg import Box2D, YoloDetection, YoloDetectionArray


class YOLOv5ONNXDetector:
    def __init__(
        self,
        model_path,
        img_size=640,
        conf_thres=0.25,
        iou_thres=0.45,
        class_names=None,
        anchors=None,
        strides=None,
        num_threads=1,
    ):
        self.model_path = model_path
        self.img_size = int(img_size)
        self.conf_thres = float(conf_thres)
        self.iou_thres = float(iou_thres)

        self.class_names = class_names if class_names is not None else ["target"]

        self.anchors = anchors if anchors is not None else [
            [(10, 13), (16, 30), (33, 23)],
            [(30, 61), (62, 45), (59, 119)],
            [(116, 90), (156, 198), (373, 326)],
        ]

        self.strides = strides if strides is not None else [8, 16, 32]

        so = ort.SessionOptions()
        so.intra_op_num_threads = int(num_threads)
        so.inter_op_num_threads = int(num_threads)

        self.session = ort.InferenceSession(
            self.model_path,
            sess_options=so,
            providers=["CPUExecutionProvider"],
        )

        self.input_name = self.session.get_inputs()[0].name
        self.output_info = self.session.get_outputs()

        print("ONNX model loaded:", self.model_path)
        print("Input name:", self.input_name)
        for out in self.output_info:
            print("Output:", out.name, out.shape, out.type)

    @staticmethod
    def sigmoid(x):
        return 1.0 / (1.0 + np.exp(-x))

    def letterbox(self, img, color=(114, 114, 114)):
        h, w = img.shape[:2]

        r = min(self.img_size / h, self.img_size / w)

        new_w = int(round(w * r))
        new_h = int(round(h * r))

        dw = self.img_size - new_w
        dh = self.img_size - new_h

        dw /= 2
        dh /= 2

        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))

        padded = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=color,
        )

        return padded, r, (dw, dh)

    def preprocess(self, img_bgr):
        img_lb, ratio, dwdh = self.letterbox(img_bgr)

        img_rgb = cv2.cvtColor(img_lb, cv2.COLOR_BGR2RGB)
        img_rgb = img_rgb.astype(np.float32) / 255.0

        img_chw = np.transpose(img_rgb, (2, 0, 1))
        img_nchw = np.expand_dims(img_chw, axis=0)

        return img_nchw.astype(np.float32), ratio, dwdh

    def decode_yolov5_head(self, pred, anchors, stride):
        """
        pred: [1, 3, grid_h, grid_w, 5 + nc]
        return: [N, 6] = x1, y1, x2, y2, score, class_id
        """
        pred = pred[0]

        num_anchors, grid_h, grid_w, _ = pred.shape

        pred = self.sigmoid(pred)

        grid_y, grid_x = np.meshgrid(
            np.arange(grid_h),
            np.arange(grid_w),
            indexing="ij",
        )

        grid = np.stack((grid_x, grid_y), axis=-1)
        grid = np.expand_dims(grid, axis=0)

        anchors = np.array(anchors, dtype=np.float32).reshape(num_anchors, 1, 1, 2)

        xy = (pred[..., 0:2] * 2.0 - 0.5 + grid) * stride
        wh = ((pred[..., 2:4] * 2.0) ** 2) * anchors

        obj_conf = pred[..., 4:5]
        cls_conf = pred[..., 5:]

        class_ids = np.argmax(cls_conf, axis=-1)
        class_scores = np.max(cls_conf, axis=-1, keepdims=True)

        scores = obj_conf * class_scores

        x = xy[..., 0]
        y = xy[..., 1]
        w = wh[..., 0]
        h = wh[..., 1]

        x1 = x - w / 2
        y1 = y - h / 2
        x2 = x + w / 2
        y2 = y + h / 2

        scores = scores.squeeze(-1)

        mask = scores > self.conf_thres

        boxes = np.stack([x1, y1, x2, y2], axis=-1)

        boxes = boxes[mask]
        scores = scores[mask]
        class_ids = class_ids[mask]

        if boxes.shape[0] == 0:
            return np.zeros((0, 6), dtype=np.float32)

        dets = np.concatenate(
            [
                boxes.astype(np.float32),
                scores.reshape(-1, 1).astype(np.float32),
                class_ids.reshape(-1, 1).astype(np.float32),
            ],
            axis=1,
        )

        return dets

    def nms(self, dets):
        if dets.shape[0] == 0:
            return dets

        boxes = dets[:, :4]
        scores = dets[:, 4]

        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]

        areas = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
        order = scores.argsort()[::-1]

        keep = []

        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            inter_w = np.maximum(0, xx2 - xx1)
            inter_h = np.maximum(0, yy2 - yy1)
            inter = inter_w * inter_h

            union = areas[i] + areas[order[1:]] - inter
            iou = inter / np.maximum(union, 1e-6)

            inds = np.where(iou <= self.iou_thres)[0]
            order = order[inds + 1]

        return dets[keep]

    @staticmethod
    def scale_boxes_to_original(dets, ratio, dwdh, original_shape):
        if dets.shape[0] == 0:
            return dets

        dw, dh = dwdh
        h0, w0 = original_shape[:2]

        dets[:, [0, 2]] -= dw
        dets[:, [1, 3]] -= dh

        dets[:, :4] /= ratio

        dets[:, [0, 2]] = np.clip(dets[:, [0, 2]], 0, w0)
        dets[:, [1, 3]] = np.clip(dets[:, [1, 3]], 0, h0)

        return dets

    def infer(self, img_bgr):
        input_tensor, ratio, dwdh = self.preprocess(img_bgr)

        t0 = time.time()
        outputs = self.session.run(None, {self.input_name: input_tensor})
        t1 = time.time()

        all_dets = []

        # YOLOv5 v5 三检测头输出:
        # [1, 3, 80, 80, 6]
        # [1, 3, 40, 40, 6]
        # [1, 3, 20, 20, 6]
        if len(outputs) == 3 and len(outputs[0].shape) == 5:
            for i, pred in enumerate(outputs):
                dets = self.decode_yolov5_head(
                    pred=pred,
                    anchors=self.anchors[i],
                    stride=self.strides[i],
                )
                all_dets.append(dets)

            if len(all_dets) > 0:
                all_dets = np.concatenate(all_dets, axis=0)
            else:
                all_dets = np.zeros((0, 6), dtype=np.float32)

        # 兼容部分已经 decode 成 [1, 25200, 6] 的 ONNX
        elif len(outputs) >= 1 and len(outputs[0].shape) == 3:
            pred = outputs[0][0]

            # 如果你的 ONNX 已经 sigmoid 过，这里可能需要关掉。
            # 但你当前 YOLOv5 v5 三输出模型不会走这里。
            pred = self.sigmoid(pred)

            obj = pred[:, 4:5]
            cls = pred[:, 5:]
            class_ids = np.argmax(cls, axis=1)
            class_scores = np.max(cls, axis=1, keepdims=True)
            scores = (obj * class_scores).squeeze(1)

            boxes_xywh = pred[:, :4]
            x = boxes_xywh[:, 0]
            y = boxes_xywh[:, 1]
            w = boxes_xywh[:, 2]
            h = boxes_xywh[:, 3]

            x1 = x - w / 2
            y1 = y - h / 2
            x2 = x + w / 2
            y2 = y + h / 2

            boxes = np.stack([x1, y1, x2, y2], axis=1)

            mask = scores > self.conf_thres

            if np.sum(mask) == 0:
                all_dets = np.zeros((0, 6), dtype=np.float32)
            else:
                all_dets = np.concatenate(
                    [
                        boxes[mask].astype(np.float32),
                        scores[mask].reshape(-1, 1).astype(np.float32),
                        class_ids[mask].reshape(-1, 1).astype(np.float32),
                    ],
                    axis=1,
                )

        else:
            raise RuntimeError(f"Unsupported ONNX output format: {[o.shape for o in outputs]}")

        final_dets = self.nms(all_dets)

        final_dets = self.scale_boxes_to_original(
            final_dets,
            ratio=ratio,
            dwdh=dwdh,
            original_shape=img_bgr.shape,
        )

        infer_ms = (t1 - t0) * 1000.0

        return final_dets, infer_ms


class YOLOv5ONNXNode(Node):
    def __init__(self):
        super().__init__("yolov5_onnx_node")
        pkg_share = get_package_share_directory("yolov5_node")
        weights_file = os.path.join(pkg_share, "weights", "best_int8.onnx")

        # ===================== 通用参数 =====================
        self.declare_parameter("model_path", weights_file)

        # input_mode:
        # topic = 订阅图像话题实时检测
        # image = 读取本地图片检测一次
        # auto  = 如果 image_path 存在则 image，否则 topic
        self.declare_parameter("input_mode", "topic")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("detections_topic", "/yolov5/detections")
        self.declare_parameter("annotated_topic", "/yolov5/annotated_image")

        # ===================== 单图模式参数 =====================
        self.declare_parameter("image_path", "")
        self.declare_parameter("save_path", "/mnt/yolov5/result.jpg")
        self.declare_parameter("exit_after_image", True)

        # ===================== 模型参数 =====================
        self.declare_parameter("img_size", 640)
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("num_threads", 1)

        self.declare_parameter("class_names", ["target"])

        self.declare_parameter(
            "anchors",
            [
                10.0, 13.0, 16.0, 30.0, 33.0, 23.0,
                30.0, 61.0, 62.0, 45.0, 59.0, 119.0,
                116.0, 90.0, 156.0, 198.0, 373.0, 326.0,
            ],
        )

        self.model_path = self.get_parameter("model_path").value
        self.input_mode = self.get_parameter("input_mode").value
        self.image_topic = self.get_parameter("image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.annotated_topic = self.get_parameter("annotated_topic").value

        self.image_path = self.get_parameter("image_path").value
        self.save_path = self.get_parameter("save_path").value
        self.exit_after_image = bool(self.get_parameter("exit_after_image").value)

        img_size = self.get_parameter("img_size").value
        conf_thres = self.get_parameter("conf_thres").value
        iou_thres = self.get_parameter("iou_thres").value
        num_threads = self.get_parameter("num_threads").value
        class_names = self.get_parameter("class_names").value
        anchors = self.parse_anchors(self.get_parameter("anchors").value)

        # ===================== auto 模式判断 =====================
        self.real_mode = self.input_mode

        if self.input_mode == "auto":
            if self.image_path != "" and os.path.exists(self.image_path):
                self.real_mode = "image"
            else:
                self.real_mode = "topic"

        if self.real_mode not in ["topic", "image"]:
            raise RuntimeError("input_mode must be topic, image, or auto.")

        self.bridge = CvBridge()

        self.detector = YOLOv5ONNXDetector(
            model_path=self.model_path,
            img_size=img_size,
            conf_thres=conf_thres,
            iou_thres=iou_thres,
            class_names=class_names,
            anchors=anchors,
            num_threads=num_threads,
        )

        self.det_pub = self.create_publisher(
            YoloDetectionArray,
            self.detections_topic,
            10,
        )

        self.image_pub = self.create_publisher(
            Image,
            self.annotated_topic,
            10,
        )

        self.get_logger().info("YOLOv5 ONNX integrated node started.")
        self.get_logger().info(f"Requested input_mode: {self.input_mode}")
        self.get_logger().info(f"Selected input_mode: {self.real_mode}")
        self.get_logger().info(f"Model path: {self.model_path}")

        if self.real_mode == "topic":
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                qos_profile_sensor_data,
            )

            self.get_logger().info(f"Subscribing image topic: {self.image_topic}")
            self.get_logger().info(f"Publishing detections: {self.detections_topic}")
            self.get_logger().info(f"Publishing annotated image: {self.annotated_topic}")

        elif self.real_mode == "image":
            self.get_logger().info(f"Single image mode.")
            self.get_logger().info(f"Image path: {self.image_path}")
            self.get_logger().info(f"Save path: {self.save_path}")

            self.timer = self.create_timer(0.2, self.single_image_callback)
            self.has_run_single_image = False

    @staticmethod
    def parse_anchors(anchors_flat):
        if len(anchors_flat) != 18:
            raise RuntimeError("anchors parameter must contain 18 numbers.")

        anchors = []
        idx = 0

        for _ in range(3):
            head_anchors = []
            for _ in range(3):
                w = float(anchors_flat[idx])
                h = float(anchors_flat[idx + 1])
                head_anchors.append((w, h))
                idx += 2
            anchors.append(head_anchors)

        return anchors

    def image_callback(self, msg):
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}")
            return

        try:
            dets, infer_ms = self.detector.infer(img_bgr)
        except Exception as e:
            self.get_logger().error(f"ONNX inference failed: {e}")
            return

        self.publish_results(
            img_bgr=img_bgr,
            dets=dets,
            infer_ms=infer_ms,
            frame_id=msg.header.frame_id if msg.header.frame_id else "camera_link",
            stamp=msg.header.stamp,
            save_result=False,
        )

        self.get_logger().info(
            f"Detected {len(dets)} objects, inference {infer_ms:.2f} ms",
            throttle_duration_sec=1.0,
        )

    def single_image_callback(self):
        if self.has_run_single_image:
            return

        self.has_run_single_image = True

        if self.image_path == "":
            self.get_logger().error("image_path is empty.")
            rclpy.shutdown()
            return

        if not os.path.exists(self.image_path):
            self.get_logger().error(f"image_path does not exist: {self.image_path}")
            rclpy.shutdown()
            return

        img_bgr = cv2.imread(self.image_path)
        if img_bgr is None:
            self.get_logger().error(f"Failed to read image: {self.image_path}")
            rclpy.shutdown()
            return

        try:
            dets, infer_ms = self.detector.infer(img_bgr)
        except Exception as e:
            self.get_logger().error(f"ONNX inference failed: {e}")
            rclpy.shutdown()
            return

        now = self.get_clock().now().to_msg()

        self.publish_results(
            img_bgr=img_bgr,
            dets=dets,
            infer_ms=infer_ms,
            frame_id="single_image",
            stamp=now,
            save_result=True,
        )

        self.get_logger().info(f"Single image detected {len(dets)} objects.")
        self.get_logger().info(f"Inference time: {infer_ms:.2f} ms")
        self.get_logger().info(f"Saved result to: {self.save_path}")

        for det in dets:
            x1, y1, x2, y2, score, cls_id = det
            self.get_logger().info(
                f"class={int(cls_id)}, score={score:.4f}, "
                f"box=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f})"
            )

        if self.exit_after_image:
            rclpy.shutdown()

    def publish_results(self, img_bgr, dets, infer_ms, frame_id, stamp, save_result=False):
        det_msg = self.build_detection_msg(dets, frame_id, stamp)
        self.det_pub.publish(det_msg)

        annotated = self.draw_detections(img_bgr.copy(), dets, infer_ms)

        if save_result:
            cv2.imwrite(self.save_path, annotated)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header.frame_id = frame_id
        annotated_msg.header.stamp = stamp
        self.image_pub.publish(annotated_msg)

    def build_detection_msg(self, dets, frame_id, stamp):
        det_array = YoloDetectionArray()
        det_array.header.frame_id = frame_id
        det_array.header.stamp = stamp

        for index, det in enumerate(dets):
            x1, y1, x2, y2, score, cls_id = det

            x1 = int(round(float(x1)))
            y1 = int(round(float(y1)))
            x2 = int(round(float(x2)))
            y2 = int(round(float(y2)))
            score = float(score)
            cls_id = int(cls_id)

            if 0 <= cls_id < len(self.detector.class_names):
                class_name = self.detector.class_names[cls_id]
            else:
                class_name = str(cls_id)

            detection = YoloDetection()
            detection.header = det_array.header
            detection.target_id = f"det_{index}"
            detection.class_name = class_name
            detection.class_id = cls_id
            detection.confidence = score
            detection.need_ocr = False

            bbox = Box2D()
            bbox.x1 = x1
            bbox.y1 = y1
            bbox.x2 = x2
            bbox.y2 = y2
            bbox.center_u = int(round((x1 + x2) / 2.0))
            bbox.center_v = int(round((y1 + y2) / 2.0))
            bbox.width = max(0, x2 - x1)
            bbox.height = max(0, y2 - y1)

            detection.box = bbox
            det_array.detections.append(detection)

        return det_array

    def draw_detections(self, img, dets, infer_ms):
        for det in dets:
            x1, y1, x2, y2, score, cls_id = det

            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)
            cls_id = int(cls_id)

            if cls_id < len(self.detector.class_names):
                name = self.detector.class_names[cls_id]
            else:
                name = str(cls_id)

            label = f"{name} {score:.2f}"

            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.putText(
                img,
                label,
                (x1, max(y1 - 5, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        cv2.putText(
            img,
            f"ONNX inference: {infer_ms:.2f} ms",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        return img


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv5ONNXNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
