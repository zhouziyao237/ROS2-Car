#!/usr/bin/env python3
import os
from typing import Dict, List, Optional, Tuple

import cv2
import depthai as dai
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


try:
    from yolo_msg.msg import YoloDetection, YoloDetectionArray, Box2D
    HAS_YOLO_MSG = True
except Exception:
    YoloDetection = None
    YoloDetectionArray = None
    Box2D = None
    HAS_YOLO_MSG = False


class OakVisionDriverNode(Node):
    """
    OAK vision driver with YAML camera configuration.

    Recommended mode:
      OAK ColorCamera 1080p sensor
      -> preview 640x640
      -> dai.node.NeuralNetwork runs best_fp16.blob on OAK
      -> host decodes YOLOv5 tensors
      -> publishes RGB / annotated image / yolo table / yolo_msg/YoloDetectionArray

    Important:
      - NN input always uses cam_rgb.preview.
      - If camera.rgb_output=preview, published RGB is the same 640x640 as NN input.
      - If camera.rgb_output=video, published RGB is video size and bbox is scaled from NN size.
    """

    def __init__(self):
        super().__init__("oak_vision_driver_node")

        # =========================
        # ROS parameters
        # =========================
        self.declare_parameter("config_path", "")
        self.declare_parameter("override_blob_path", "")
        self.declare_parameter("override_parser", "")
        self.declare_parameter("override_conf", -1.0)
        self.declare_parameter("override_iou", -1.0)

        self.config_path = str(self.get_parameter("config_path").value)
        self.override_blob_path = str(self.get_parameter("override_blob_path").value)
        self.override_parser = str(self.get_parameter("override_parser").value)
        self.override_conf = float(self.get_parameter("override_conf").value)
        self.override_iou = float(self.get_parameter("override_iou").value)

        cfg = self.load_yolo_yaml_config(self.config_path)

        model_cfg = cfg.get("model", {})
        camera_cfg = cfg.get("camera", {})
        input_cfg = cfg.get("input", {})
        classes_cfg = cfg.get("classes", {})
        threshold_cfg = cfg.get("threshold", {})
        yolo_cfg = cfg.get("yolo", {})
        ros_cfg = cfg.get("ros", {})
        runtime_cfg = cfg.get("runtime", {})

        # =========================
        # Runtime config
        # =========================
        self.enable_rgb = bool(runtime_cfg.get("enable_rgb", True))
        self.enable_depth = bool(runtime_cfg.get("enable_depth", False))
        self.enable_yolo = bool(runtime_cfg.get("enable_yolo", True))

        self.draw_oak_detections_on_rgb = bool(runtime_cfg.get("draw_oak_detections_on_rgb", True))
        self.publish_annotated_image = bool(runtime_cfg.get("publish_annotated_image", True))
        self.publish_debug_text_enabled = bool(runtime_cfg.get("publish_debug_text", True))
        self.publish_yolo_table_enabled = bool(runtime_cfg.get("publish_yolo_table", True))

        self.force_usb2 = bool(runtime_cfg.get("force_usb2", True))
        self.stop_on_xlink_error = bool(runtime_cfg.get("stop_on_xlink_error", True))
        self.queue_size = int(runtime_cfg.get("queue_size", 1))
        self.timer_period = float(runtime_cfg.get("timer_period", 0.02))
        self.ros_image_encoding = str(runtime_cfg.get("ros_image_encoding", "bgr8"))

        self.debug_scores = bool(runtime_cfg.get("debug_scores", False))
        self.score_log_interval = int(runtime_cfg.get("score_log_interval", 60))
        self._score_log_count = 0

        # =========================
        # Camera config from YAML
        # =========================
        self.sensor_resolution = str(camera_cfg.get("sensor_resolution", "1080p")).lower()
        self.rgb_output = str(camera_cfg.get("rgb_output", "preview")).lower()
        self.color_order = str(camera_cfg.get("color_order", "RGB")).upper()
        self.keep_aspect_ratio = bool(camera_cfg.get("keep_aspect_ratio", False))

        self.preview_width = int(camera_cfg.get("preview_width", input_cfg.get("width", 640)))
        self.preview_height = int(camera_cfg.get("preview_height", input_cfg.get("height", 640)))

        self.video_width = int(camera_cfg.get("video_width", 1920))
        self.video_height = int(camera_cfg.get("video_height", 1080))

        # Network input size. Must match blob export size.
        self.runtime_width = int(input_cfg.get("width", self.preview_width))
        self.runtime_height = int(input_cfg.get("height", self.preview_height))
        self.fps = float(input_cfg.get("fps", 6.0))

        if self.preview_width != self.runtime_width or self.preview_height != self.runtime_height:
            self.get_logger().warn(
                "preview_width/height and input.width/height are different. "
                "NN receives preview, so these should normally match."
            )

        # Published RGB size.
        if self.rgb_output == "preview":
            self.rgb_width = self.preview_width
            self.rgb_height = self.preview_height
        elif self.rgb_output == "video":
            self.rgb_width = self.video_width
            self.rgb_height = self.video_height
        else:
            self.get_logger().warn(f"Unsupported rgb_output={self.rgb_output}, fallback to preview")
            self.rgb_output = "preview"
            self.rgb_width = self.preview_width
            self.rgb_height = self.preview_height

        # =========================
        # ROS topic config
        # =========================
        self.rgb_topic = str(ros_cfg.get("rgb_topic", "/oak/rgb/image_raw"))
        self.depth_topic = str(ros_cfg.get("depth_topic", "/oak/depth/image_raw"))
        self.camera_info_topic = str(ros_cfg.get("camera_info_topic", "/oak/rgb/camera_info"))
        self.detections_topic = str(ros_cfg.get("detections_topic", "/yolov5/detections"))
        self.yolo_debug_topic = str(ros_cfg.get("yolo_debug_topic", "/oak/yolo_debug_text"))
        self.annotated_image_topic = str(ros_cfg.get("annotated_image_topic", "/oak/yolo_annotated_image"))
        self.yolo_table_topic = str(ros_cfg.get("yolo_table_topic", "/oak/yolo_table"))
        self.frame_id = str(ros_cfg.get("frame_id", "oak_rgb_camera_optical_frame"))

        # =========================
        # Model config
        # =========================
        self.blob_path = str(model_cfg.get("blob_path", "/host-home/src/oak_vision_driver/weights/best_fp16.blob"))
        self.yolo_backend = "oak"
        self.parser = str(model_cfg.get("parser", "NeuralNetwork"))
        self.network_type = str(model_cfg.get("network_type", "yolov5"))

        # =========================
        # Classes
        # =========================
        self.num_classes = int(classes_cfg.get("num_classes", 1))
        self.class_names = list(classes_cfg.get("names", ["target"]))

        # =========================
        # Threshold
        # =========================
        self.conf = float(threshold_cfg.get("confidence", 0.08))
        self.iou = float(threshold_cfg.get("iou", 0.35))

        # =========================
        # YOLO decode config
        # =========================
        self.coordinate_size = int(yolo_cfg.get("coordinate_size", 4))
        self.anchors = [
            float(x)
            for x in yolo_cfg.get(
                "anchors",
                [
                    10, 13,
                    16, 30,
                    33, 23,
                    30, 61,
                    62, 45,
                    59, 119,
                    116, 90,
                    156, 198,
                    373, 326,
                ],
            )
        ]

        anchor_masks = yolo_cfg.get("anchor_masks", {})
        self.anchor_masks_side80 = [int(x) for x in anchor_masks.get("side80", [0, 1, 2])]
        self.anchor_masks_side40 = [int(x) for x in anchor_masks.get("side40", [3, 4, 5])]
        self.anchor_masks_side20 = [int(x) for x in anchor_masks.get("side20", [6, 7, 8])]

        self.score_mode = str(yolo_cfg.get("score_mode", "obj_cls"))
        self.output_value_mode = str(yolo_cfg.get("output_value_mode", "auto"))

        # =========================
        # Launch overrides
        # =========================
        if self.override_blob_path:
            self.blob_path = self.override_blob_path

        if self.override_parser:
            self.parser = self.override_parser

        if self.override_conf >= 0.0:
            self.conf = float(self.override_conf)

        if self.override_iou >= 0.0:
            self.iou = float(self.override_iou)

        # =========================
        # ROS publishers
        # =========================
        self.bridge = CvBridge()

        self.rgb_pub = self.create_publisher(Image, self.rgb_topic, 10)
        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)

        self.annotated_pub = self.create_publisher(Image, self.annotated_image_topic, 10)
        self.debug_pub = self.create_publisher(String, self.yolo_debug_topic, 10)
        self.table_pub = self.create_publisher(String, self.yolo_table_topic, 10)

        if HAS_YOLO_MSG:
            self.detections_pub = self.create_publisher(YoloDetectionArray, self.detections_topic, 10)
        else:
            self.detections_pub = None
            self.get_logger().warn(
                "yolo_msg import failed. /yolov5/detections will not be published. "
                "Expected: yolo_msg/msg/YoloDetectionArray, YoloDetection, Box2D."
            )

        # =========================
        # Runtime state
        # =========================
        self.pipeline = None
        self.device = None
        self.rgb_queue = None
        self.depth_queue = None
        self.nn_queue = None
        self.oak_alive = False

        self.latest_detections: List[Dict] = []
        self.latest_frame_detections: List[Dict] = []
        self.last_rgb_frame: Optional[np.ndarray] = None
        self.last_depth_frame: Optional[np.ndarray] = None
        self.camera_intrinsics = None
        self.printed_nn_layers = False

        self.log_config()
        self.connect_oak()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # =========================
    # Config
    # =========================
    def load_yolo_yaml_config(self, config_path: str):
        if not config_path:
            self.get_logger().warn("No config_path provided. Using default values.")
            return {}

        if not os.path.exists(config_path):
            raise RuntimeError(f"YOLO config file does not exist: {config_path}")

        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        if cfg is None:
            cfg = {}

        self.get_logger().info(f"Loaded OAK YOLO config: {config_path}")
        return cfg

    def log_config(self):
        self.get_logger().info("oak_vision_driver_node started")
        self.get_logger().info(f"enable_rgb={self.enable_rgb}, enable_depth={self.enable_depth}, enable_yolo={self.enable_yolo}")
        self.get_logger().info(f"yolo_backend={self.yolo_backend}, parser={self.parser}")
        self.get_logger().info(f"blob_path={self.blob_path}")
        self.get_logger().info(f"network_type={self.network_type}")
        self.get_logger().info(f"sensor_resolution={self.sensor_resolution}, rgb_output={self.rgb_output}, color_order={self.color_order}")
        self.get_logger().info(f"preview={self.preview_width}x{self.preview_height}, video={self.video_width}x{self.video_height}")
        self.get_logger().info(f"network_input={self.runtime_width}x{self.runtime_height}@{self.fps}")
        self.get_logger().info(f"rgb_publish_size={self.rgb_width}x{self.rgb_height}")
        self.get_logger().info(f"num_classes={self.num_classes}, class_names={self.class_names}")
        self.get_logger().info(f"conf={self.conf}, iou={self.iou}")
        self.get_logger().info(f"score_mode={self.score_mode}, output_value_mode={self.output_value_mode}")
        self.get_logger().info(f"anchors={self.anchors}")
        self.get_logger().info(
            f"anchor_masks={{side80:{self.anchor_masks_side80}, side40:{self.anchor_masks_side40}, side20:{self.anchor_masks_side20}}}"
        )
        self.get_logger().info(f"rgb_topic={self.rgb_topic}")
        self.get_logger().info(f"depth_topic={self.depth_topic}")
        self.get_logger().info(f"camera_info_topic={self.camera_info_topic}")
        self.get_logger().info(f"detections_topic={self.detections_topic}")
        self.get_logger().info(f"yolo_debug_topic={self.yolo_debug_topic}")
        self.get_logger().info(f"annotated_image_topic={self.annotated_image_topic}")
        self.get_logger().info(f"yolo_table_topic={self.yolo_table_topic}")
        self.get_logger().info(f"frame_id={self.frame_id}")
        self.get_logger().info(f"ros_image_encoding={self.ros_image_encoding}")
        self.get_logger().info(f"force_usb2={self.force_usb2}")
        self.get_logger().info(f"stop_on_xlink_error={self.stop_on_xlink_error}")

    # =========================
    # DepthAI pipeline
    # =========================
    def get_sensor_resolution_enum(self):
        res = self.sensor_resolution.replace("_", "").replace("-", "")
        if res in ["1080p", "1080"]:
            return dai.ColorCameraProperties.SensorResolution.THE_1080_P
        if res in ["720p", "720"]:
            return dai.ColorCameraProperties.SensorResolution.THE_720_P
        if res in ["4k", "2160p", "2160"]:
            return dai.ColorCameraProperties.SensorResolution.THE_4_K
        if res in ["12mp", "12m"]:
            return dai.ColorCameraProperties.SensorResolution.THE_12_MP

        self.get_logger().warn(f"Unknown sensor_resolution={self.sensor_resolution}, fallback to 1080p")
        return dai.ColorCameraProperties.SensorResolution.THE_1080_P

    def create_pipeline(self):
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(self.get_sensor_resolution_enum())
        cam_rgb.setPreviewSize(self.preview_width, self.preview_height)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(self.fps)

        if hasattr(cam_rgb, "setPreviewKeepAspectRatio"):
            cam_rgb.setPreviewKeepAspectRatio(self.keep_aspect_ratio)

        if self.color_order == "BGR":
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        else:
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        if self.rgb_output == "video":
            cam_rgb.setVideoSize(self.video_width, self.video_height)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")

        if self.rgb_output == "video":
            cam_rgb.video.link(xout_rgb.input)
        else:
            cam_rgb.preview.link(xout_rgb.input)

        if self.enable_depth:
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_right = pipeline.create(dai.node.MonoCamera)
            stereo = pipeline.create(dai.node.StereoDepth)

            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            stereo.setOutputSize(self.preview_width, self.preview_height)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(False)

            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)

            xout_depth = pipeline.create(dai.node.XLinkOut)
            xout_depth.setStreamName("depth")
            stereo.depth.link(xout_depth.input)

        if self.enable_yolo:
            xout_nn = pipeline.create(dai.node.XLinkOut)
            xout_nn.setStreamName("nn")

            if self.parser == "YoloDetectionNetwork":
                yolo = pipeline.create(dai.node.YoloDetectionNetwork)
                yolo.setBlobPath(self.blob_path)
                yolo.setConfidenceThreshold(float(self.conf))
                yolo.setNumClasses(int(self.num_classes))
                yolo.setCoordinateSize(int(self.coordinate_size))
                yolo.setAnchors([float(x) for x in self.anchors])
                yolo.setAnchorMasks(
                    {
                        "side80": [int(x) for x in self.anchor_masks_side80],
                        "side40": [int(x) for x in self.anchor_masks_side40],
                        "side20": [int(x) for x in self.anchor_masks_side20],
                    }
                )
                yolo.setIouThreshold(float(self.iou))

                cam_rgb.preview.link(yolo.input)
                yolo.out.link(xout_nn.input)

                self.get_logger().info("OAK YOLO backend enabled: YoloDetectionNetwork")

            elif self.parser == "NeuralNetwork":
                nn = pipeline.create(dai.node.NeuralNetwork)
                nn.setBlobPath(self.blob_path)

                # NN always receives preview.
                cam_rgb.preview.link(nn.input)
                nn.out.link(xout_nn.input)

                self.get_logger().info(
                    "OAK NeuralNetwork backend enabled. NN inference runs on OAK; host decodes YOLO tensors."
                )
            else:
                raise RuntimeError(f"Unsupported parser={self.parser}")

        return pipeline

    def connect_oak(self):
        try:
            self.pipeline = self.create_pipeline()

            if self.force_usb2:
                self.get_logger().warn("Opening OAK with USB2 fallback: dai.UsbSpeed.HIGH")
                self.device = dai.Device(self.pipeline, dai.UsbSpeed.HIGH)
            else:
                self.device = dai.Device(self.pipeline)

            self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=self.queue_size, blocking=False)

            if self.enable_depth:
                self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=self.queue_size, blocking=False)

            if self.enable_yolo:
                self.nn_queue = self.device.getOutputQueue(name="nn", maxSize=self.queue_size, blocking=False)

            self.load_camera_intrinsics()
            self.oak_alive = True
            self.get_logger().info("OAK connected successfully")

        except Exception as e:
            self.oak_alive = False
            self.get_logger().error(f"Failed to connect OAK: {e}")
            if self.stop_on_xlink_error:
                raise

    def load_camera_intrinsics(self):
        try:
            calib = self.device.readCalibration()
            intr = calib.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A,
                self.rgb_width,
                self.rgb_height,
            )
            self.camera_intrinsics = intr
            self.get_logger().info(f"OAK RGB intrinsics for published image: {intr}")
        except Exception as e:
            self.camera_intrinsics = None
            self.get_logger().warn(f"Failed to read OAK calibration intrinsics: {e}")

    # =========================
    # YOLO decode helpers
    # =========================
    def sigmoid(self, x):
        return 1.0 / (1.0 + np.exp(-x))

    def get_anchor_pairs(self):
        pairs = []
        for i in range(0, len(self.anchors), 2):
            pairs.append((float(self.anchors[i]), float(self.anchors[i + 1])))
        return pairs

    def bbox_iou(self, a: Dict, b: Dict) -> float:
        ax1, ay1, ax2, ay2 = a["xmin"], a["ymin"], a["xmax"], a["ymax"]
        bx1, by1, bx2, by2 = b["xmin"], b["ymin"], b["xmax"], b["ymax"]

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)

        union = area_a + area_b - inter_area
        if union <= 0.0:
            return 0.0

        return inter_area / union

    def nms(self, detections: List[Dict], iou_thres: float) -> List[Dict]:
        if not detections:
            return []

        detections = sorted(detections, key=lambda x: x["confidence"], reverse=True)
        keep = []

        while detections:
            best = detections.pop(0)
            keep.append(best)
            detections = [det for det in detections if self.bbox_iou(best, det) < iou_thres]

        return keep

    def decode_neural_network_packet(self, nn_packet) -> List[Dict]:
        layer_names = list(nn_packet.getAllLayerNames())

        if not self.printed_nn_layers:
            sizes = {}
            for name in layer_names:
                try:
                    sizes[name] = len(nn_packet.getLayerFp16(name))
                except Exception:
                    sizes[name] = -1

            self.get_logger().info(f"NN layer_names={layer_names}, sizes={sizes}")
            self.printed_nn_layers = True

        attrs = 5 + int(self.num_classes)
        anchor_pairs = self.get_anchor_pairs()

        # For 640 input: 80,40,20. For other sizes: input/8,input/16,input/32.
        sides = [
            int(self.runtime_width // 8),
            int(self.runtime_width // 16),
            int(self.runtime_width // 32),
        ]

        side_to_mask = {
            sides[0]: self.anchor_masks_side80,
            sides[1]: self.anchor_masks_side40,
            sides[2]: self.anchor_masks_side20,
        }

        detections = []
        per_layer_stats = []

        for name in layer_names:
            raw = np.array(nn_packet.getLayerFp16(name), dtype=np.float32)

            matched_side = None
            for side in sides:
                expected = 3 * side * side * attrs
                if raw.size == expected:
                    matched_side = side
                    break

            if matched_side is None:
                continue

            side = matched_side
            mask = side_to_mask[side]
            arr = raw.reshape(3, side, side, attrs)

            layer_dets, stats = self.decode_yolov5_layer(
                arr=arr,
                side=side,
                mask=mask,
                anchor_pairs=anchor_pairs,
                attrs=attrs,
                layer_name=name,
            )
            detections.extend(layer_dets)
            per_layer_stats.append(stats)

        if self.debug_scores:
            self._score_log_count += 1
            if self._score_log_count % max(1, self.score_log_interval) == 0 and per_layer_stats:
                self.get_logger().info(" | ".join(per_layer_stats))

        return self.nms(detections, float(self.iou))

    def decode_yolov5_layer(
        self,
        arr: np.ndarray,
        side: int,
        mask: List[int],
        anchor_pairs: List[Tuple[float, float]],
        attrs: int,
        layer_name: str = "",
    ):
        dets = []
        stride = float(self.runtime_width) / float(side)

        max_obj_all = 0.0
        max_cls_all = 0.0
        max_score_all = 0.0

        for anchor_idx_in_layer, global_anchor_idx in enumerate(mask):
            if global_anchor_idx < 0 or global_anchor_idx >= len(anchor_pairs):
                continue

            anchor_w, anchor_h = anchor_pairs[global_anchor_idx]
            pred = arr[anchor_idx_in_layer]  # [side, side, attrs]

            tx = pred[:, :, 0]
            ty = pred[:, :, 1]
            tw = pred[:, :, 2]
            th = pred[:, :, 3]
            tobj = pred[:, :, 4]

            pred_5 = pred[:, :, :5]

            if self.output_value_mode == "sigmoid":
                already_sigmoid = True
            elif self.output_value_mode == "logits":
                already_sigmoid = False
            else:
                already_sigmoid = (np.nanmin(pred_5) >= 0.0 and np.nanmax(pred_5) <= 1.0)

            if already_sigmoid:
                sx, sy, sw, sh, obj = tx, ty, tw, th, tobj
            else:
                sx, sy, sw, sh, obj = self.sigmoid(tx), self.sigmoid(ty), self.sigmoid(tw), self.sigmoid(th), self.sigmoid(tobj)

            if self.num_classes == 1:
                cls_raw = pred[:, :, 5]

                if self.output_value_mode == "sigmoid":
                    cls_score = cls_raw
                elif self.output_value_mode == "logits":
                    cls_score = self.sigmoid(cls_raw)
                else:
                    cls_score = cls_raw if (np.nanmin(cls_raw) >= 0.0 and np.nanmax(cls_raw) <= 1.0) else self.sigmoid(cls_raw)

                if self.score_mode == "objectness":
                    score = obj
                else:
                    score = obj * cls_score

                class_id_map = np.zeros_like(score, dtype=np.int32)
            else:
                cls_raw = pred[:, :, 5:5 + self.num_classes]

                if self.output_value_mode == "sigmoid":
                    cls_scores = cls_raw
                elif self.output_value_mode == "logits":
                    cls_scores = self.sigmoid(cls_raw)
                else:
                    cls_scores = cls_raw if (np.nanmin(cls_raw) >= 0.0 and np.nanmax(cls_raw) <= 1.0) else self.sigmoid(cls_raw)

                class_id_map = np.argmax(cls_scores, axis=-1)
                best_cls_score = np.max(cls_scores, axis=-1)

                if self.score_mode == "objectness":
                    score = obj
                else:
                    score = obj * best_cls_score

                cls_score = best_cls_score

            max_obj_all = max(max_obj_all, float(np.nanmax(obj)))
            max_cls_all = max(max_cls_all, float(np.nanmax(cls_score)))
            max_score_all = max(max_score_all, float(np.nanmax(score)))

            ys, xs = np.where(score >= float(self.conf))

            for gy, gx in zip(ys, xs):
                conf = float(score[gy, gx])
                class_id = int(class_id_map[gy, gx])

                bx = (float(sx[gy, gx]) * 2.0 - 0.5 + float(gx)) * stride
                by = (float(sy[gy, gx]) * 2.0 - 0.5 + float(gy)) * stride
                bw = ((float(sw[gy, gx]) * 2.0) ** 2) * anchor_w
                bh = ((float(sh[gy, gx]) * 2.0) ** 2) * anchor_h

                x1 = bx - bw / 2.0
                y1 = by - bh / 2.0
                x2 = bx + bw / 2.0
                y2 = by + bh / 2.0

                x1 = max(0.0, min(float(self.runtime_width - 1), x1))
                y1 = max(0.0, min(float(self.runtime_height - 1), y1))
                x2 = max(0.0, min(float(self.runtime_width - 1), x2))
                y2 = max(0.0, min(float(self.runtime_height - 1), y2))

                if x2 <= x1 or y2 <= y1:
                    continue

                class_name = self.class_names[class_id] if 0 <= class_id < len(self.class_names) else str(class_id)

                dets.append(
                    {
                        "class_id": class_id,
                        "class_name": class_name,
                        "confidence": conf,
                        "xmin": x1,
                        "ymin": y1,
                        "xmax": x2,
                        "ymax": y2,
                    }
                )

        stats = (
            f"{layer_name}: side={side}, max_obj={max_obj_all:.3f}, "
            f"max_cls={max_cls_all:.3f}, max_score={max_score_all:.3f}"
        )

        return dets, stats

    def normalize_oak_detections(self, detections):
        out = []

        for det in detections:
            label_id = int(det.label)
            conf = float(det.confidence)

            xmin = max(0.0, min(1.0, float(det.xmin)))
            ymin = max(0.0, min(1.0, float(det.ymin)))
            xmax = max(0.0, min(1.0, float(det.xmax)))
            ymax = max(0.0, min(1.0, float(det.ymax)))

            if xmax <= xmin or ymax <= ymin:
                continue

            class_name = self.class_names[label_id] if 0 <= label_id < len(self.class_names) else str(label_id)

            out.append(
                {
                    "class_id": label_id,
                    "class_name": class_name,
                    "confidence": conf,
                    "xmin_norm": xmin,
                    "ymin_norm": ymin,
                    "xmax_norm": xmax,
                    "ymax_norm": ymax,
                    "xmin": xmin,
                    "ymin": ymin,
                    "xmax": xmax,
                    "ymax": ymax,
                }
            )

        return out

    def scale_detections_to_frame(self, detections, frame):
        if frame is None:
            return []

        h, w = frame.shape[:2]
        scaled = []

        for det in detections:
            d = dict(det)

            if "xmin_norm" in d:
                x1 = int(float(d["xmin_norm"]) * w)
                y1 = int(float(d["ymin_norm"]) * h)
                x2 = int(float(d["xmax_norm"]) * w)
                y2 = int(float(d["ymax_norm"]) * h)
            else:
                raw_x1 = float(d.get("xmin", 0.0))
                raw_y1 = float(d.get("ymin", 0.0))
                raw_x2 = float(d.get("xmax", 0.0))
                raw_y2 = float(d.get("ymax", 0.0))

                x1 = int(raw_x1 * w / float(self.runtime_width))
                y1 = int(raw_y1 * h / float(self.runtime_height))
                x2 = int(raw_x2 * w / float(self.runtime_width))
                y2 = int(raw_y2 * h / float(self.runtime_height))

            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w - 1, x2))
            y2 = max(0, min(h - 1, y2))

            if x2 <= x1 or y2 <= y1:
                continue

            d["xmin"] = x1
            d["ymin"] = y1
            d["xmax"] = x2
            d["ymax"] = y2
            d["bbox_width"] = x2 - x1
            d["bbox_height"] = y2 - y1
            d["center_x"] = (x1 + x2) / 2.0
            d["center_y"] = (y1 + y2) / 2.0
            d["image_width"] = w
            d["image_height"] = h

            scaled.append(d)

        return scaled

    # =========================
    # ROS message helpers
    # =========================
    def make_stamp(self):
        return self.get_clock().now().to_msg()

    def publish_camera_info(self, stamp):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = int(self.rgb_width)
        msg.height = int(self.rgb_height)

        if self.camera_intrinsics is not None:
            fx = float(self.camera_intrinsics[0][0])
            fy = float(self.camera_intrinsics[1][1])
            cx = float(self.camera_intrinsics[0][2])
            cy = float(self.camera_intrinsics[1][2])
        else:
            fx = 450.0
            fy = 450.0
            cx = self.rgb_width / 2.0
            cy = self.rgb_height / 2.0

        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.distortion_model = "plumb_bob"

        self.camera_info_pub.publish(msg)

    def publish_debug_text(self):
        if not self.publish_debug_text_enabled:
            return

        msg = String()
        msg.data = (
            f"backend={self.yolo_backend}, parser={self.parser}, "
            f"oak_alive={self.oak_alive}, detections={len(self.latest_frame_detections)}, "
            f"conf={self.conf}, iou={self.iou}, score_mode={self.score_mode}, "
            f"rgb_output={self.rgb_output}, rgb_size={self.rgb_width}x{self.rgb_height}, "
            f"nn_size={self.runtime_width}x{self.runtime_height}"
        )
        self.debug_pub.publish(msg)

    def publish_yolo_table(self):
        if not self.publish_yolo_table_enabled:
            return

        lines = []
        for i, det in enumerate(self.latest_frame_detections):
            lines.append(
                f"{i}: {det.get('class_name', det.get('class_id', '?'))} "
                f"conf={det.get('confidence', 0.0):.3f} "
                f"bbox=({det.get('xmin', 0)}, {det.get('ymin', 0)}, "
                f"{det.get('xmax', 0)}, {det.get('ymax', 0)})"
            )

        msg = String()
        msg.data = "\n".join(lines)
        self.table_pub.publish(msg)

    def set_if_exists(self, obj, name, value):
        if hasattr(obj, name):
            try:
                setattr(obj, name, value)
            except Exception:
                pass

    # def make_detection_msg(self, det: Dict):
    #     dmsg = YoloDetection()

    #     class_id = int(det.get("class_id", 0))
    #     class_name = str(det.get("class_name", class_id))
    #     confidence = float(det.get("confidence", 0.0))

    #     x1 = int(det.get("xmin", 0))
    #     y1 = int(det.get("ymin", 0))
    #     x2 = int(det.get("xmax", 0))
    #     y2 = int(det.get("ymax", 0))

    #     cx = float(det.get("center_x", (x1 + x2) / 2.0))
    #     cy = float(det.get("center_y", (y1 + y2) / 2.0))
    #     bw = float(det.get("bbox_width", x2 - x1))
    #     bh = float(det.get("bbox_height", y2 - y1))

    #     fields = [
    #         ("class_id", class_id),
    #         ("id", class_id),
    #         ("label_id", class_id),
    #         ("class_name", class_name),
    #         ("label", class_name),
    #         ("name", class_name),
    #         ("confidence", confidence),
    #         ("score", confidence),
    #         ("probability", confidence),
    #         ("xmin", x1),
    #         ("ymin", y1),
    #         ("xmax", x2),
    #         ("ymax", y2),
    #         ("x1", x1),
    #         ("y1", y1),
    #         ("x2", x2),
    #         ("y2", y2),
    #         ("center_x", cx),
    #         ("center_y", cy),
    #         ("cx", cx),
    #         ("cy", cy),
    #         ("width", bw),
    #         ("height", bh),
    #         ("bbox_width", bw),
    #         ("bbox_height", bh),
    #     ]

    #     for name, value in fields:
    #         self.set_if_exists(dmsg, name, value)

    #     if hasattr(dmsg, "bbox") and Box2D is not None:
    #         box = Box2D()
    #         box_fields = [
    #             ("xmin", x1),
    #             ("ymin", y1),
    #             ("xmax", x2),
    #             ("ymax", y2),
    #             ("x1", x1),
    #             ("y1", y1),
    #             ("x2", x2),
    #             ("y2", y2),
    #             ("center_x", cx),
    #             ("center_y", cy),
    #             ("cx", cx),
    #             ("cy", cy),
    #             ("width", bw),
    #             ("height", bh),
    #         ]
    #         for name, value in box_fields:
    #             self.set_if_exists(box, name, value)
    #         dmsg.bbox = box

    #     if hasattr(dmsg, "box") and Box2D is not None:
    #         box = Box2D()
    #         box_fields = [
    #             ("xmin", x1),
    #             ("ymin", y1),
    #             ("xmax", x2),
    #             ("ymax", y2),
    #             ("center_x", cx),
    #             ("center_y", cy),
    #             ("width", bw),
    #             ("height", bh),
    #         ]
    #         for name, value in box_fields:
    #             self.set_if_exists(box, name, value)
    #         dmsg.box = box

    #     return dmsg

    def make_detection_msg(self, det: Dict, stamp, index: int = 0):
        dmsg = YoloDetection()

        if hasattr(dmsg, "header"):
            dmsg.header.stamp = stamp
            dmsg.header.frame_id = self.frame_id

        class_id = int(det.get("class_id", 0))
        class_name = str(det.get("class_name", "target"))
        confidence = float(det.get("confidence", 0.0))

        x1 = int(det.get("xmin", 0))
        y1 = int(det.get("ymin", 0))
        x2 = int(det.get("xmax", 0))
        y2 = int(det.get("ymax", 0))

        width = max(0, x2 - x1)
        height = max(0, y2 - y1)
        center_u = int((x1 + x2) / 2)
        center_v = int((y1 + y2) / 2)

        if hasattr(dmsg, "target_id"):
            dmsg.target_id = f"oak_det_{index}"

        if hasattr(dmsg, "class_name"):
            dmsg.class_name = class_name

        if hasattr(dmsg, "class_id"):
            dmsg.class_id = class_id

        if hasattr(dmsg, "confidence"):
            dmsg.confidence = confidence

        if hasattr(dmsg, "need_ocr"):
            dmsg.need_ocr = True

        if hasattr(dmsg, "box") and Box2D is not None:
            box = Box2D()
            box.x1 = x1
            box.y1 = y1
            box.x2 = x2
            box.y2 = y2
            box.center_u = center_u
            box.center_v = center_v
            box.width = width
            box.height = height
            dmsg.box = box

        return dmsg

    # def publish_detections_msg(self, stamp):
    #     if not HAS_YOLO_MSG or self.detections_pub is None:
    #         return

    #     arr = YoloDetectionArray()

    #     if hasattr(arr, "header"):
    #         arr.header.stamp = stamp
    #         arr.header.frame_id = self.frame_id

    #     detections_list = [self.make_detection_msg(det) for det in self.latest_frame_detections]

    #     if hasattr(arr, "detections"):
    #         arr.detections = detections_list
    #     elif hasattr(arr, "results"):
    #         arr.results = detections_list
    #     elif hasattr(arr, "objects"):
    #         arr.objects = detections_list
    #     elif hasattr(arr, "yolo_detections"):
    #         arr.yolo_detections = detections_list

    #     self.detections_pub.publish(arr)

    def publish_detections_msg(self, stamp):
        if not HAS_YOLO_MSG or self.detections_pub is None:
            return

        arr = YoloDetectionArray()

        if hasattr(arr, "header"):
            arr.header.stamp = stamp
            arr.header.frame_id = self.frame_id

        detections_list = []
        for i, det in enumerate(self.latest_frame_detections):
            detections_list.append(self.make_detection_msg(det, stamp, i))

        arr.detections = detections_list
        self.detections_pub.publish(arr)


    def draw_detections(self, frame, detections):
        out = frame.copy()

        for det in detections:
            x1 = int(det["xmin"])
            y1 = int(det["ymin"])
            x2 = int(det["xmax"])
            y2 = int(det["ymax"])

            confidence = float(det.get("confidence", 0.0))
            class_name = str(det.get("class_name", det.get("class_id", "target")))

            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 0, 255), 2)

            text = f"{class_name} {confidence:.2f}"
            text_y = max(20, y1 - 6)

            cv2.putText(
                out,
                text,
                (x1, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        return out

    # =========================
    # Main loop
    # =========================
    def timer_callback(self):
        if not self.oak_alive:
            return

        stamp = self.make_stamp()

        try:
            rgb_packet = self.rgb_queue.tryGet() if self.rgb_queue is not None else None

            if rgb_packet is not None:
                rgb_frame = rgb_packet.getCvFrame()
                self.last_rgb_frame = rgb_frame

                rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding=self.ros_image_encoding)
                rgb_msg.header.stamp = stamp
                rgb_msg.header.frame_id = self.frame_id
                self.rgb_pub.publish(rgb_msg)
                self.publish_camera_info(stamp)

            if self.enable_depth and self.depth_queue is not None:
                depth_packet = self.depth_queue.tryGet()
                if depth_packet is not None:
                    depth_frame = depth_packet.getFrame()
                    self.last_depth_frame = depth_frame

                    depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="16UC1")
                    depth_msg.header.stamp = stamp
                    depth_msg.header.frame_id = self.frame_id
                    self.depth_pub.publish(depth_msg)

            if self.enable_yolo and self.nn_queue is not None:
                nn_packet = self.nn_queue.tryGet()
                if nn_packet is not None:
                    if self.parser == "YoloDetectionNetwork":
                        self.latest_detections = self.normalize_oak_detections(nn_packet.detections)
                    elif self.parser == "NeuralNetwork":
                        self.latest_detections = self.decode_neural_network_packet(nn_packet)
                    else:
                        self.latest_detections = []

            if self.last_rgb_frame is not None:
                self.latest_frame_detections = self.scale_detections_to_frame(
                    self.latest_detections,
                    self.last_rgb_frame,
                )
                # Second NMS after mapping to current published-image coordinates.
                self.latest_frame_detections = self.nms(self.latest_frame_detections, float(self.iou))
            else:
                self.latest_frame_detections = []

            if self.publish_annotated_image and self.last_rgb_frame is not None:
                annotated = self.last_rgb_frame.copy()

                if self.enable_yolo and self.draw_oak_detections_on_rgb and self.latest_frame_detections:
                    annotated = self.draw_detections(annotated, self.latest_frame_detections)

                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding=self.ros_image_encoding)
                annotated_msg.header.stamp = stamp
                annotated_msg.header.frame_id = self.frame_id
                self.annotated_pub.publish(annotated_msg)

            self.publish_detections_msg(stamp)
            self.publish_debug_text()
            self.publish_yolo_table()

        except Exception as e:
            self.get_logger().error(f"timer_callback error: {e}")

            if self.stop_on_xlink_error:
                raise


def main(args=None):
    rclpy.init(args=args)
    node = OakVisionDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
