from dataclasses import dataclass, field
from typing import Dict, List
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as RosTime
from std_msgs.msg import String, Int32, Bool

from yolo_msg.msg import YoloDetectionArray, YoloDetection, Box2D
from ocr_msg.msg import OcrRequestArray, OcrRequest, OcrResultArray

from target_matcher_msg.msg import (
    OcrLine,
    OcrCache as OcrCacheMsg,
    TrackedTarget,
    TrackedTargetArray,
    MatchedTarget,
)

from .kalman_tracker import KalmanTracker, Track, bbox_to_list
from .matcher_utils import (
    merge_ocr_lines,
    match_target_code,
    normalize_code,
    score_ocr_text,
)


@dataclass
class OcrCache:
    ocr_done: bool = False
    ocr_pending: bool = False
    lines: List[str] = field(default_factory=list)
    confidences: List[float] = field(default_factory=list)
    indices_list: List[List[int]] = field(default_factory=list)
    merged_text: str = ""
    norm_text: str = ""
    avg_conf: float = 0.0
    score: float = 0.0
    last_ocr_time: float = 0.0
    last_request_time: float = 0.0
    fail_count: int = 0
    matched_token: str = ""


class TargetMatcherNode(Node):
    def __init__(self):
        super().__init__("target_matcher_node")

        # =====================
        # Parameters
        # =====================
        self.declare_parameter("yolo_topic", "/yolov5/detections")
        self.declare_parameter("ocr_request_topic", "/vision/ocr_requests")
        self.declare_parameter("ocr_result_topic", "/vision/ocr_results")
        self.declare_parameter("target_code_topic", "/target_code")
        self.declare_parameter("enable_topic", "/target_matcher/enable")

        self.declare_parameter("matched_index_topic", "/target_matcher/matched_index")
        self.declare_parameter("matched_detection_topic", "/target_matcher/matched_detection")
        self.declare_parameter("tracked_detections_topic", "/target_matcher/tracked_detections")
        self.declare_parameter("debug_text_topic", "/target_matcher/debug_text")

        self.declare_parameter("tracked_targets_topic", "/target_matcher/tracked_targets")
        self.declare_parameter("matched_target_topic", "/target_matcher/matched_target")

        self.declare_parameter("track_iou_threshold", 0.10)
        self.declare_parameter("track_center_distance_threshold", 1.2)
        self.declare_parameter("max_missing_frames", 60)
        self.declare_parameter("min_yolo_confidence", 0.20)
        self.declare_parameter("only_need_ocr", True)

        self.declare_parameter("ocr_min_confidence", 0.30)
        self.declare_parameter("ocr_retry_interval_sec", 0.5)
        self.declare_parameter("ocr_cache_timeout_sec", 30.0)
        self.declare_parameter("avoid_repeat_ocr", False)

        self.declare_parameter("ocr_roi_expand_scale", 1.2)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        self.declare_parameter("reuse_track_indices", True)
        self.declare_parameter("tracking_enabled", True)
        self.declare_parameter("auto_enable_on_target_code", True)
        self.declare_parameter("require_target_code_for_ocr", False)

        self.declare_parameter("publish_debug_text", True)
        self.declare_parameter("target_code", "")

        # =====================
        # Load parameters
        # =====================
        self.yolo_topic = str(self.get_parameter("yolo_topic").value)
        self.ocr_request_topic = str(self.get_parameter("ocr_request_topic").value)
        self.ocr_result_topic = str(self.get_parameter("ocr_result_topic").value)
        self.target_code_topic = str(self.get_parameter("target_code_topic").value)
        self.enable_topic = str(self.get_parameter("enable_topic").value)

        self.matched_index_topic = str(self.get_parameter("matched_index_topic").value)
        self.matched_detection_topic = str(self.get_parameter("matched_detection_topic").value)
        self.tracked_detections_topic = str(self.get_parameter("tracked_detections_topic").value)
        self.debug_text_topic = str(self.get_parameter("debug_text_topic").value)
        self.tracked_targets_topic = str(self.get_parameter("tracked_targets_topic").value)
        self.matched_target_topic = str(self.get_parameter("matched_target_topic").value)

        self.track_iou_threshold = float(self.get_parameter("track_iou_threshold").value)
        self.track_center_distance_threshold = float(
            self.get_parameter("track_center_distance_threshold").value
        )
        self.max_missing_frames = int(self.get_parameter("max_missing_frames").value)
        self.min_yolo_confidence = float(self.get_parameter("min_yolo_confidence").value)
        self.only_need_ocr = bool(self.get_parameter("only_need_ocr").value)

        self.ocr_min_confidence = float(self.get_parameter("ocr_min_confidence").value)
        self.ocr_retry_interval_sec = float(self.get_parameter("ocr_retry_interval_sec").value)
        self.ocr_cache_timeout_sec = float(self.get_parameter("ocr_cache_timeout_sec").value)
        self.avoid_repeat_ocr = bool(self.get_parameter("avoid_repeat_ocr").value)

        self.ocr_roi_expand_scale = float(self.get_parameter("ocr_roi_expand_scale").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)

        self.reuse_track_indices = bool(self.get_parameter("reuse_track_indices").value)
        self.tracking_enabled = bool(self.get_parameter("tracking_enabled").value)
        self.auto_enable_on_target_code = bool(self.get_parameter("auto_enable_on_target_code").value)
        self.require_target_code_for_ocr = bool(
            self.get_parameter("require_target_code_for_ocr").value
        )
        self.publish_debug_text = bool(self.get_parameter("publish_debug_text").value)
        self.target_code = str(self.get_parameter("target_code").value or "").strip()

        if self.auto_enable_on_target_code and self.target_code:
            self.tracking_enabled = True

        # =====================
        # Tracker
        # =====================
        self.tracker = KalmanTracker(
            iou_threshold=self.track_iou_threshold,
            max_missing_frames=self.max_missing_frames,
            center_distance_threshold=self.track_center_distance_threshold,
        )

        # ret_data:
        # ret_data[0] = track info
        # ret_data[1] = OCR cache
        # ret_data[2] = matched index list
        self.ret_data = [[], [], []]

        self.track_id_to_index: Dict[str, int] = {}
        self.free_indices: List[int] = []
        self.last_header = None

        # =====================
        # Subscribers
        # =====================
        self.create_subscription(
            YoloDetectionArray,
            self.yolo_topic,
            self.on_yolo_detections,
            10,
        )

        self.create_subscription(
            OcrResultArray,
            self.ocr_result_topic,
            self.on_ocr_results,
            10,
        )

        self.create_subscription(
            String,
            self.target_code_topic,
            self.on_target_code,
            10,
        )

        self.create_subscription(
            Bool,
            self.enable_topic,
            self.on_enable,
            10,
        )

        # =====================
        # Publishers
        # =====================
        self.ocr_request_pub = self.create_publisher(
            OcrRequestArray,
            self.ocr_request_topic,
            10,
        )

        self.matched_index_pub = self.create_publisher(
            Int32,
            self.matched_index_topic,
            10,
        )

        self.matched_detection_pub = self.create_publisher(
            YoloDetectionArray,
            self.matched_detection_topic,
            10,
        )

        self.tracked_detections_pub = self.create_publisher(
            YoloDetectionArray,
            self.tracked_detections_topic,
            10,
        )

        self.debug_text_pub = self.create_publisher(
            String,
            self.debug_text_topic,
            10,
        )

        self.tracked_targets_pub = self.create_publisher(
            TrackedTargetArray,
            self.tracked_targets_topic,
            10,
        )

        self.matched_target_pub = self.create_publisher(
            MatchedTarget,
            self.matched_target_topic,
            10,
        )

        self.get_logger().info("target_matcher_node started.")
        self.get_logger().info(f"yolo_topic={self.yolo_topic}")
        self.get_logger().info(f"ocr_request_topic={self.ocr_request_topic}")
        self.get_logger().info(f"ocr_result_topic={self.ocr_result_topic}")
        self.get_logger().info(f"target_code_topic={self.target_code_topic}")
        self.get_logger().info(f"enable_topic={self.enable_topic}")
        self.get_logger().info(f"tracked_targets_topic={self.tracked_targets_topic}")
        self.get_logger().info(f"matched_target_topic={self.matched_target_topic}")
        self.get_logger().info(f"tracking_enabled={self.tracking_enabled}")
        self.get_logger().info(f"only_need_ocr={self.only_need_ocr}")
        self.get_logger().info(f"min_yolo_confidence={self.min_yolo_confidence}")
        self.get_logger().info(f"ocr_roi_expand_scale={self.ocr_roi_expand_scale}")
        self.get_logger().info(f"require_target_code_for_ocr={self.require_target_code_for_ocr}")
        self.get_logger().info(f"avoid_repeat_ocr={self.avoid_repeat_ocr}")

    # =====================
    # Callbacks
    # =====================
    def on_enable(self, msg: Bool):
        self.tracking_enabled = bool(msg.data)

        if not self.tracking_enabled:
            self.ret_data[2] = []
            self.publish_debug("Tracking task disabled. Matched target cleared.")
        else:
            self.publish_debug("Tracking task enabled.")

        self.get_logger().info(f"tracking_enabled updated: {self.tracking_enabled}")
        self.publish_outputs()

    def on_target_code(self, msg: String):
        self.target_code = str(msg.data or "").strip()

        if self.auto_enable_on_target_code and self.target_code:
            self.tracking_enabled = True

        self.get_logger().info(
            f"target_code updated: {self.target_code}, tracking_enabled={self.tracking_enabled}"
        )

        self.update_matches()
        self.publish_outputs()

    def on_yolo_detections(self, msg: YoloDetectionArray):
        self.last_header = msg.header

        detections = self.convert_yolo_detections(msg)

        if len(detections) == 0:
            tracks = self.tracker.update([])
            self.remove_dead_tracks(self.tracker.removed_track_ids)
            self.update_matches()
            self.publish_outputs()
            return

        tracks = self.tracker.update(detections)
        self.remove_dead_tracks(self.tracker.removed_track_ids)

        for track in tracks:
            self.ensure_track_slot(track)
            self.update_tracking_slot(track)

            if self.should_request_ocr(track):
                self.publish_ocr_request(track, msg.header)

        self.update_matches()
        self.publish_outputs()

    def on_ocr_results(self, msg: OcrResultArray):
        grouped = {}

        for result in msg.results:
            target_id = str(result.target_id)
            grouped.setdefault(target_id, []).append(result)

        for track_id, results in grouped.items():
            if track_id not in self.track_id_to_index:
                continue

            index = self.track_id_to_index[track_id]
            self.ensure_cache_slot(index)

            lines = []
            confidences = []
            indices_list = []

            for result in results:
                text = str(result.raw_text or "").strip()

                if not bool(result.has_text):
                    continue

                if not text:
                    continue

                lines.append(text)
                confidences.append(float(result.confidence))
                indices_list.append([int(x) for x in result.indices])

            cache: OcrCache = self.ret_data[1][index]

            if not lines:
                cache.ocr_pending = False
                cache.fail_count += 1
                self.get_logger().warn(
                    f"OCR returned no valid text: track_id={track_id}, index={index}, "
                    f"fail_count={cache.fail_count}"
                )
                continue

            merged_text = merge_ocr_lines(lines)
            avg_conf = sum(confidences) / max(1, len(confidences))
            new_score = score_ocr_text(lines, confidences)

            if cache.ocr_done and new_score < cache.score:
                cache.ocr_pending = False
                continue

            cache.ocr_done = True
            cache.ocr_pending = False
            cache.lines = lines
            cache.confidences = confidences
            cache.indices_list = indices_list
            cache.merged_text = merged_text
            cache.norm_text = normalize_code(merged_text)
            cache.avg_conf = float(avg_conf)
            cache.score = float(new_score)
            cache.last_ocr_time = time.time()
            cache.fail_count = 0
            cache.matched_token = ""

            self.get_logger().info(
                f"OCR cache updated: track_id={track_id}, index={index}, "
                f"text={merged_text}, avg_conf={avg_conf:.3f}"
            )

        self.update_matches()
        self.publish_outputs()

    # =====================
    # YOLO conversion
    # =====================
    def convert_yolo_detections(self, msg: YoloDetectionArray) -> List[dict]:
        converted = []

        for det in msg.detections:
            confidence = float(det.confidence)

            if confidence < self.min_yolo_confidence:
                continue

            bbox = bbox_to_list(det.box)

            if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                continue

            # 关键修复：
            # 不再用 det.need_ocr 过滤。
            # YOLO 节点一般不会给 need_ocr=True，所以这里不能卡住 OCR。
            converted.append({
                "bbox": bbox,
                "class_name": str(det.class_name),
                "class_id": int(det.class_id),
                "confidence": confidence,
                "need_ocr": True if self.only_need_ocr else bool(getattr(det, "need_ocr", True)),
                "source_target_id": str(det.target_id),
            })

        return converted

    # =====================
    # Track slot management
    # =====================
    def allocate_index(self, fallback_index: int) -> int:
        if self.reuse_track_indices and self.free_indices:
            return int(self.free_indices.pop(0))

        return int(fallback_index)

    def ensure_track_slot(self, track: Track):
        if track.track_id in self.track_id_to_index:
            return

        index = self.allocate_index(track.index)
        self.track_id_to_index[track.track_id] = index

        while len(self.ret_data[0]) <= index:
            self.ret_data[0].append(None)

        while len(self.ret_data[1]) <= index:
            self.ret_data[1].append(OcrCache())

        self.ret_data[0][index] = None
        self.ret_data[1][index] = OcrCache()

        self.get_logger().info(
            f"New track slot: track_id={track.track_id}, index={index}"
        )

    def ensure_cache_slot(self, index: int):
        while len(self.ret_data[1]) <= index:
            self.ret_data[1].append(OcrCache())

        if self.ret_data[1][index] is None:
            self.ret_data[1][index] = OcrCache()

    def update_tracking_slot(self, track: Track):
        index = self.track_id_to_index[track.track_id]

        self.ret_data[0][index] = {
            "track_id": track.track_id,
            "index": index,
            "bbox": list(track.bbox),
            "class_name": track.class_name,
            "class_id": int(track.class_id),
            "confidence": float(track.confidence),
            "missing_frames": int(track.missing_frames),
            "age": int(track.age),
            "need_ocr": True if self.only_need_ocr else bool(track.need_ocr),
        }

    def remove_dead_tracks(self, removed_track_ids: List[str]):
        for track_id in removed_track_ids:
            if track_id not in self.track_id_to_index:
                continue

            index = self.track_id_to_index.pop(track_id)

            if index < len(self.ret_data[0]):
                self.ret_data[0][index] = None

            if index < len(self.ret_data[1]):
                self.ret_data[1][index] = OcrCache()

            self.ret_data[2] = [idx for idx in self.ret_data[2] if idx != index]

            if self.reuse_track_indices and index not in self.free_indices:
                self.free_indices.append(index)
                self.free_indices.sort()

            self.get_logger().info(
                f"Removed track: track_id={track_id}, index={index}; "
                f"free_indices={len(self.free_indices)}"
            )

    # =====================
    # OCR request logic
    # =====================
    def should_request_ocr(self, track: Track) -> bool:
        if not self.tracking_enabled:
            return False

        if self.require_target_code_for_ocr and not self.target_code.strip():
            return False

        # 关键修复：
        # only_need_ocr=True 时，不能再检查 track.need_ocr。
        # 因为 YOLO detection 的 need_ocr 默认通常是 False。
        # 调试模式下只要有合法 track，就允许 OCR。
        index = self.track_id_to_index[track.track_id]
        cache: OcrCache = self.ret_data[1][index]
        now = time.time()

        if cache.ocr_pending:
            return False

        if cache.ocr_done and cache.avg_conf >= self.ocr_min_confidence:
            if self.avoid_repeat_ocr:
                return False

            if self.ocr_cache_timeout_sec <= 0:
                return False

            if now - cache.last_ocr_time < self.ocr_cache_timeout_sec:
                return False

        if now - cache.last_request_time < self.ocr_retry_interval_sec:
            return False

        return True

    def expand_bbox(self, bbox: List[float], scale: float) -> List[float]:
        x1, y1, x2, y2 = bbox

        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        width = max(1.0, x2 - x1)
        height = max(1.0, y2 - y1)

        new_width = width * float(scale)
        new_height = height * float(scale)

        nx1 = max(0.0, cx - new_width * 0.5)
        ny1 = max(0.0, cy - new_height * 0.5)
        nx2 = min(float(self.image_width - 1), cx + new_width * 0.5)
        ny2 = min(float(self.image_height - 1), cy + new_height * 0.5)

        return [nx1, ny1, nx2, ny2]

    def publish_ocr_request(self, track: Track, header):
        index = self.track_id_to_index[track.track_id]
        cache: OcrCache = self.ret_data[1][index]

        raw_bbox = list(track.bbox)
        ocr_bbox = self.expand_bbox(raw_bbox, self.ocr_roi_expand_scale)

        req_array = OcrRequestArray()
        req_array.header = header

        req = OcrRequest()
        req.header = header
        req.target_id = str(track.track_id)
        req.class_name = str(track.class_name)
        req.yolo_confidence = float(track.confidence)
        req.need_ocr = True
        req.box = self.make_box2d_from_bbox(ocr_bbox)

        req_array.requests.append(req)

        cache.ocr_pending = True
        cache.last_request_time = time.time()

        self.ocr_request_pub.publish(req_array)

        text = (
            f"OCR request sent: track_id={track.track_id}, index={index}, "
            f"confidence={track.confidence:.3f}, raw_bbox={raw_bbox}, ocr_bbox={ocr_bbox}"
        )

        self.get_logger().info(text)
        self.publish_debug(text)

    # =====================
    # Matching
    # =====================
    def update_matches(self):
        self.ret_data[2] = []

        if not self.tracking_enabled:
            return

        if not self.target_code.strip():
            return

        for index, cache in enumerate(self.ret_data[1]):
            if cache is None:
                continue

            if not isinstance(cache, OcrCache):
                continue

            if not cache.ocr_done:
                continue

            ok, token = match_target_code(self.target_code, cache.merged_text)

            if ok:
                cache.matched_token = token
                self.ret_data[2].append(index)
                self.publish_debug(
                    f"MATCH index={index}, token={token}, text={cache.merged_text}"
                )
            else:
                cache.matched_token = ""

    # =====================
    # Publish outputs
    # =====================
    def publish_outputs(self):
        self.publish_matched_index()
        self.publish_all_tracked_detections()
        self.publish_all_tracked_targets()
        self.publish_matched_detection_array()
        self.publish_matched_target()

    def publish_matched_index(self):
        msg = Int32()
        msg.data = int(self.ret_data[2][0]) if self.ret_data[2] else -1
        self.matched_index_pub.publish(msg)

    def publish_all_tracked_detections(self):
        arr = YoloDetectionArray()

        if self.last_header is not None:
            arr.header = self.last_header

        for track_info in self.ret_data[0]:
            if track_info:
                arr.detections.append(self.make_detection_from_track_info(track_info))

        self.tracked_detections_pub.publish(arr)

    def publish_all_tracked_targets(self):
        arr = TrackedTargetArray()

        if self.last_header is not None:
            arr.header = self.last_header

        for index, track_info in enumerate(self.ret_data[0]):
            if not track_info:
                continue

            cache = self.ret_data[1][index] if index < len(self.ret_data[1]) else OcrCache()
            arr.targets.append(self.make_tracked_target_msg(index, track_info, cache))

        self.tracked_targets_pub.publish(arr)

    def publish_matched_detection_array(self):
        arr = YoloDetectionArray()

        if self.last_header is not None:
            arr.header = self.last_header

        if self.ret_data[2]:
            index = int(self.ret_data[2][0])

            if index < len(self.ret_data[0]) and self.ret_data[0][index]:
                arr.detections.append(
                    self.make_detection_from_track_info(self.ret_data[0][index])
                )

        self.matched_detection_pub.publish(arr)

    def publish_matched_target(self):
        msg = MatchedTarget()

        if self.last_header is not None:
            msg.header = self.last_header

        msg.target_code = str(self.target_code)
        msg.matched = False
        msg.matched_index = -1
        msg.box = self.make_box2d_from_bbox([0.0, 0.0, 0.0, 0.0])
        msg.ocr_cache = self.make_ocr_cache_msg(OcrCache())

        if self.ret_data[2]:
            index = int(self.ret_data[2][0])

            if index < len(self.ret_data[0]) and self.ret_data[0][index]:
                track_info = self.ret_data[0][index]
                cache = self.ret_data[1][index]

                msg.matched = True
                msg.matched_index = index
                msg.track_id = str(track_info["track_id"])
                msg.target_code = str(self.target_code)
                msg.matched_token = str(cache.matched_token if isinstance(cache, OcrCache) else "")
                msg.class_name = str(track_info["class_name"])
                msg.class_id = int(track_info["class_id"])
                msg.yolo_confidence = float(track_info["confidence"])
                msg.need_ocr = bool(track_info["need_ocr"])
                msg.box = self.make_box2d_from_bbox(track_info["bbox"])
                msg.age = int(track_info.get("age", 0))
                msg.missing_frames = int(track_info.get("missing_frames", 0))
                msg.ocr_cache = self.make_ocr_cache_msg(cache)

        self.matched_target_pub.publish(msg)

    # =====================
    # Message helpers
    # =====================
    def make_tracked_target_msg(
        self,
        index: int,
        track_info: dict,
        cache: OcrCache,
    ) -> TrackedTarget:
        msg = TrackedTarget()

        if self.last_header is not None:
            msg.header = self.last_header

        msg.index = int(index)
        msg.track_id = str(track_info["track_id"])
        msg.class_name = str(track_info["class_name"])
        msg.class_id = int(track_info["class_id"])
        msg.yolo_confidence = float(track_info["confidence"])
        msg.need_ocr = bool(track_info["need_ocr"])
        msg.box = self.make_box2d_from_bbox(track_info["bbox"])
        msg.age = int(track_info.get("age", 0))
        msg.missing_frames = int(track_info.get("missing_frames", 0))

        msg.has_ocr = bool(isinstance(cache, OcrCache) and cache.ocr_done)
        msg.ocr_cache = self.make_ocr_cache_msg(
            cache if isinstance(cache, OcrCache) else OcrCache()
        )

        msg.matched = int(index) in self.ret_data[2]
        msg.matched_token = str(cache.matched_token if isinstance(cache, OcrCache) else "")

        return msg

    def make_ocr_cache_msg(self, cache: OcrCache) -> OcrCacheMsg:
        msg = OcrCacheMsg()

        msg.ocr_done = bool(cache.ocr_done)
        msg.ocr_pending = bool(cache.ocr_pending)
        msg.merged_text = str(cache.merged_text)
        msg.norm_text = str(cache.norm_text)
        msg.avg_confidence = float(cache.avg_conf)
        msg.score = float(cache.score)
        msg.fail_count = int(cache.fail_count)
        msg.last_ocr_stamp = self.float_time_to_ros_time(cache.last_ocr_time)

        for line_id, text in enumerate(cache.lines):
            line_msg = OcrLine()
            line_msg.line_id = int(line_id)
            line_msg.text = str(text)
            line_msg.confidence = (
                float(cache.confidences[line_id])
                if line_id < len(cache.confidences)
                else 0.0
            )

            if line_id < len(cache.indices_list):
                line_msg.indices = [int(x) for x in cache.indices_list[line_id]]

            msg.lines.append(line_msg)

        return msg

    def float_time_to_ros_time(self, value: float) -> RosTime:
        msg = RosTime()

        if not value or value <= 0:
            return msg

        sec = int(value)
        msg.sec = sec
        msg.nanosec = int((float(value) - sec) * 1e9)

        return msg

    def make_detection_from_track_info(self, track_info) -> YoloDetection:
        det = YoloDetection()

        if self.last_header is not None:
            det.header = self.last_header

        det.target_id = str(track_info["track_id"])
        det.class_name = str(track_info["class_name"])
        det.class_id = int(track_info["class_id"])
        det.confidence = float(track_info["confidence"])
        det.box = self.make_box2d_from_bbox(track_info["bbox"])
        det.need_ocr = bool(track_info.get("need_ocr", True))

        return det

    def make_box2d_from_bbox(self, bbox: List[float]) -> Box2D:
        x1, y1, x2, y2 = bbox

        box = Box2D()
        box.x1 = int(round(x1))
        box.y1 = int(round(y1))
        box.x2 = int(round(x2))
        box.y2 = int(round(y2))
        box.center_u = int(round((x1 + x2) * 0.5))
        box.center_v = int(round((y1 + y2) * 0.5))
        box.width = int(round(max(0.0, x2 - x1)))
        box.height = int(round(max(0.0, y2 - y1)))

        return box

    def publish_debug(self, text: str):
        if not self.publish_debug_text:
            return

        msg = String()
        msg.data = str(text)
        self.debug_text_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TargetMatcherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()