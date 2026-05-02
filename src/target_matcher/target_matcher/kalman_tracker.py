from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np


def bbox_to_list(box) -> List[float]:
    return [float(box.x1), float(box.y1), float(box.x2), float(box.y2)]


def bbox_center(bbox: List[float]) -> Tuple[float, float]:
    x1, y1, x2, y2 = bbox
    return 0.5 * (x1 + x2), 0.5 * (y1 + y2)


def bbox_size(bbox: List[float]) -> Tuple[float, float]:
    x1, y1, x2, y2 = bbox
    return max(1.0, x2 - x1), max(1.0, y2 - y1)


def make_bbox_from_center(cx: float, cy: float, w: float, h: float) -> List[float]:
    return [cx - w * 0.5, cy - h * 0.5, cx + w * 0.5, cy + h * 0.5]


def bbox_iou(a: List[float], b: List[float]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return 0.0 if union <= 0.0 else float(inter / union)


def normalized_center_distance(a: List[float], b: List[float]) -> float:
    acx, acy = bbox_center(a)
    bcx, bcy = bbox_center(b)
    aw, ah = bbox_size(a)
    bw, bh = bbox_size(b)
    scale = max(1.0, 0.5 * (aw + ah + bw + bh) * 0.5)
    return float(np.hypot(acx - bcx, acy - bcy) / scale)


class SimpleKalman2D:
    """Constant-velocity 2D Kalman filter for bbox center."""

    def __init__(self, x: float, y: float):
        self.x = np.array([[x], [y], [0.0], [0.0]], dtype=np.float32)
        self.P = np.eye(4, dtype=np.float32) * 50.0
        self.F = np.eye(4, dtype=np.float32)
        self.H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float32)
        self.Q = np.eye(4, dtype=np.float32) * 1.0
        self.R = np.eye(2, dtype=np.float32) * 18.0

    def predict(self, dt: float = 1.0) -> Tuple[float, float]:
        dt = max(1e-3, float(dt))
        self.F = np.array(
            [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return float(self.x[0, 0]), float(self.x[1, 0])

    def update(self, x_meas: float, y_meas: float) -> Tuple[float, float]:
        z = np.array([[x_meas], [y_meas]], dtype=np.float32)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(4, dtype=np.float32)
        self.P = (I - K @ self.H) @ self.P
        return float(self.x[0, 0]), float(self.x[1, 0])


@dataclass
class Track:
    track_id: str
    index: int
    bbox: List[float]
    class_name: str = "unknown"
    class_id: int = -1
    confidence: float = 0.0
    need_ocr: bool = True
    age: int = 1
    missing_frames: int = 0
    width: float = 1.0
    height: float = 1.0
    kalman: SimpleKalman2D = field(default=None)

    def __post_init__(self):
        cx, cy = bbox_center(self.bbox)
        self.width, self.height = bbox_size(self.bbox)
        if self.kalman is None:
            self.kalman = SimpleKalman2D(cx, cy)

    def predict(self):
        cx, cy = self.kalman.predict()
        self.bbox = make_bbox_from_center(cx, cy, self.width, self.height)

    def update_with_detection(self, det_bbox: List[float], confidence: float, class_name: str, class_id: int, need_ocr: bool):
        cx, cy = bbox_center(det_bbox)
        kx, ky = self.kalman.update(cx, cy)
        self.width, self.height = bbox_size(det_bbox)
        self.bbox = make_bbox_from_center(kx, ky, self.width, self.height)
        self.confidence = float(confidence)
        self.class_name = str(class_name)
        self.class_id = int(class_id)
        self.need_ocr = bool(need_ocr)
        self.age += 1
        self.missing_frames = 0


class KalmanTracker:
    def __init__(self, iou_threshold: float = 0.1, max_missing_frames: int = 60, center_distance_threshold: float = 1.2):
        self.iou_threshold = float(iou_threshold)
        self.max_missing_frames = int(max_missing_frames)
        self.center_distance_threshold = float(center_distance_threshold)
        self.tracks: List[Track] = []
        self.next_track_number = 0
        self.next_index = 0
        self.removed_track_ids: List[str] = []

    def _new_track_id(self) -> str:
        track_id = f"track_{self.next_track_number}"
        self.next_track_number += 1
        return track_id

    def update(self, detections: List[dict]) -> List[Track]:
        self.removed_track_ids = []

        for track in self.tracks:
            track.predict()

        unmatched_tracks = set(range(len(self.tracks)))
        unmatched_dets = set(range(len(detections)))
        matches = []

        if self.tracks and detections:
            pairs = []
            for ti, track in enumerate(self.tracks):
                for di, det in enumerate(detections):
                    iou = bbox_iou(track.bbox, det["bbox"])
                    dist = normalized_center_distance(track.bbox, det["bbox"])
                    # Higher is better. Distance gives association even when IoU is unstable.
                    score = iou + max(0.0, 1.0 - dist)
                    if iou >= self.iou_threshold or dist <= self.center_distance_threshold:
                        pairs.append((score, ti, di))
            pairs.sort(reverse=True, key=lambda x: x[0])

            used_t, used_d = set(), set()
            for _, ti, di in pairs:
                if ti in used_t or di in used_d:
                    continue
                matches.append((ti, di))
                used_t.add(ti)
                used_d.add(di)
            unmatched_tracks -= used_t
            unmatched_dets -= used_d

        for ti, di in matches:
            track = self.tracks[ti]
            det = detections[di]
            track.update_with_detection(det["bbox"], det["confidence"], det["class_name"], det["class_id"], det["need_ocr"])

        for ti in unmatched_tracks:
            self.tracks[ti].missing_frames += 1
            self.tracks[ti].age += 1

        for di in unmatched_dets:
            det = detections[di]
            track = Track(
                track_id=self._new_track_id(),
                index=self.next_index,
                bbox=det["bbox"],
                class_name=det["class_name"],
                class_id=det["class_id"],
                confidence=det["confidence"],
                need_ocr=det["need_ocr"],
            )
            self.next_index += 1
            self.tracks.append(track)

        alive = []
        for track in self.tracks:
            if track.missing_frames > self.max_missing_frames:
                self.removed_track_ids.append(track.track_id)
            else:
                alive.append(track)
        self.tracks = alive
        return self.tracks
