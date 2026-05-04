#!/usr/bin/env python3
from typing import Dict, List, Tuple

import cv2
import numpy as np


class YoloV5OakDecoder:
    """
    YOLOv5 decoder for OAK NeuralNetwork output.

    Supports:
      1. Raw YOLOv5 heads:
         [1,3,80,80,5+C], [1,3,40,40,5+C], [1,3,20,20,5+C]

      2. Decoded / concat output:
         [1,25200,5+C]

    Output detection format:
      {
        "label": int,
        "confidence": float,
        "xmin": float,   # normalized 0~1
        "ymin": float,
        "xmax": float,
        "ymax": float,
      }
    """

    def __init__(
        self,
        input_width: int = 640,
        input_height: int = 640,
        num_classes: int = 1,
        conf_thres: float = 0.25,
        iou_thres: float = 0.45,
        class_names=None,
        anchors=None,
        debug: bool = False,
    ):
        self.input_width = int(input_width)
        self.input_height = int(input_height)
        self.num_classes = int(num_classes)
        self.conf_thres = float(conf_thres)
        self.iou_thres = float(iou_thres)
        self.class_names = class_names if class_names is not None else ["target"]
        self.debug = bool(debug)

        if anchors is None:
            self.anchors = {
                80: np.array([[10, 13], [16, 30], [33, 23]], dtype=np.float32),
                40: np.array([[30, 61], [62, 45], [59, 119]], dtype=np.float32),
                20: np.array([[116, 90], [156, 198], [373, 326]], dtype=np.float32),
            }
        else:
            flat = np.array(anchors, dtype=np.float32).reshape(-1, 2)
            if flat.shape[0] < 9:
                raise ValueError("anchors must contain 9 anchor pairs")
            self.anchors = {
                80: flat[0:3],
                40: flat[3:6],
                20: flat[6:9],
            }

        self.strides = {
            80: 8.0,
            40: 16.0,
            20: 32.0,
        }

        self.last_layer_info = ""

    @staticmethod
    def sigmoid(x):
        return 1.0 / (1.0 + np.exp(-x))

    def decode_packet(self, nn_packet) -> List[Dict[str, float]]:
        layer_names = nn_packet.getAllLayerNames()

        if not layer_names:
            self.last_layer_info = "no layers"
            return []

        layers = []
        layer_info_parts = []

        for name in layer_names:
            data = np.array(nn_packet.getLayerFp16(name), dtype=np.float32)
            layers.append((name, data))

            if data.size > 0:
                layer_info_parts.append(
                    f"{name}:size={data.size},min={float(data.min()):.4f},max={float(data.max()):.4f}"
                )
            else:
                layer_info_parts.append(f"{name}:size=0")

        self.last_layer_info = "; ".join(layer_info_parts)

        if self.debug:
            print("[YoloV5OakDecoder] " + self.last_layer_info)

        stride_len = 5 + self.num_classes

        raw_heads = self._find_raw_heads(layers, stride_len)

        if len(raw_heads) > 0:
            detections = self._decode_raw_heads(raw_heads, stride_len)
            return self.apply_nms(detections)

        detections = self._decode_concat_output(layers, stride_len)
        return self.apply_nms(detections)

    def _find_raw_heads(
        self,
        layers: List[Tuple[str, np.ndarray]],
        stride_len: int,
    ):
        expected_sizes = {
            80: 3 * 80 * 80 * stride_len,
            40: 3 * 40 * 40 * stride_len,
            20: 3 * 20 * 20 * stride_len,
        }

        raw_heads = []

        for name, data in layers:
            for side, expected_size in expected_sizes.items():
                if data.size == expected_size:
                    raw_heads.append((side, name, data))

        raw_heads.sort(key=lambda x: -x[0])
        return raw_heads

    def _decode_raw_heads(
        self,
        raw_heads: List[Tuple[int, str, np.ndarray]],
        stride_len: int,
    ) -> List[Dict[str, float]]:
        decoded_all = []

        for side, name, data in raw_heads:
            anchors = self.anchors[side]
            stride_px = self.strides[side]

            raw = data.reshape(1, 3, side, side, stride_len)[0]

            # Default: raw logits -> sigmoid.
            # If boxes are still messy later, change this line to: pred = raw
            pred = self.sigmoid(raw)

            for a in range(3):
                anchor_w = float(anchors[a, 0])
                anchor_h = float(anchors[a, 1])

                for gy in range(side):
                    for gx in range(side):
                        row = pred[a, gy, gx]

                        obj = float(row[4])

                        if self.num_classes > 0:
                            class_scores = row[5:5 + self.num_classes]
                            label_id = int(np.argmax(class_scores))
                            cls_conf = float(class_scores[label_id])
                        else:
                            label_id = 0
                            cls_conf = 1.0

                        score = obj * cls_conf

                        if score < self.conf_thres:
                            continue

                        # YOLOv5 Detect.forward decode:
                        # xy = (sigmoid(xy) * 2 - 0.5 + grid) * stride
                        # wh = (sigmoid(wh) * 2)^2 * anchor
                        cx = (float(row[0]) * 2.0 - 0.5 + gx) * stride_px
                        cy = (float(row[1]) * 2.0 - 0.5 + gy) * stride_px
                        bw = ((float(row[2]) * 2.0) ** 2) * anchor_w
                        bh = ((float(row[3]) * 2.0) ** 2) * anchor_h

                        det = self._make_detection(
                            label_id=label_id,
                            score=score,
                            cx=cx,
                            cy=cy,
                            bw=bw,
                            bh=bh,
                        )

                        if det is not None:
                            decoded_all.append(det)

        return decoded_all

    def _decode_concat_output(
        self,
        layers: List[Tuple[str, np.ndarray]],
        stride_len: int,
    ) -> List[Dict[str, float]]:
        best_data = None
        best_name = None

        for name, data in layers:
            if best_data is None or data.size > best_data.size:
                best_data = data
                best_name = name

        if best_data is None or best_data.size == 0:
            return []

        if best_data.size % stride_len != 0:
            if self.debug:
                print(
                    f"[YoloV5OakDecoder] output size {best_data.size} "
                    f"is not divisible by {stride_len}, layer={best_name}"
                )
            return []

        arr = best_data.reshape(-1, stride_len)

        coord_max = float(np.max(np.abs(arr[:, 0:4]))) if arr.shape[0] > 0 else 0.0
        coords_are_normalized = coord_max <= 2.0

        decoded_all = []

        for row in arr:
            cx = float(row[0])
            cy = float(row[1])
            bw = float(row[2])
            bh = float(row[3])
            obj = float(row[4])

            if coords_are_normalized:
                cx *= self.input_width
                bw *= self.input_width
                cy *= self.input_height
                bh *= self.input_height

            if self.num_classes > 0:
                class_scores = row[5:5 + self.num_classes]
                label_id = int(np.argmax(class_scores))
                cls_conf = float(class_scores[label_id])
            else:
                label_id = 0
                cls_conf = 1.0

            score = obj * cls_conf

            if score < self.conf_thres:
                continue

            det = self._make_detection(
                label_id=label_id,
                score=score,
                cx=cx,
                cy=cy,
                bw=bw,
                bh=bh,
            )

            if det is not None:
                decoded_all.append(det)

        return decoded_all

    def _make_detection(
        self,
        label_id: int,
        score: float,
        cx: float,
        cy: float,
        bw: float,
        bh: float,
    ):
        x1 = cx - bw / 2.0
        y1 = cy - bh / 2.0
        x2 = cx + bw / 2.0
        y2 = cy + bh / 2.0

        x1 = max(0.0, min(float(self.input_width - 1), x1))
        y1 = max(0.0, min(float(self.input_height - 1), y1))
        x2 = max(0.0, min(float(self.input_width - 1), x2))
        y2 = max(0.0, min(float(self.input_height - 1), y2))

        if x2 <= x1 or y2 <= y1:
            return None

        return {
            "label": int(label_id),
            "confidence": float(score),
            "xmin": x1 / float(self.input_width),
            "ymin": y1 / float(self.input_height),
            "xmax": x2 / float(self.input_width),
            "ymax": y2 / float(self.input_height),
        }

    def apply_nms(self, detections: List[Dict[str, float]]) -> List[Dict[str, float]]:
        if len(detections) == 0:
            return []

        boxes = []
        scores = []

        for det in detections:
            x1 = int(det["xmin"] * self.input_width)
            y1 = int(det["ymin"] * self.input_height)
            x2 = int(det["xmax"] * self.input_width)
            y2 = int(det["ymax"] * self.input_height)

            boxes.append([x1, y1, max(0, x2 - x1), max(0, y2 - y1)])
            scores.append(float(det["confidence"]))

        indices = cv2.dnn.NMSBoxes(
            bboxes=boxes,
            scores=scores,
            score_threshold=float(self.conf_thres),
            nms_threshold=float(self.iou_thres),
        )

        if len(indices) == 0:
            return []

        result = []
        for idx in np.array(indices).flatten():
            result.append(detections[int(idx)])

        return result

    def draw_detections(
        self,
        frame,
        detections: List[Dict[str, float]],
    ):
        vis = frame.copy()
        h, w = vis.shape[:2]

        for det in detections:
            x1 = int(det["xmin"] * w)
            y1 = int(det["ymin"] * h)
            x2 = int(det["xmax"] * w)
            y2 = int(det["ymax"] * h)

            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w - 1, x2))
            y2 = max(0, min(h - 1, y2))

            label_id = int(det["label"])
            label = "target"
            if 0 <= label_id < len(self.class_names):
                label = str(self.class_names[label_id])

            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(
                vis,
                f"{label} {det['confidence']:.2f}",
                (x1, max(20, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )

        return vis

    def make_table(
        self,
        detections: List[Dict[str, float]],
    ) -> str:
        lines = []
        lines.append("idx | class | conf | cx_px | cy_px | w_px | h_px | x1 | y1 | x2 | y2")
        lines.append("-" * 78)

        for i, det in enumerate(detections):
            label_id = int(det["label"])
            class_name = "target"
            if 0 <= label_id < len(self.class_names):
                class_name = str(self.class_names[label_id])

            x1 = int(det["xmin"] * self.input_width)
            y1 = int(det["ymin"] * self.input_height)
            x2 = int(det["xmax"] * self.input_width)
            y2 = int(det["ymax"] * self.input_height)

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            bw = int(x2 - x1)
            bh = int(y2 - y1)

            lines.append(
                f"{i} | {class_name} | {det['confidence']:.3f} | "
                f"{cx} | {cy} | {bw} | {bh} | {x1} | {y1} | {x2} | {y2}"
            )

        return "\n".join(lines)