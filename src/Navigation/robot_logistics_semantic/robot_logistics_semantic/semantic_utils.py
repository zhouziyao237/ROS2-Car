#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple


def now_sec() -> float:
    return time.time()


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_from_yaw(yaw: float) -> Tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def pose_dict(x=0.0, y=0.0, yaw=0.0) -> Dict[str, float]:
    return {"x": float(x), "y": float(y), "yaw": float(norm_angle(yaw))}


def apply_transform(t: Dict[str, float], p: Dict[str, float]) -> Dict[str, float]:
    cy = math.cos(t.get('yaw', 0.0))
    sy = math.sin(t.get('yaw', 0.0))
    x = t.get('x', 0.0) + cy * p.get('x', 0.0) - sy * p.get('y', 0.0)
    y = t.get('y', 0.0) + sy * p.get('x', 0.0) + cy * p.get('y', 0.0)
    yaw = norm_angle(t.get('yaw', 0.0) + p.get('yaw', 0.0))
    return pose_dict(x, y, yaw)


def inverse_transform(t: Dict[str, float]) -> Dict[str, float]:
    yaw = -t.get('yaw', 0.0)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    tx = -t.get('x', 0.0)
    ty = -t.get('y', 0.0)
    x = cy * tx - sy * ty
    y = sy * tx + cy * ty
    return pose_dict(x, y, yaw)


def transform_from_pair(p_sem: Dict[str, float], p_map: Dict[str, float]) -> Dict[str, float]:
    # T_map_sem such that p_map ~= T_map_sem * p_sem
    yaw = norm_angle(p_map.get('yaw', 0.0) - p_sem.get('yaw', 0.0))
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    x = p_map.get('x', 0.0) - (cy * p_sem.get('x', 0.0) - sy * p_sem.get('y', 0.0))
    y = p_map.get('y', 0.0) - (sy * p_sem.get('x', 0.0) + cy * p_sem.get('y', 0.0))
    return pose_dict(x, y, yaw)


def default_library(run_id: str = '') -> Dict[str, Any]:
    return {
        "version": 1,
        "semantic_frame": "semantic_world",
        "last_update": now_sec(),
        "relocalization": {
            "current_run_id": run_id,
            "has_transform": False,
            "T_map_semantic": pose_dict(),
            "matched_anchors": [],
            "confidence": 0.0,
        },
        "shelves": {},
    }


class SemanticLibraryStore:
    def __init__(self, path: str, run_id: str = ''):
        self.path = Path(path)
        self.run_id = run_id

    def load(self) -> Dict[str, Any]:
        if not self.path.exists():
            data = default_library(self.run_id)
            self.save(data)
            return data
        try:
            text = self.path.read_text().strip()
            if not text:
                data = default_library(self.run_id)
                self.save(data)
                return data
            data = json.loads(text)
            if not isinstance(data, dict):
                data = default_library(self.run_id)
            data.setdefault('version', 1)
            data.setdefault('semantic_frame', 'semantic_world')
            data.setdefault('last_update', now_sec())
            data.setdefault('relocalization', default_library(self.run_id)['relocalization'])
            data.setdefault('shelves', {})
            if not isinstance(data['shelves'], dict):
                data['shelves'] = {}
            return data
        except Exception:
            data = default_library(self.run_id)
            self.save(data)
            return data

    def save(self, data: Dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data['last_update'] = now_sec()
        tmp = self.path.with_suffix(self.path.suffix + '.tmp')
        tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False))
        tmp.replace(self.path)

    def reset_current_run(self) -> Dict[str, Any]:
        data = self.load()
        data['relocalization'] = default_library(self.run_id)['relocalization']
        for shelf in data.get('shelves', {}).values():
            shelf['current_map_pose'] = {'valid': False, 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'run_id': self.run_id}
        self.save(data)
        return data


def ensure_shelf(data: Dict[str, Any], shelf_id: str, rows: int = 3, cols: int = 3) -> Dict[str, Any]:
    shelves = data.setdefault('shelves', {})
    if shelf_id not in shelves:
        shelves[shelf_id] = {
            'shelf_id': shelf_id,
            'semantic_pose': pose_dict(),
            'current_map_pose': {'valid': False, 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'run_id': ''},
            'size': {'width': 1.2, 'depth': 0.35, 'height': 1.5},
            'grid': {'rows': int(rows), 'cols': int(cols)},
            'faces': {
                'front': {'seen_count': 0, 'qr_ids': []},
                'side_left': {'seen_count': 0, 'qr_ids': []},
                'side_right': {'seen_count': 0, 'qr_ids': []},
                'unknown': {'seen_count': 0, 'qr_ids': []},
            },
            'cells': {},
            'observations': [],
            'confidence': 0.0,
            'status': 'active',
            'missing_count': 0,
            'last_seen': 0.0,
        }
    return shelves[shelf_id]


def update_cell(shelf: Dict[str, Any], row: int, col: int, package_code: str = '', confidence: float = 0.0):
    if row <= 0 or col <= 0:
        return
    key = f'L{row}-C{col}'
    cells = shelf.setdefault('cells', {})
    cell = cells.setdefault(key, {'row': row, 'col': col, 'package_code': '', 'seen': False, 'last_seen': 0.0, 'confidence': 0.0})
    cell['row'] = int(row)
    cell['col'] = int(col)
    if package_code:
        cell['package_code'] = package_code
    cell['seen'] = True
    cell['last_seen'] = now_sec()
    cell['confidence'] = max(float(confidence), float(cell.get('confidence', 0.0)))
