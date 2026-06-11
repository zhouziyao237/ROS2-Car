#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import subprocess
import re
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

CODE_FILES = [
    Path("/host-home/src/target_detection/oak_package_search/oak_package_search/oak_package_search_node.py"),
    Path("/host-home/install/oak_package_search/lib/python3.10/site-packages/oak_package_search/oak_package_search_node.py"),
]

BRINGUP_FILES = [
    Path("/host-home/src/Navigation/robot2_bringup/config/bringup_modes.yaml"),
    Path("/host-home/install/robot2_bringup/share/robot2_bringup/config/bringup_modes.yaml"),
]

YAML_FILES = [
    Path("/host-home/src/target_detection/oak_package_search/config/oak_package_search.yaml"),
    Path("/host-home/install/oak_package_search/share/oak_package_search/config/oak_package_search.yaml"),
]


def compiles(p: Path) -> bool:
    r = subprocess.run(
        ["python3", "-m", "py_compile", str(p)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return r.returncode == 0


def restore_clean_code(p: Path):
    if not p.exists():
        raise SystemExit(f"[ERROR] code file not found: {p}")

    current_backup = p.with_suffix(p.suffix + f".broken_before_restore_{STAMP}")
    shutil.copy2(p, current_backup)
    print(f"[BACKUP BROKEN] {p} -> {current_backup}")

    candidates = []
    candidates += sorted(p.parent.glob(p.name + ".bak_oak_search_guard_*"))
    candidates += sorted(p.parent.glob(p.name + ".bak_monkey_guard_*"))

    good = []
    for c in candidates:
        txt = c.read_text(errors="ignore")
        if "OAK SEARCH REENTRY GUARD MONKEY PATCH" in txt:
            continue
        if not compiles(c):
            continue
        good.append(c)

    if not good:
        raise SystemExit(
            f"[ERROR] no clean compiling backup found for {p}\n"
            f"        Run: ls -lh {p}.bak_*"
        )

    # 优先用最早的 oak_search_guard 备份，一般就是第一次修改前的干净原文件
    chosen = good[0]
    shutil.copy2(chosen, p)
    print(f"[RESTORE] {p} <- {chosen}")


def find_node_class_name(s: str) -> str:
    init_pos = s.find('super().__init__("oak_package_search_node")')
    if init_pos < 0:
        init_pos = s.find("super().__init__('oak_package_search_node')")
    if init_pos < 0:
        raise RuntimeError("cannot find oak_package_search_node init")

    matches = list(re.finditer(r"^class\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(", s, re.M))
    last = None
    for m in matches:
        if m.start() < init_pos:
            last = m
        else:
            break

    if last is None:
        raise RuntimeError("cannot locate node class")

    return last.group(1)


def patch_code(p: Path):
    s = p.read_text(errors="ignore")

    if "OAK SEARCH REENTRY GUARD MONKEY PATCH" in s:
        print(f"[SKIP] monkey patch already exists: {p}")
        return

    cls = find_node_class_name(s)
    print(f"[INFO] detected class {cls} in {p}")

    monkey = f'''

# --- OAK SEARCH REENTRY GUARD MONKEY PATCH START ---
# Coarse package-search alignment guard.
# Purpose: prevent repeated small OAK rotations when the current pose is already
# good enough for package code scanning.

import math as _oak_guard_math
import time as _oak_guard_time

def _oak_guard_norm_angle(a):
    while a > _oak_guard_math.pi:
        a -= 2.0 * _oak_guard_math.pi
    while a < -_oak_guard_math.pi:
        a += 2.0 * _oak_guard_math.pi
    return a

def _oak_guard_now(node):
    try:
        return float(node.now_sec())
    except Exception:
        return _oak_guard_time.time()

def _oak_guard_declare(node, name, default):
    try:
        node.get_parameter(name)
    except Exception:
        try:
            node.declare_parameter(name, default)
        except Exception:
            pass

def _oak_guard_float(node, name, default):
    try:
        return float(node.get_parameter(name).value)
    except Exception:
        return float(default)

def _oak_guard_key(node):
    shelf = str(getattr(node, "shelf_id", "") or "")
    layer = str(getattr(node, "layer_id", "") or "")
    pickup = str(getattr(node, "pickup_code", "") or "")
    if shelf or layer:
        return f"{{shelf}}:{{layer}}"
    return pickup

def _oak_guard_remember(node):
    key = _oak_guard_key(node)
    if key:
        node.last_aligned_key = key
        node.last_aligned_time = _oak_guard_now(node)

def _oak_guard_maybe_skip(node):
    now = _oak_guard_now(node)
    key = _oak_guard_key(node)

    last_key = str(getattr(node, "last_aligned_key", "") or "")
    last_time = float(getattr(node, "last_aligned_time", 0.0) or 0.0)
    cooldown = _oak_guard_float(node, "same_shelf_skip_sec", 12.0)

    if key and key == last_key and 0.0 <= now - last_time <= cooldown:
        return True, "same shelf/layer aligned recently; skip repeated search align", {{
            "key": key,
            "age_sec": now - last_time,
            "same_shelf_skip_sec": cooldown,
        }}

    target_yaw = getattr(node, "target_oak_yaw", None)
    if target_yaw is None:
        return False, "", {{}}

    if not hasattr(node, "get_oak_yaw_from_tf"):
        return False, "", {{}}

    ok, current_yaw, err_msg = node.get_oak_yaw_from_tf()
    if not ok:
        return False, "", {{"tf_error": str(err_msg)}}

    yaw_error = _oak_guard_norm_angle(float(target_yaw) - float(current_yaw))
    yaw_error_deg = abs(_oak_guard_math.degrees(yaw_error))

    already_ok = _oak_guard_float(node, "search_already_ok_deg", 8.0)
    start_align = _oak_guard_float(node, "search_start_align_deg", 12.0)

    if yaw_error_deg <= already_ok:
        return True, "OAK already good enough for package search; skip align", {{
            "current_oak_yaw": current_yaw,
            "target_oak_yaw": target_yaw,
            "yaw_error": yaw_error,
            "yaw_error_deg": yaw_error_deg,
            "search_already_ok_deg": already_ok,
        }}

    if yaw_error_deg < start_align:
        return True, "OAK yaw error is inside search deadband; skip small correction", {{
            "current_oak_yaw": current_yaw,
            "target_oak_yaw": target_yaw,
            "yaw_error": yaw_error,
            "yaw_error_deg": yaw_error_deg,
            "search_start_align_deg": start_align,
        }}

    return False, "", {{
        "current_oak_yaw": current_yaw,
        "target_oak_yaw": target_yaw,
        "yaw_error": yaw_error,
        "yaw_error_deg": yaw_error_deg,
        "search_start_align_deg": start_align,
    }}

_oak_guard_original_init = {cls}.__init__
_oak_guard_original_set_state = {cls}.set_state
_oak_guard_original_publish_status = {cls}.publish_status

def _oak_guard_init(self, *args, **kwargs):
    _oak_guard_original_init(self, *args, **kwargs)

    _oak_guard_declare(self, "search_already_ok_deg", 8.0)
    _oak_guard_declare(self, "search_start_align_deg", 12.0)
    _oak_guard_declare(self, "same_shelf_skip_sec", 12.0)
    _oak_guard_declare(self, "search_align_only", True)

    if not hasattr(self, "last_aligned_key"):
        self.last_aligned_key = ""
    if not hasattr(self, "last_aligned_time"):
        self.last_aligned_time = 0.0

    try:
        self.get_logger().info(
            "OAK search guard enabled: "
            f"already_ok={{self.get_parameter('search_already_ok_deg').value}}, "
            f"start_align={{self.get_parameter('search_start_align_deg').value}}, "
            f"same_shelf_skip={{self.get_parameter('same_shelf_skip_sec').value}}, "
            f"search_align_only={{self.get_parameter('search_align_only').value}}, "
            f"max_wz={{self.get_parameter('max_wz').value}}"
        )
    except Exception:
        pass

def _oak_guard_publish_status(self, status, message, extra=None):
    if str(status) == "aligned":
        try:
            _oak_guard_remember(self)
        except Exception:
            pass

    try:
        return _oak_guard_original_publish_status(self, status, message, extra)
    except TypeError:
        return _oak_guard_original_publish_status(self, status, message)

def _oak_guard_publish_skip(self, reason, extra=None):
    try:
        self.publish_stop()
    except Exception:
        pass

    _oak_guard_remember(self)

    data = {{
        "skip_align": True,
        "reason": reason,
        "last_aligned_key": str(getattr(self, "last_aligned_key", "")),
    }}
    if isinstance(extra, dict):
        data.update(extra)

    try:
        self.publish_status("aligned", reason, data)
    except Exception as e:
        try:
            self.get_logger().warn(f"publish aligned skip failed: {{e}}")
        except Exception:
            pass

    try:
        _oak_guard_original_set_state(self, self.IDLE, reason)
    except Exception:
        try:
            self.state = self.IDLE
        except Exception:
            pass

    try:
        self.get_logger().info(f"search alignment skipped: {{reason}}, data={{data}}")
    except Exception:
        pass

def _oak_guard_set_state(self, state, reason=""):
    try:
        if state == self.ALIGN:
            skip, skip_reason, skip_extra = _oak_guard_maybe_skip(self)
            if skip:
                _oak_guard_publish_skip(self, skip_reason, skip_extra)
                return
    except Exception as e:
        try:
            self.get_logger().warn(f"OAK search guard check failed, continue normal align: {{e}}")
        except Exception:
            pass

    return _oak_guard_original_set_state(self, state, reason)

{cls}.__init__ = _oak_guard_init
{cls}.set_state = _oak_guard_set_state
{cls}.publish_status = _oak_guard_publish_status

# --- OAK SEARCH REENTRY GUARD MONKEY PATCH END ---
'''

    m = re.search(r"^def\s+main\s*\(", s, re.M)
    if not m:
        raise SystemExit(f"[ERROR] cannot find def main() in {p}")

    s = s[:m.start()] + monkey + "\n" + s[m.start():]

    b = p.with_suffix(p.suffix + f".bak_clean_monkey_patch_{STAMP}")
    shutil.copy2(p, b)
    p.write_text(s)
    print(f"[PATCH] {p}")
    print(f"        backup: {b}")


def patch_yaml(p: Path):
    if not p.exists():
        print(f"[SKIP] yaml not found: {p}")
        return

    s = p.read_text(errors="ignore")

    s = re.sub(r"align_yaw_tolerance_deg:\s*[-0-9.]+", "align_yaw_tolerance_deg: 5.0", s)
    s = re.sub(r"kp_wz:\s*[-0-9.]+", "kp_wz: 0.45", s)
    s = re.sub(r"min_wz:\s*[-0-9.]+", "min_wz: 0.03", s)
    s = re.sub(r"max_wz:\s*[-0-9.]+", "max_wz: 0.10", s)

    additions = []
    if "search_already_ok_deg:" not in s:
        additions.append("    search_already_ok_deg: 8.0")
    if "search_start_align_deg:" not in s:
        additions.append("    search_start_align_deg: 12.0")
    if "same_shelf_skip_sec:" not in s:
        additions.append("    same_shelf_skip_sec: 12.0")
    if "search_align_only:" not in s:
        additions.append("    search_align_only: true")

    if additions:
        s = s.rstrip() + "\n" + "\n".join(additions) + "\n"

    b = p.with_suffix(p.suffix + f".bak_clean_guard_yaml_{STAMP}")
    shutil.copy2(p, b)
    p.write_text(s)
    print(f"[PATCH YAML] {p}")
    print(f"             backup: {b}")


def patch_bringup(p: Path):
    if not p.exists():
        print(f"[SKIP] bringup not found: {p}")
        return

    s = p.read_text(errors="ignore")

    s = re.sub(r"(\n\s+align_yaw_tolerance_deg:\s*)[-0-9.]+", r"\g<1>5.0", s)
    s = re.sub(r"(\n\s+kp_wz:\s*)[-0-9.]+", r"\g<1>0.45", s)
    s = re.sub(r"(\n\s+min_wz:\s*)[-0-9.]+", r"\g<1>0.03", s)
    s = re.sub(r"(\n\s+max_wz:\s*)[-0-9.]+", r"\g<1>0.10", s)

    if "search_already_ok_deg:" not in s:
        m = re.search(r"(\n\s+max_wz:\s*0\.10\s*)", s)
        if m:
            insert = (
                m.group(1)
                + "      search_already_ok_deg: 8.0\n"
                + "      search_start_align_deg: 12.0\n"
                + "      same_shelf_skip_sec: 12.0\n"
                + "      search_align_only: true\n"
            )
            s = s[:m.start()] + insert + s[m.end():]
        else:
            print(f"[WARN] cannot insert guard params after max_wz in {p}")

    b = p.with_suffix(p.suffix + f".bak_clean_guard_bringup_{STAMP}")
    shutil.copy2(p, b)
    p.write_text(s)
    print(f"[PATCH BRINGUP] {p}")
    print(f"               backup: {b}")


for p in CODE_FILES:
    restore_clean_code(p)
    patch_code(p)

for p in YAML_FILES:
    patch_yaml(p)

for p in BRINGUP_FILES:
    patch_bringup(p)

print("")
print("[DONE] restored clean code and applied safe monkey patch.")
print("Now run py_compile.")
