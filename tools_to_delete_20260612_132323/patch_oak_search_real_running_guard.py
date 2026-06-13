#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import re
import shutil
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

OAK_YAML_FILES = [
    Path("/host-home/src/target_detection/oak_package_search/config/oak_package_search.yaml"),
    Path("/host-home/install/oak_package_search/share/oak_package_search/config/oak_package_search.yaml"),
]


def backup(p: Path) -> Path:
    b = p.with_suffix(p.suffix + f".bak_oak_search_guard_{STAMP}")
    shutil.copy2(p, b)
    return b


def write_if_changed(p: Path, s: str, label: str) -> bool:
    old = p.read_text(errors="ignore")
    if old == s:
        print(f"[NO CHANGE] {label}: {p}")
        return False
    b = backup(p)
    p.write_text(s)
    print(f"[OK] {label}: {p}")
    print(f"     backup: {b}")
    return True


def patch_code_file(p: Path):
    if not p.exists():
        print(f"[SKIP] code not found: {p}")
        return

    s = p.read_text(errors="ignore")
    old = s

    # 1. Declare search-stage guard parameters.
    if "search_already_ok_deg" not in s:
        pat = r"(\s*self\.declare_parameter\([\"']max_wz[\"'],\s*[-0-9.]+\)\s*)"
        repl = r"""\1
        # Search-stage coarse alignment guard.
        # This is for finding/scanning package codes, not final pickup alignment.
        # If OAK is already roughly parallel/facing the shelf, do not rotate again.
        self.declare_parameter("search_already_ok_deg", 8.0)
        self.declare_parameter("search_start_align_deg", 12.0)
        self.declare_parameter("same_shelf_skip_sec", 12.0)
        self.declare_parameter("search_align_only", True)
"""
        s, n = re.subn(pat, repl, s, count=1)
        if n == 0:
            print(f"[WARN] cannot insert search parameters after max_wz in {p}")

    # 2. Make alignment less aggressive.
    s = re.sub(
        r"self\.declare_parameter\([\"']align_yaw_tolerance_deg[\"'],\s*[-0-9.]+\)",
        'self.declare_parameter("align_yaw_tolerance_deg", 5.0)',
        s,
    )
    s = re.sub(
        r"self\.declare_parameter\([\"']kp_wz[\"'],\s*[-0-9.]+\)",
        'self.declare_parameter("kp_wz", 0.45)',
        s,
    )
    s = re.sub(
        r"self\.declare_parameter\([\"']min_wz[\"'],\s*[-0-9.]+\)",
        'self.declare_parameter("min_wz", 0.03)',
        s,
    )
    s = re.sub(
        r"self\.declare_parameter\([\"']max_wz[\"'],\s*[-0-9.]+\)",
        'self.declare_parameter("max_wz", 0.10)',
        s,
    )

    # 3. Initialize last aligned cache.
    if "self.last_aligned_key" not in s:
        marker = "        self.approach_yaw"
        idx = s.find(marker)
        if idx >= 0:
            line_end = s.find("\n", idx)
            insert = """

        # Re-entry guard: avoid repeatedly aligning the same shelf/layer.
        self.last_aligned_key = ""
        self.last_aligned_time = 0.0
"""
            s = s[:line_end + 1] + insert + s[line_end + 1:]
        else:
            print(f"[WARN] cannot insert last_aligned state in {p}")

    # 4. Add helper methods before cancel().
    if "def maybe_skip_search_align" not in s:
        helper = r'''    def current_request_key(self) -> str:
        shelf = str(getattr(self, "shelf_id", "") or "")
        layer = str(getattr(self, "layer_id", "") or "")
        pickup = str(getattr(self, "pickup_code", "") or "")
        if shelf or layer:
            return f"{shelf}:{layer}"
        return pickup

    def remember_aligned_now(self) -> None:
        key = self.current_request_key()
        if key:
            self.last_aligned_key = key
            self.last_aligned_time = self.now_sec()

    def publish_aligned_skip(self, reason: str, extra=None) -> None:
        self.publish_stop()
        self.remember_aligned_now()
        data = {
            "skip_align": True,
            "reason": reason,
            "last_aligned_key": getattr(self, "last_aligned_key", ""),
            "same_shelf_skip_sec": float(self.get_parameter("same_shelf_skip_sec").value),
        }
        if isinstance(extra, dict):
            data.update(extra)
        self.publish_status("aligned", reason, data)
        self.set_state(self.IDLE, reason)
        self.get_logger().info(f"search alignment skipped: {reason}")

    def maybe_skip_search_align(self):
        """Return (skip, reason, extra).

        Search alignment is only a coarse relation check before scanning package codes.
        Small corrections are intentionally skipped to avoid turning a usable pose into
        a worse pose.
        """
        now = self.now_sec()
        key = self.current_request_key()

        last_key = str(getattr(self, "last_aligned_key", "") or "")
        last_time = float(getattr(self, "last_aligned_time", 0.0) or 0.0)
        cooldown = float(self.get_parameter("same_shelf_skip_sec").value)

        if key and key == last_key and 0.0 <= now - last_time <= cooldown:
            return True, "same shelf/layer aligned recently; skip repeated search align", {
                "age_sec": now - last_time,
                "key": key,
            }

        target_yaw = getattr(self, "target_oak_yaw", None)
        if target_yaw is None:
            return False, "", {}

        ok, current_yaw, err_msg = self.get_oak_yaw_from_tf()
        if not ok:
            return False, "", {"tf_error": err_msg}

        yaw_error = normalize_angle(float(target_yaw) - float(current_yaw))
        yaw_error_deg = abs(math.degrees(yaw_error))

        already_ok = float(self.get_parameter("search_already_ok_deg").value)
        start_align = float(self.get_parameter("search_start_align_deg").value)

        if yaw_error_deg <= already_ok:
            return True, "OAK already good enough for package search; skip align", {
                "current_oak_yaw": current_yaw,
                "target_oak_yaw": target_yaw,
                "yaw_error": yaw_error,
                "yaw_error_deg": yaw_error_deg,
                "search_already_ok_deg": already_ok,
            }

        # Deadband: do not chase small/medium visual-TF jitter.
        if yaw_error_deg < start_align:
            return True, "OAK yaw error is inside search deadband; skip small correction", {
                "current_oak_yaw": current_yaw,
                "target_oak_yaw": target_yaw,
                "yaw_error": yaw_error,
                "yaw_error_deg": yaw_error_deg,
                "search_start_align_deg": start_align,
            }

        return False, "", {
            "current_oak_yaw": current_yaw,
            "target_oak_yaw": target_yaw,
            "yaw_error": yaw_error,
            "yaw_error_deg": yaw_error_deg,
            "search_start_align_deg": start_align,
        }

'''
        m = re.search(r"    def cancel\(self,\s*reason:\s*str\)\s*->\s*None:\n", s)
        if not m:
            m = re.search(r"    def datamatrix_cb\(self,", s)
        if m:
            s = s[:m.start()] + helper + s[m.start():]
        else:
            print(f"[WARN] cannot insert helper methods in {p}")

    # 5. Make publish_status remember normal aligned completions too.
    if "if status == \"aligned\":" not in s and "if status == 'aligned':" not in s:
        pat = r"(    def publish_status\(self,\s*status:\s*str,\s*message:\s*str,\s*extra:\s*Optional\[Dict\[str,\s*Any\]\]\s*=\s*None\)\s*->\s*None:\n)"
        repl = r"""\1        if status == "aligned":
            try:
                self.remember_aligned_now()
            except Exception:
                pass
"""
        s, n = re.subn(pat, repl, s, count=1)
        if n == 0:
            print(f"[WARN] cannot patch publish_status aligned memory in {p}")

    # 6. Insert request-time early exit before accepted/status.
    if "maybe_skip_search_align()" not in s:
        accepted_patterns = [
            "        self.publish_status('accepted', 'request accepted')",
            '        self.publish_status("accepted", "request accepted")',
        ]
        inserted = False
        guard = """        skip_align, skip_reason, skip_extra = self.maybe_skip_search_align()
        if skip_align:
            self.publish_aligned_skip(skip_reason, skip_extra)
            return

"""
        for pat in accepted_patterns:
            if pat in s:
                s = s.replace(pat, guard + pat, 1)
                inserted = True
                break
        if not inserted:
            print(f"[WARN] cannot insert request-time skip guard in {p}")

    # 7. If aligned in ALIGN state and search_align_only is true, stop right there.
    # This prevents moving to scan_start yaw after coarse alignment when BT only needs "aligned".
    if "search_align_only" in s and "search_align_only coarse mode completed" not in s:
        patterns = [
            """            if aligned:
                self.prepare_scan()
                self.set_state(self.MOVE_TO_SCAN_START, 'OAK aligned to shelf front')
            return""",
            """            if aligned:
                self.prepare_scan()
                self.set_state(self.MOVE_TO_SCAN_START, "OAK aligned to shelf front")
            return""",
        ]
        repl = """            if aligned:
                if bool(self.get_parameter("search_align_only").value):
                    self.publish_aligned_skip(
                        "search_align_only coarse mode completed",
                        {"current_oak_yaw": current_yaw, "yaw_error": yaw_error},
                    )
                    return
                self.prepare_scan()
                self.set_state(self.MOVE_TO_SCAN_START, "OAK aligned to shelf front")
            return"""
        done = False
        for pat in patterns:
            if pat in s:
                s = s.replace(pat, repl, 1)
                done = True
                break
        if not done:
            print(f"[INFO] ALIGN completion pattern not found or newer align_only branch already exists in {p}")

    write_if_changed(p, s, "patched oak_package_search_node.py")


def patch_yaml_file(p: Path):
    if not p.exists():
        print(f"[SKIP] yaml not found: {p}")
        return

    s = p.read_text(errors="ignore")
    old = s

    # General replacements.
    s = re.sub(r"align_yaw_tolerance_deg:\s*[-0-9.]+", "align_yaw_tolerance_deg: 5.0", s)
    s = re.sub(r"kp_wz:\s*[-0-9.]+", "kp_wz: 0.45", s)
    s = re.sub(r"min_wz:\s*[-0-9.]+", "min_wz: 0.03", s)
    s = re.sub(r"max_wz:\s*[-0-9.]+", "max_wz: 0.10", s)

    # Add missing guard params at end of the oak node parameter section.
    missing = []
    if "search_already_ok_deg:" not in s:
        missing.append("    search_already_ok_deg: 8.0")
    if "search_start_align_deg:" not in s:
        missing.append("    search_start_align_deg: 12.0")
    if "same_shelf_skip_sec:" not in s:
        missing.append("    same_shelf_skip_sec: 12.0")
    if "search_align_only:" not in s:
        missing.append("    search_align_only: true")

    if missing:
        s = s.rstrip() + "\n" + "\n".join(missing) + "\n"

    write_if_changed(p, s, "patched oak yaml")


def patch_bringup_modes(p: Path):
    if not p.exists():
        print(f"[SKIP] bringup_modes not found: {p}")
        return

    s = p.read_text(errors="ignore")
    old = s

    # Patch inline overrides that become /tmp/launch_params_xxx and override the YAML.
    s = re.sub(r"(\n\s+align_yaw_tolerance_deg:\s*)[-0-9.]+", r"\g<1>5.0", s)
    s = re.sub(r"(\n\s+kp_wz:\s*)[-0-9.]+", r"\g<1>0.45", s)
    s = re.sub(r"(\n\s+min_wz:\s*)[-0-9.]+", r"\g<1>0.03", s)
    s = re.sub(r"(\n\s+max_wz:\s*)[-0-9.]+", r"\g<1>0.10", s)

    # Add guard keys inside oak_package_search parameter override block.
    if "search_already_ok_deg:" not in s:
        marker = re.search(r"(\n\s+max_wz:\s*0\.10\s*)", s)
        if marker:
            insert = (
                marker.group(1)
                + "      search_already_ok_deg: 8.0\n"
                + "      search_start_align_deg: 12.0\n"
                + "      same_shelf_skip_sec: 12.0\n"
                + "      search_align_only: true\n"
            )
            s = s[:marker.start()] + insert + s[marker.end():]
        else:
            print(f"[WARN] cannot find max_wz marker in bringup_modes: {p}")

    write_if_changed(p, s, "patched bringup_modes.yaml")


for f in CODE_FILES:
    patch_code_file(f)

for f in OAK_YAML_FILES:
    patch_yaml_file(f)

for f in BRINGUP_FILES:
    patch_bringup_modes(f)

print("")
print("[DONE] real-running OAK search guard patch finished.")
print("Next: restart bringup, then check ROS parameters.")
