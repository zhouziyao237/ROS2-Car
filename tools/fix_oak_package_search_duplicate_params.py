#!/usr/bin/env python3
from pathlib import Path
import time
import shutil
import re

NODE = Path("/host-home/src/target_detection/oak_package_search/oak_package_search/oak_package_search_node.py")
STAMP = time.strftime("%Y%m%d_%H%M%S")

PARAMS = [
    "align_only",
    "yaw_control_sign",
    "yaw_deadband_deg",
    "yaw_settle_count_required",
    "stop_publish_repeat",
    "ki_wz",
    "kd_wz",
    "pid_integral_limit",
]

DEFAULT_LINES = {
    "align_only": "        self.declare_parameter('align_only', False)",
    "yaw_control_sign": "        self.declare_parameter('yaw_control_sign', 1.0)",
    "yaw_deadband_deg": "        self.declare_parameter('yaw_deadband_deg', 2.0)",
    "yaw_settle_count_required": "        self.declare_parameter('yaw_settle_count_required', 3)",
    "stop_publish_repeat": "        self.declare_parameter('stop_publish_repeat', 5)",
}


def backup(p: Path):
    bak = p.with_suffix(p.suffix + f".bak_dedupe_params_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BACKUP] {p} -> {bak}")


def main():
    if not NODE.exists():
        raise SystemExit(f"[ERROR] not found: {NODE}")

    s = NODE.read_text(errors="ignore")
    backup(NODE)

    lines = s.splitlines()
    seen = set()
    out = []
    removed = []

    # 1. Remove duplicate declare_parameter lines for our added parameters.
    for line in lines:
        m = re.search(r"self\.declare_parameter\(['\"]([^'\"]+)['\"]", line)
        if m:
            name = m.group(1)
            if name in PARAMS:
                if name in seen:
                    removed.append(line)
                    continue
                seen.add(name)
        out.append(line)

    s = "\n".join(out) + "\n"

    if removed:
        print("[OK] removed duplicate parameter declarations:")
        for x in removed:
            print("  " + x.strip())
    else:
        print("[INFO] no duplicate declarations removed")

    # 2. Ensure core align-only parameters exist once.
    missing = [p for p in DEFAULT_LINES if p not in seen]

    if missing:
        print(f"[INFO] missing parameters: {missing}")

        # Insert near controller params if possible.
        insert_text = "\n        # Alignment-only / yaw deadband parameters.\n"
        insert_text += "\n".join(DEFAULT_LINES[p] for p in missing)
        insert_text += "\n"

        # Prefer after max_wz declaration.
        m = re.search(r"^        self\.declare_parameter\(['\"]max_wz['\"].*$", s, flags=re.M)
        if m:
            pos = m.end()
            s = s[:pos] + insert_text + s[pos:]
            print("[OK] inserted missing declarations after max_wz")
        else:
            # Fallback: before tick_hz.
            idx = s.find("tick_hz =")
            if idx < 0:
                idx = s.find("self.create_timer")
            if idx < 0:
                raise SystemExit("[ERROR] cannot find insertion point for missing parameters")

            s = s[:idx] + insert_text + s[idx:]
            print("[OK] inserted missing declarations before timer setup")
    else:
        print("[OK] all core align-only parameters already exist")

    # 3. Remove duplicate self._yaw_settle_count assignment, keep first.
    lines = s.splitlines()
    out = []
    settle_seen = False
    settle_removed = 0

    for line in lines:
        if "self._yaw_settle_count" in line and "=" in line and line.strip().startswith("self._yaw_settle_count"):
            if settle_seen:
                settle_removed += 1
                continue
            settle_seen = True
        out.append(line)

    s = "\n".join(out) + "\n"

    if settle_removed:
        print(f"[OK] removed duplicate _yaw_settle_count assignments: {settle_removed}")

    NODE.write_text(s)
    print(f"[DONE] fixed duplicate params in {NODE}")

    print("")
    print("Check remaining declarations:")
    print(f"  grep -n \"declare_parameter('yaw_control_sign\\|declare_parameter(\\\"yaw_control_sign\" {NODE}")
    print(f"  grep -n \"declare_parameter('align_only\\|declare_parameter(\\\"align_only\" {NODE}")


if __name__ == "__main__":
    main()
