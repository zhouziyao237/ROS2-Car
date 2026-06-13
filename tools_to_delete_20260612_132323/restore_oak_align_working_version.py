#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")

ROOTS = [
    Path("/host-home/src"),
    Path("/host-home/install"),
]

BT_ROOTS = [
    Path("/host-home/src/Navigation/robot_logistics_task"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task"),
]


def read_text(p: Path) -> str:
    try:
        return p.read_text(errors="ignore")
    except Exception:
        return ""


def backup(p: Path, tag: str):
    if not p.exists():
        return
    bak = p.with_suffix(p.suffix + f".bak_{tag}_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")


def score_cpp(p: Path):
    s = read_text(p)

    if "mission_btcpp_node" not in p.name:
        return -9999

    score = 0

    # 必须恢复这些类
    if "class WaitNavDoneBeforeOak" in s:
        score += 300
    if "registerNodeType<WaitNavDoneBeforeOak" in s:
        score += 300
    if "class RequestOakPackageSearchAlign" in s:
        score += 200
    if "class ScanTargetPackageOnShelf" in s:
        score += 100
    if "class VerifyShelfQR" in s:
        score += 80
    if "class SendPlatformTarget" in s:
        score += 50

    # 关键：RequestOakPackageSearchAlign 需要能带 shelf_front_yaw
    if "shelf_front_yaw" in s:
        score += 250

    # 避免选择明显被改坏的版本
    if "target_oak_yaw is None" in s:
        score -= 100
    if "removed stale WaitNavDoneBeforeOak" in s:
        score -= 500
    if "asymmetric_scan_class" in p.name:
        score -= 300
    if "safe_left_right_offsets" in p.name:
        score -= 100

    return score


def restore_best_cpp():
    candidates = []

    for root in ROOTS:
        if not root.exists():
            continue

        for p in root.rglob("mission_btcpp_node.cpp*"):
            if not p.is_file():
                continue

            sc = score_cpp(p)
            if sc >= 700:
                candidates.append((sc, p.stat().st_mtime, p))

    if not candidates:
        print("[ERROR] 找不到可恢复的 mission_btcpp_node.cpp 备份。")
        print("[INFO] 当前找到的 mission_btcpp_node.cpp 文件如下：")
        for root in ROOTS:
            if root.exists():
                for p in root.rglob("mission_btcpp_node.cpp*"):
                    if p.is_file():
                        print(" ", p, "score=", score_cpp(p))
        raise SystemExit(2)

    candidates.sort(reverse=True)
    score, mtime, best = candidates[0]

    print(f"[SELECT_CPP] score={score} file={best}")

    backup(CPP, "before_restore_oak_align")
    shutil.copy2(best, CPP)
    print(f"[OK] restored C++: {CPP}")

    s = read_text(CPP)

    required = [
        "class WaitNavDoneBeforeOak",
        "registerNodeType<WaitNavDoneBeforeOak",
        "class RequestOakPackageSearchAlign",
        "class ScanTargetPackageOnShelf",
        "shelf_front_yaw",
    ]

    for r in required:
        if r not in s:
            print(f"[ERROR] restored cpp missing: {r}")
            raise SystemExit(3)

    print("[OK] C++ contains WaitNavDoneBeforeOak + RequestOakPackageSearchAlign + shelf_front_yaw")


def patch_bt_text(s: str) -> str:
    old = s

    if "RequestOakPackageSearchAlign" not in s:
        return s

    # 保证 WaitNavDoneBeforeOak 在 OAK 对正前面
    if "WaitNavDoneBeforeOak" not in s:
        patterns = [
            r"(<\s*WaitNavArrive\s*/\s*>)",
            r"(<\s*WaitShelfNavigationArrive\s*/\s*>)",
        ]

        inserted = False

        for pat in patterns:
            m = re.search(pat, s)
            if m:
                s = s[:m.end()] + "\n        <WaitNavDoneBeforeOak/>" + s[m.end():]
                inserted = True
                break

        if not inserted:
            # 找不到 WaitNavArrive，就插到 RequestOakPackageSearchAlign 前
            s = re.sub(
                r"(<\s*RequestOakPackageSearchAlign\s*/\s*>)",
                r"<WaitNavDoneBeforeOak/>\n        \1",
                s,
                count=1,
            )

    return s


def patch_bt_files():
    changed = 0

    suffixes = {".yaml", ".yml", ".xml"}

    for root in BT_ROOTS:
        if not root.exists():
            continue

        for p in root.rglob("*"):
            if not p.is_file():
                continue

            if p.suffix not in suffixes:
                continue

            if ".bak" in p.name:
                continue

            s = read_text(p)

            if "RequestOakPackageSearchAlign" not in s:
                continue

            ns = patch_bt_text(s)

            if ns != s:
                backup(p, "restore_waitnavdone_before_oak")
                p.write_text(ns)
                print(f"[OK] patched BT/YAML: {p}")
                changed += 1

    print(f"[DONE] patched BT/YAML count={changed}")


def show_check():
    print("")
    print("[CHECK] C++:")
    for key in [
        "class WaitNavDoneBeforeOak",
        "registerNodeType<WaitNavDoneBeforeOak",
        "class RequestOakPackageSearchAlign",
        "shelf_front_yaw",
    ]:
        print(f"  {key}: {'YES' if key in read_text(CPP) else 'NO'}")

    print("")
    print("[CHECK] active BT/YAML containing RequestOakPackageSearchAlign:")
    for root in BT_ROOTS:
        if not root.exists():
            continue
        for p in root.rglob("*"):
            if not p.is_file() or p.suffix not in {".yaml", ".yml", ".xml"}:
                continue
            if ".bak" in p.name:
                continue
            s = read_text(p)
            if "RequestOakPackageSearchAlign" in s:
                has_wait = "WaitNavDoneBeforeOak" in s
                print(f"  {p} | WaitNavDoneBeforeOak={has_wait}")


def main():
    restore_best_cpp()
    patch_bt_files()
    show_check()

    print("")
    print("[DONE] OAK align working version restored.")
    print("Next: build robot_logistics_task and robot2_bringup.")


if __name__ == "__main__":
    main()
