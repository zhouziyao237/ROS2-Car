#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import sys

STAMP = time.strftime("%Y%m%d_%H%M%S")

P = Path("/host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")
MARK = "AUTO_RESET_CONTROL_MODE_BEFORE_PICKUP_TASK_20260611_MULTILINE"


def backup(path: Path):
    bak = path.with_suffix(path.suffix + f".bak_auto_reset_multiline_{STAMP}")
    shutil.copy2(path, bak)
    print(f"[BAK] {path} -> {bak}")
    return bak


def find_multiline_call_end(lines, start_idx):
    """
    从 self.xxx = self.create_publisher( 这一行开始，
    找到括号闭合的结束行。
    """
    depth = 0
    started = False

    for i in range(start_idx, len(lines)):
        line = lines[i]

        for ch in line:
            if ch == "(":
                depth += 1
                started = True
            elif ch == ")":
                depth -= 1

        if started and depth <= 0:
            return i

    return None


def main():
    if not P.exists():
        print(f"[ERROR] missing: {P}")
        sys.exit(1)

    text = P.read_text(errors="ignore")

    if MARK in text:
        print("[NO CHANGE] already patched")
        return

    if "from std_msgs.msg import String" not in text:
        print("[ERROR] cannot find: from std_msgs.msg import String")
        print("请先发这个输出：")
        print("grep -n \"std_msgs\\|String\" /host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")
        sys.exit(1)

    lines = text.splitlines(True)

    # 1. 找 self.goal_pub = self.create_publisher( 这个多行调用
    goal_start = None
    for i, line in enumerate(lines):
        if "self.goal_pub" in line and "self.create_publisher" in line:
            goal_start = i
            break

    if goal_start is None:
        print("[ERROR] cannot find self.goal_pub = self.create_publisher(...)")
        print("请发这个输出：")
        print("sed -n '25,55p' /host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")
        sys.exit(1)

    goal_end = find_multiline_call_end(lines, goal_start)
    if goal_end is None:
        print("[ERROR] cannot find end of self.goal_pub create_publisher block")
        sys.exit(1)

    indent = lines[goal_start].split("self.goal_pub")[0]

    insert_pub = (
        f"{indent}# {MARK}\n"
        f"{indent}self.control_mode_pub = self.create_publisher(\n"
        f"{indent}    String,\n"
        f"{indent}    '/control_mode',\n"
        f"{indent}    10\n"
        f"{indent})\n"
    )

    lines.insert(goal_end + 1, insert_pub)

    text2 = "".join(lines)
    lines = text2.splitlines(True)

    # 2. 找 self.goal_pub.publish(...)，在它前面插入 auto 发布
    publish_idx = None
    for i, line in enumerate(lines):
        if "self.goal_pub.publish" in line:
            publish_idx = i
            break

    if publish_idx is None:
        print("[ERROR] cannot find self.goal_pub.publish(...)")
        print("请发这个输出：")
        print("grep -n \"goal_pub\\|publish\" /host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")
        sys.exit(1)

    pub_indent = lines[publish_idx].split("self.goal_pub.publish")[0]

    auto_code = (
        f"{pub_indent}# {MARK}\n"
        f"{pub_indent}try:\n"
        f"{pub_indent}    auto_msg = String()\n"
        f"{pub_indent}    auto_msg.data = 'auto'\n"
        f"{pub_indent}    self.control_mode_pub.publish(auto_msg)\n"
        f"{pub_indent}except Exception:\n"
        f"{pub_indent}    pass\n"
    )

    lines.insert(publish_idx, auto_code)

    new_text = "".join(lines)

    backup(P)
    P.write_text(new_text)

    print("[OK] patched pickup_code_input.py")
    print("[OK] before every /task/shelf_goal publish, it will publish /control_mode='auto'")


if __name__ == "__main__":
    main()
