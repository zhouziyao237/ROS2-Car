#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import re
import sys

STAMP = time.strftime("%Y%m%d_%H%M%S")

P = Path("/host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")
MARK = "AUTO_RESET_CONTROL_MODE_BEFORE_PICKUP_TASK_20260611_SIMPLE"

def backup(path: Path):
    bak = path.with_suffix(path.suffix + f".bak_auto_reset_{STAMP}")
    shutil.copy2(path, bak)
    print(f"[BAK] {path} -> {bak}")
    return bak

def main():
    if not P.exists():
        print(f"[ERROR] missing: {P}")
        sys.exit(1)

    s = P.read_text(errors="ignore")

    if MARK in s:
        print("[NO CHANGE] already patched")
        return

    old = s

    # 1) 确保有 String import
    if "from std_msgs.msg import String" not in s:
        print("[ERROR] cannot find: from std_msgs.msg import String")
        print("先执行：grep -n \"std_msgs\\|String\" /host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")
        sys.exit(1)

    # 2) 在 __init__ 里面插入 control_mode_pub
    # 找到任意 self.xxx = self.create_publisher(String, ..., 10) 后面追加
    m = re.search(
        r'(?m)^(\s*)self\.\w+\s*=\s*self\.create_publisher\(\s*String\s*,\s*[\'"][^\'"]+[\'"]\s*,\s*\d+\s*\)\s*$',
        s
    )

    if not m:
        print("[ERROR] cannot find any self.create_publisher(String, ..., N)")
        print("请发这个输出：")
        print("grep -n \"create_publisher\" /host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")
        sys.exit(1)

    indent = m.group(1)
    insert_pub = (
        f"\n{indent}# {MARK}\n"
        f"{indent}self.control_mode_pub = self.create_publisher(String, '/control_mode', 10)\n"
    )

    s = s[:m.end()] + insert_pub + s[m.end():]

    # 3) 找发布 task 的 publish 行
    # 优先找 publish(msg)，其次找任何 .publish(...) 且附近包含 shelf_goal / task
    candidates = list(re.finditer(r'(?m)^(\s*)self\.\w+\.publish\(([^)]*)\)\s*$', s))

    if not candidates:
        print("[ERROR] cannot find self.xxx.publish(...) line")
        print("请发这个输出：")
        print("grep -n \"publish\" /host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")
        sys.exit(1)

    chosen = None

    # 优先选附近有 /task/shelf_goal 或 已发布取件任务 的 publish
    for c in candidates:
        window = s[max(0, c.start()-800):min(len(s), c.end()+800)]
        if "/task/shelf_goal" in window or "已发布取件任务" in window or "shelf_goal" in window:
            chosen = c
            break

    # 再优先选参数是 msg 的 publish
    if chosen is None:
        for c in candidates:
            if c.group(2).strip() == "msg":
                chosen = c
                break

    if chosen is None:
        chosen = candidates[0]

    pub_indent = chosen.group(1)

    auto_code = (
        f"{pub_indent}# {MARK}\n"
        f"{pub_indent}try:\n"
        f"{pub_indent}    auto_msg = String()\n"
        f"{pub_indent}    auto_msg.data = 'auto'\n"
        f"{pub_indent}    self.control_mode_pub.publish(auto_msg)\n"
        f"{pub_indent}except Exception:\n"
        f"{pub_indent}    pass\n"
    )

    s = s[:chosen.start()] + auto_code + s[chosen.start():]

    if s == old:
        print("[NO CHANGE] no modification made")
        return

    backup(P)
    P.write_text(s)

    print("[OK] patched pickup_code_input.py")
    print("[OK] every new pickup task will publish /control_mode='auto' before /task/shelf_goal")

if __name__ == "__main__":
    main()
