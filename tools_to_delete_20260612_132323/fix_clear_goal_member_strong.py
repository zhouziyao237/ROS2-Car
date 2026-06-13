#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time
import shutil
import re

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
STAMP = time.strftime("%Y%m%d_%H%M%S")

if not CPP.exists():
    raise SystemExit(f"[ERROR] not found: {CPP}")

bak = CPP.with_suffix(CPP.suffix + f".bak_fix_clear_goal_member_strong_{STAMP}")
shutil.copy2(CPP, bak)
print(f"[BACKUP] {CPP} -> {bak}")

s = CPP.read_text(errors="ignore")

# 找 struct SharedContext 的范围
m = re.search(r"struct\s+SharedContext\s*\{", s)
if not m:
    raise SystemExit("[ERROR] cannot find struct SharedContext")

start = m.start()
brace_start = s.find("{", m.start())
if brace_start < 0:
    raise SystemExit("[ERROR] cannot find SharedContext opening brace")

# 简单括号匹配，找到 SharedContext 结束位置
depth = 0
end = None
for i in range(brace_start, len(s)):
    if s[i] == "{":
        depth += 1
    elif s[i] == "}":
        depth -= 1
        if depth == 0:
            end = i
            break

if end is None:
    raise SystemExit("[ERROR] cannot find SharedContext closing brace")

struct_text = s[start:end+1]

if "clear_goal_pub" in struct_text:
    print("[SKIP] SharedContext already has clear_goal_pub")
else:
    # 优先插在 goal_pose_pub 前面
    pat = r"(rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr\s+goal_pose_pub\s*;)"
    repl = r"rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_goal_pub;\1"
    new_struct, n = re.subn(pat, repl, struct_text, count=1)

    if n == 0:
        # 如果找不到 goal_pose_pub，就插在 struct 结束前
        insert = "rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_goal_pub;"
        new_struct = struct_text[:-1] + insert + struct_text[-1]
        print("[WARN] goal_pose_pub not found in struct, inserted before struct end")
    else:
        print("[OK] inserted clear_goal_pub before goal_pose_pub")

    s = s[:start] + new_struct + s[end+1:]

# 确保 include 存在
if '#include "std_msgs/msg/empty.hpp"' not in s:
    if '#include "std_msgs/msg/string.hpp"' in s:
        s = s.replace(
            '#include "std_msgs/msg/string.hpp"',
            '#include "std_msgs/msg/string.hpp"\n#include "std_msgs/msg/empty.hpp"',
            1
        )
        print("[OK] added empty include")
    else:
        print("[WARN] cannot find string include; empty include not added")

CPP.write_text(s)
print("[DONE] wrote patched file")
