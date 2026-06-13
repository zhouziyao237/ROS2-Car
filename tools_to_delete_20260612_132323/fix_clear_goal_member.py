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

bak = CPP.with_suffix(CPP.suffix + f".bak_fix_clear_goal_member_{STAMP}")
shutil.copy2(CPP, bak)
print(f"[BACKUP] {CPP} -> {bak}")

s = CPP.read_text(errors="ignore")

if "clear_goal_pub" in s:
    # 如果已经在 init 里用了，但 struct 没有声明，就补 struct 成员
    pass

# 只在 SharedContext 结构体内部补成员。
# 找 control_mode_pub 后面接 goal_pose_pub 的紧凑写法。
patterns = [
    (
        "control_mode_pub;rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;",
        "control_mode_pub;rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_goal_pub;rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;"
    ),
    (
        "control_mode_pub; rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;",
        "control_mode_pub; rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_goal_pub; rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;"
    ),
]

changed = False

# 先检查 struct 里是否已经有 clear_goal_pub 声明
struct_match = re.search(r"struct\s+SharedContext\s*\{.*?\};", s, re.S)
if not struct_match:
    raise SystemExit("[ERROR] cannot find struct SharedContext")

struct_text = struct_match.group(0)

if "clear_goal_pub" in struct_text:
    print("[SKIP] SharedContext already has clear_goal_pub")
else:
    patched = struct_text
    for old, new in patterns:
        if old in patched:
            patched = patched.replace(old, new, 1)
            changed = True
            print("[OK] inserted clear_goal_pub after control_mode_pub")
            break

    if not changed:
        # 更宽松：找到 control_mode_pub; 后直接插入
        patched2 = re.sub(
            r"(control_mode_pub\s*;)",
            r"\1rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_goal_pub;",
            patched,
            count=1
        )
        if patched2 != patched:
            patched = patched2
            changed = True
            print("[OK] inserted clear_goal_pub by loose regex")
        else:
            print("[ERROR] cannot insert clear_goal_pub into SharedContext")
            print("Please grep SharedContext publisher line manually:")
            print("grep -n \"control_mode_pub\\|goal_pose_pub\" /host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
            raise SystemExit(1)

    s = s[:struct_match.start()] + patched + s[struct_match.end():]

# 确保 include empty 存在
if '#include "std_msgs/msg/empty.hpp"' not in s:
    if '#include "std_msgs/msg/string.hpp"' in s:
        s = s.replace(
            '#include "std_msgs/msg/string.hpp"',
            '#include "std_msgs/msg/string.hpp"\n#include "std_msgs/msg/empty.hpp"',
            1
        )
        changed = True
        print("[OK] added empty include")
    else:
        print("[WARN] std_msgs/msg/string.hpp include not found")

if changed:
    CPP.write_text(s)
    print("[DONE] wrote patched file")
else:
    print("[DONE] no changes needed")
