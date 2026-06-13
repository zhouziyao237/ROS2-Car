#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time
import shutil
import re

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
XML_DIR = Path("/host-home/src/Navigation/robot_logistics_task/behavior_tree")


def backup(p: Path, tag: str):
    b = p.with_suffix(p.suffix + f".bak_{tag}_{STAMP}")
    shutil.copy2(p, b)
    print(f"[BACKUP] {p} -> {b}")


def remove_stop_before_oak_xml():
    print("\n[STEP 1] Remove StopBeforeOakAlign from XML")
    if not XML_DIR.exists():
        print(f"[WARN] XML dir not found: {XML_DIR}")
        return

    for p in sorted(XML_DIR.glob("*.xml")):
        s = p.read_text(errors="ignore")
        old = s

        s = re.sub(r'\s*<StopBeforeOakAlign\s*/>\s*', '\n      ', s)
        s = re.sub(r'\s*<!--[^>]*StopBeforeOakAlign[^>]*-->\s*', '\n      ', s)

        if s != old:
            backup(p, "remove_StopBeforeOakAlign")
            p.write_text(s)
            print(f"[OK] removed StopBeforeOakAlign from {p}")
        else:
            print(f"[SKIP] no StopBeforeOakAlign in {p}")


def patch_cpp():
    print("\n[STEP 2] Patch mission_btcpp_node.cpp")
    if not CPP.exists():
        raise SystemExit(f"[ERROR] not found: {CPP}")

    backup(CPP, "clear_goal_stop")
    s = CPP.read_text(errors="ignore")
    changed = False

    # include Empty
    if '#include "std_msgs/msg/empty.hpp"' not in s:
        if '#include "std_msgs/msg/string.hpp"' in s:
            s = s.replace(
                '#include "std_msgs/msg/string.hpp"',
                '#include "std_msgs/msg/string.hpp"\n#include "std_msgs/msg/empty.hpp"',
                1
            )
            changed = True
            print("[OK] added include std_msgs/msg/empty.hpp")
        else:
            print("[WARN] cannot find std_msgs/msg/string.hpp include")

    # SharedContext publisher field
    if "clear_goal_pub" not in s:
        old = "control_mode_pub;rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;"
        new = "control_mode_pub;rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_goal_pub;rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;"
        if old in s:
            s = s.replace(old, new, 1)
            changed = True
            print("[OK] added clear_goal_pub field")
        else:
            print("[WARN] cannot find publisher field insertion point")
    else:
        print("[SKIP] clear_goal_pub already exists")

    # declare parameter
    if 'declare_parameter<std::string>("clear_goal_topic"' not in s:
        old = 'declare_parameter<std::string>("control_mode_topic","/control_mode");'
        new = 'declare_parameter<std::string>("control_mode_topic","/control_mode");declare_parameter<std::string>("clear_goal_topic","/clear_goal");'
        if old in s:
            s = s.replace(old, new, 1)
            changed = True
            print("[OK] declared clear_goal_topic parameter")
        else:
            print("[WARN] cannot find control_mode_topic declaration")
    else:
        print("[SKIP] clear_goal_topic already declared")

    # create publisher
    if "create_publisher<std_msgs::msg::Empty>" not in s:
        old = 'ctx_->control_mode_pub=create_publisher<std_msgs::msg::String>(get_parameter("control_mode_topic").as_string(),10);'
        new = old + 'ctx_->clear_goal_pub=create_publisher<std_msgs::msg::Empty>(get_parameter("clear_goal_topic").as_string(),10);'
        if old in s:
            s = s.replace(old, new, 1)
            changed = True
            print("[OK] created clear_goal publisher")
        else:
            print("[WARN] cannot find control_mode_pub create point")
    else:
        print("[SKIP] clear_goal publisher already created")

    # helper
    if "static void publish_clear_goal" not in s:
        helper = r'''
static void publish_clear_goal(std::shared_ptr<SharedContext> ctx){
    if(!ctx || !ctx->clear_goal_pub)return;
    std_msgs::msg::Empty msg;
    ctx->clear_goal_pub->publish(msg);
}
'''
        marker = "static double clamp_abs_min"
        if marker in s:
            s = s.replace(marker, helper + "\n" + marker, 1)
            changed = True
            print("[OK] added publish_clear_goal helper")
        else:
            print("[WARN] cannot find helper insertion point")
    else:
        print("[SKIP] publish_clear_goal already exists")

    # Patch WaitNavSettle.
    # Current old version after previous patch should be StatefulActionNode with message "stop and settle after navigation before OAK align".
    if "cancel Nav2 goal, stop and settle before OAK align" not in s:
        old = '''        std_msgs::msg::String mode;
        mode.data = "stop";
        ctx->control_mode_pub->publish(mode);
        publish_stop(ctx);

        ctx->status("RUNNING","stop and settle after navigation before OAK align");'''
        new = '''        publish_clear_goal(ctx);

        std_msgs::msg::String mode;
        mode.data = "stop";
        ctx->control_mode_pub->publish(mode);
        publish_stop(ctx);

        ctx->status("RUNNING","cancel Nav2 goal, stop and settle before OAK align");'''
        if old in s:
            s = s.replace(old, new, 1)
            changed = True
            print("[OK] WaitNavSettle now publishes /clear_goal before stop")
        else:
            print("[WARN] WaitNavSettle exact block not found; check manually")
    else:
        print("[SKIP] WaitNavSettle already clears Nav2 goal")

    # Patch WaitNavArrive timeout.
    if "cancel Nav2 goal and stop cmd_vel output" not in s:
        old = '''            std_msgs::msg::String mode;
            mode.data = "stop";
            ctx->control_mode_pub->publish(mode);
            publish_stop(ctx);

            ctx->status("FAILURE","WaitNavArrive timeout before reaching goal; stop cmd_vel output");'''
        new = '''            publish_clear_goal(ctx);

            std_msgs::msg::String mode;
            mode.data = "stop";
            ctx->control_mode_pub->publish(mode);
            publish_stop(ctx);

            ctx->status("FAILURE","WaitNavArrive timeout before reaching goal; cancel Nav2 goal and stop cmd_vel output");'''
        if old in s:
            s = s.replace(old, new, 1)
            changed = True
            print("[OK] WaitNavArrive timeout now clears goal and stops")
        else:
            print("[WARN] WaitNavArrive timeout block not found; check manually")
    else:
        print("[SKIP] WaitNavArrive timeout already clears goal")

    # Patch global BT failure branch.
    if "behavior tree returned FAILURE; cancel Nav2 goal and stop" not in s:
        old = '''else if(st==BT::NodeStatus::FAILURE){ctx_->status("FAILED","behavior tree returned FAILURE");{std::lock_guard<std::mutex>l(ctx_->mutex);ctx_->task=TaskData();ctx_->resolved_pose=Pose2D();}tree_->haltTree();}'''
        new = '''else if(st==BT::NodeStatus::FAILURE){
        publish_clear_goal(ctx_);
        std_msgs::msg::String mode;
        mode.data = "stop";
        ctx_->control_mode_pub->publish(mode);
        publish_stop(ctx_);

        ctx_->status("FAILED","behavior tree returned FAILURE; cancel Nav2 goal and stop");
        {std::lock_guard<std::mutex>l(ctx_->mutex);ctx_->task=TaskData();ctx_->resolved_pose=Pose2D();}
        tree_->haltTree();
    }'''
        if old in s:
            s = s.replace(old, new, 1)
            changed = True
            print("[OK] global BT FAILURE now clears goal and stops")
        else:
            print("[WARN] global BT FAILURE exact block not found; check manually")
    else:
        print("[SKIP] global BT FAILURE already patched")

    if changed:
        CPP.write_text(s)
        print(f"[OK] wrote {CPP}")
    else:
        print("[INFO] no C++ changes written")


def main():
    remove_stop_before_oak_xml()
    patch_cpp()
    print("\n[DONE] patch finished")
    print("Next:")
    print("  colcon build --packages-select robot_logistics_task")
    print("  source install/setup.bash")
    print("  restart launch")


if __name__ == "__main__":
    main()
