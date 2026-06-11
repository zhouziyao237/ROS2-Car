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

bak = CPP.with_suffix(CPP.suffix + f".bak_waitnav_stop_settle_{STAMP}")
shutil.copy2(CPP, bak)
print(f"[BACKUP] {CPP} -> {bak}")

s = CPP.read_text(errors="ignore")
changed = False

# 1) WaitNavArrive timeout 前强制 stop
old_timeout = '''
        if(steady_seconds() - start_ > 40.0){
            ctx->status("FAILURE","WaitNavArrive timeout before reaching goal");
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }
'''

new_timeout = '''
        if(steady_seconds() - start_ > 40.0){
            std_msgs::msg::String mode;
            mode.data = "stop";
            ctx->control_mode_pub->publish(mode);
            publish_stop(ctx);

            ctx->status("FAILURE","WaitNavArrive timeout before reaching goal; stop cmd_vel output");
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }
'''

if old_timeout in s:
    s = s.replace(old_timeout, new_timeout, 1)
    changed = True
    print("[OK] patched WaitNavArrive timeout to stop output")
else:
    print("[WARN] WaitNavArrive timeout block not found, skip timeout patch")

# 2) 把 WaitNavSettle 从 TimedAction 改成真正 stop-settle 节点
old_settle = r'class WaitNavSettle:public TimedAction{public:WaitNavSettle(const std::string&n,const BT::NodeConfiguration&c):TimedAction(n,c,"WaitNavSettle",3.0){}SIMPLE_PORTS};'

new_settle = r'''
class WaitNavSettle:public BT::StatefulActionNode{
    double start_=0.0;
public:
    WaitNavSettle(const std::string&n,const BT::NodeConfiguration&c):BT::StatefulActionNode(n,c){}
    SIMPLE_PORTS

    BT::NodeStatus onStart()override{
        auto ctx=get_ctx(*this);
        ctx->set_state("WaitNavSettle");
        start_=steady_seconds();

        std_msgs::msg::String mode;
        mode.data = "stop";
        ctx->control_mode_pub->publish(mode);
        publish_stop(ctx);

        ctx->status("RUNNING","stop and settle after navigation before OAK align");
        ctx->trace(name(),"RUNNING");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()override{
        auto ctx=get_ctx(*this);

        publish_stop(ctx);

        if(steady_seconds() - start_ >= 1.5){
            std_msgs::msg::String mode;
            mode.data = "auto";
            ctx->control_mode_pub->publish(mode);

            ctx->status("SUCCESS","nav output stopped and settled; switch back to auto");
            ctx->trace(name(),"SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted()override{
        auto ctx=get_ctx(*this);
        publish_stop(ctx);
        ctx->trace(name(),"HALTED");
    }
};
'''

if old_settle in s:
    s = s.replace(old_settle, new_settle, 1)
    changed = True
    print("[OK] replaced WaitNavSettle with stop-settle version")
elif "stop and settle after navigation before OAK align" in s:
    print("[SKIP] WaitNavSettle already patched")
else:
    print("[ERROR] old WaitNavSettle class not found. Check current file manually.")

if changed:
    CPP.write_text(s)
    print("[DONE] wrote patched C++")
else:
    print("[DONE] no changes written")
