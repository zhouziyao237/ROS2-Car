#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import time
import shutil
import re

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
stamp = time.strftime("%Y%m%d_%H%M%S")

if not CPP.exists():
    raise SystemExit(f"[ERROR] not found: {CPP}")

bak = CPP.with_suffix(CPP.suffix + f".bak_real_wait_nav_{stamp}")
shutil.copy2(CPP, bak)
print(f"[BACKUP] {CPP} -> {bak}")

s = CPP.read_text(errors="ignore")

old = r'class WaitNavArrive:public TimedAction{public:WaitNavArrive(const std::string&n,const BT::NodeConfiguration&c):TimedAction(n,c,"WaitNavArrive",2.0){}SIMPLE_PORTS};'

new = r'''
class WaitNavArrive:public BT::StatefulActionNode{
    double start_=0.0;
    double stable_start_=0.0;
public:
    WaitNavArrive(const std::string&n,const BT::NodeConfiguration&c):BT::StatefulActionNode(n,c){}
    SIMPLE_PORTS

    BT::NodeStatus onStart()override{
        auto ctx=get_ctx(*this);
        ctx->set_state("WaitNavArrive");
        start_=steady_seconds();
        stable_start_=0.0;
        ctx->status("RUNNING","waiting real nav arrival by TF distance to goal");
        ctx->trace(name(),"RUNNING");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()override{
        auto ctx=get_ctx(*this);

        Pose2D goal;
        {
            std::lock_guard<std::mutex> l(ctx->mutex);
            goal = ctx->resolved_pose;
        }

        if(!goal.valid){
            ctx->status("FAILURE","WaitNavArrive has no resolved goal pose");
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }

        if(steady_seconds() - start_ > 40.0){
            ctx->status("FAILURE","WaitNavArrive timeout before reaching goal");
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }

        try{
            auto tf = ctx->tf_buffer->lookupTransform(
                ctx->map_frame,
                ctx->base_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(0.20)
            );

            double bx = tf.transform.translation.x;
            double by = tf.transform.translation.y;
            double dist = std::hypot(goal.x - bx, goal.y - by);

            std::stringstream ss;
            ss << "WaitNavArrive dist=" << dist
               << " goal=(" << goal.x << "," << goal.y << ")"
               << " base=(" << bx << "," << by << ")";

            if(dist < 0.18){
                if(stable_start_ <= 0.0){
                    stable_start_ = steady_seconds();
                }

                if(steady_seconds() - stable_start_ >= 1.0){
                    ctx->status("SUCCESS", ss.str());
                    ctx->trace(name(),"SUCCESS");
                    return BT::NodeStatus::SUCCESS;
                }
            }else{
                stable_start_ = 0.0;
            }

            ctx->status("RUNNING", ss.str());
            return BT::NodeStatus::RUNNING;
        }catch(const std::exception& e){
            ctx->status("RUNNING", std::string("WaitNavArrive TF waiting: ") + e.what());
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted()override{
        get_ctx(*this)->trace(name(),"HALTED");
    }
};
'''

if old not in s:
    raise SystemExit("[ERROR] cannot find old WaitNavArrive class. It may already be patched or code format changed.")

s = s.replace(old, new, 1)
CPP.write_text(s)
print("[OK] WaitNavArrive replaced with real TF-distance wait")
