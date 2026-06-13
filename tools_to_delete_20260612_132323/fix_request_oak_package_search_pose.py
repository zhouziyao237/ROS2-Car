#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import re
import sys

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")


NEW_CLASS = r'''
class RequestOakPackageSearchAlign:public BT::StatefulActionNode{
    double start_=0;
public:
    RequestOakPackageSearchAlign(const std::string&n,const BT::NodeConfiguration&c):BT::StatefulActionNode(n,c){}
    SIMPLE_PORTS

    BT::NodeStatus onStart()override{
        start_=steady_seconds();
        auto ctx=get_ctx(*this);
        ctx->set_state("RequestOakPackageSearchAlign");

        // Before handing control to oak_package_search_node, publish zero once.
        if(ctx->cmd_vel_pub){
            geometry_msgs::msg::Twist stop;
            ctx->cmd_vel_pub->publish(stop);
        }

        std_msgs::msg::String msg;
        {
            std::lock_guard<std::mutex>l(ctx->mutex);

            if(!ctx->task.active){
                ctx->status("FAILURE","no active task");
                ctx->trace(name(),"FAILURE");
                return BT::NodeStatus::FAILURE;
            }

            if(!ctx->resolved_pose.valid){
                ctx->status("FAILURE","resolved shelf pose invalid; cannot send OAK package search request");
                ctx->trace(name(),"FAILURE");
                return BT::NodeStatus::FAILURE;
            }

            ctx->latest_package_search_status.clear();
            ctx->latest_package_search_json.clear();

            // 关键：OAK package_search 必须拿到 shelf_front_yaw 或 shelf point。
            // 这里直接使用 QueryShelfPose 得到的 resolved_pose：
            // x/y = 货架/预停点语义位姿
            // yaw = 货架正面 yaw
            // shelf_front_yaw = yaw
            std::stringstream ss;
            ss<<"{";
            ss<<"\"task_type\":\"package_search\"";
            ss<<",\"pickup_code\":\""<<ctx->task.cell_id<<"\"";
            ss<<",\"shelf_id\":\""<<ctx->task.shelf_id<<"\"";
            ss<<",\"layer_id\":\"";
            if(ctx->task.row>0) ss<<"L"<<ctx->task.row;
            ss<<"\"";
            ss<<",\"package_code\":\""<<ctx->task.col<<"\"";
            ss<<",\"x\":"<<ctx->resolved_pose.x;
            ss<<",\"y\":"<<ctx->resolved_pose.y;
            ss<<",\"yaw\":"<<ctx->resolved_pose.yaw;
            ss<<",\"shelf_front_yaw\":"<<ctx->resolved_pose.yaw;
            ss<<",\"source\":\"mission_btcpp_node\"";
            ss<<"}";

            msg.data=ss.str();
            RCLCPP_INFO(ctx->node->get_logger(),"[RequestOakPackageSearchAlign] request=%s",msg.data.c_str());
        }

        if(!ctx->package_search_request_pub){
            ctx->status("FAILURE","package_search_request_pub is null");
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }

        ctx->package_search_request_pub->publish(msg);
        ctx->status("RUNNING","published /target_detection/package_search/request with shelf_front_yaw; waiting for aligned/found");
        ctx->trace(name(),"RUNNING");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()override{
        auto ctx=get_ctx(*this);

        std::string st,json;
        {
            std::lock_guard<std::mutex>l(ctx->mutex);
            st=ctx->latest_package_search_status;
            json=ctx->latest_package_search_json;
        }

        if(st=="aligned"||st=="found"||json.find("\"status\":\"aligned\"")!=std::string::npos||json.find("\"status\":\"found\"")!=std::string::npos){
            ctx->status("SUCCESS","OAK package search alignment completed: "+st);
            ctx->trace(name(),"SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }

        if(st=="failed"||st=="timeout"||json.find("\"status\":\"failed\"")!=std::string::npos||json.find("\"status\":\"timeout\"")!=std::string::npos){
            ctx->status("FAILURE","OAK package search failed: "+json);
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }

        if(steady_seconds()-start_ > ctx->package_search_timeout_sec){
            ctx->status("FAILURE","OAK package search align timeout");
            ctx->trace(name(),"TIMEOUT");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted()override{
        auto ctx=get_ctx(*this);
        if(ctx->cmd_vel_pub){
            geometry_msgs::msg::Twist stop;
            ctx->cmd_vel_pub->publish(stop);
        }
        ctx->trace(name(),"HALTED");
    }
};
'''


def backup(path: Path):
    bak = path.with_suffix(path.suffix + f".bak_fix_oak_request_pose_{STAMP}")
    shutil.copy2(path, bak)
    print(f"[BAK] {path} -> {bak}")
    return bak


def find_class_bounds(s: str, class_name: str):
    start = s.find("class " + class_name)
    if start < 0:
        return None

    # 找到 class 起始后的第一个 {
    brace = s.find("{", start)
    if brace < 0:
        return None

    depth = 0
    i = brace
    while i < len(s):
        if s[i] == "{":
            depth += 1
        elif s[i] == "}":
            depth -= 1
            if depth == 0:
                # class 结束应该是 };
                semi = s.find(";", i)
                if semi < 0:
                    return None
                return start, semi + 1
        i += 1

    return None


def main():
    if not CPP.exists():
        print(f"[ERROR] not found: {CPP}")
        sys.exit(1)

    s = CPP.read_text(errors="ignore")

    bounds = find_class_bounds(s, "RequestOakPackageSearchAlign")
    if not bounds:
        print("[ERROR] cannot find class RequestOakPackageSearchAlign in mission_btcpp_node.cpp")
        sys.exit(1)

    start, end = bounds

    backup(CPP)

    s2 = s[:start] + NEW_CLASS.strip() + s[end:]
    CPP.write_text(s2)

    print("[OK] patched RequestOakPackageSearchAlign")
    print("[OK] request will include x/y/yaw/shelf_front_yaw")
    print("[NEXT] build robot_logistics_task")


if __name__ == "__main__":
    main()
