#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import re
import sys
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")
MARK = "WAIT_NAV_DONE_BEFORE_OAK_ALIGN_20260611"

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")

XML_FILES = [
    Path("/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_full_groot1.xml"),
    Path("/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_full_groot2.xml"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/behavior_tree/pickup_task_full_groot1.xml"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/behavior_tree/pickup_task_full_groot2.xml"),
]

YAML_FILES = [
    Path("/host-home/src/Navigation/robot_logistics_task/config/mission_task.yaml"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/config/mission_task.yaml"),
]

NAV_SETTLE_PARAMS = {
    "nav_settle_quiet_sec": 1.0,
    "nav_settle_timeout_sec": 8.0,
    "nav_settle_speed_eps": 0.04,
}


def backup(path: Path, tag: str):
    bak = path.with_suffix(path.suffix + f".bak_{tag}_{STAMP}")
    shutil.copy2(path, bak)
    print(f"[BAK] {path} -> {bak}")
    return bak


def write_checked(path: Path, text: str, tag: str):
    backup(path, tag)
    path.write_text(text)
    print(f"[OK] patched {path}")


def patch_cpp():
    if not CPP.exists():
        print(f"[ERROR] missing: {CPP}")
        sys.exit(1)

    s = CPP.read_text(errors="ignore")

    if MARK in s:
        print(f"[NO CHANGE] {CPP} already patched")
        return False

    old = s

    # ------------------------------------------------------------
    # 1) SharedContext 里增加 Nav2 速度监测字段
    # ------------------------------------------------------------
    if "latest_nav_cmd_stamp" not in s:
        target = "double latest_datamatrix_stamp=0.0;"
        insert = (
            "double latest_datamatrix_stamp=0.0;"
            f"/* {MARK}: nav velocity settle fields */"
            "double latest_nav_cmd_stamp=0.0, latest_nav_cmd_mag=0.0;"
            "double latest_nav_smoothed_stamp=0.0, latest_nav_smoothed_mag=0.0;"
            "double latest_cmd_vel_watch_stamp=0.0, latest_cmd_vel_watch_mag=0.0;"
            "double nav_settle_quiet_sec=1.0, nav_settle_timeout_sec=8.0, nav_settle_speed_eps=0.04;"
        )

        if target not in s:
            print("[ERROR] cannot find SharedContext latest_datamatrix_stamp field")
            print("请先执行：")
            print("grep -n \"latest_datamatrix_stamp\\|struct SharedContext\" /host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
            sys.exit(1)

        s = s.replace(target, insert, 1)

    # ------------------------------------------------------------
    # 2) MissionNode 成员变量里增加 Twist 订阅器
    # ------------------------------------------------------------
    if "nav_cmd_sub_" not in s:
        # 常见当前写法：goal_sub_,cancel_sub_,...,datamatrix_sub_;
        m = re.search(
            r"(rclcpp::Subscription<std_msgs::msg::String>::SharedPtr\s+[^;]*datamatrix_sub_;)",
            s,
            flags=re.S,
        )
        if m:
            repl = (
                m.group(1)
                + "rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr "
                  "nav_cmd_sub_, nav_smoothed_sub_, cmd_vel_watch_sub_;"
            )
            s = s[:m.start()] + repl + s[m.end():]
        else:
            m = re.search(
                r"(rclcpp::Subscription<std_msgs::msg::String>::SharedPtr\s+[^;]*platform_status_sub_;)",
                s,
                flags=re.S,
            )
            if not m:
                print("[ERROR] cannot find MissionNode subscription member line")
                print("请先执行：")
                print("grep -n \"Subscription<.*SharedPtr\" /host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
                sys.exit(1)

            repl = (
                m.group(1)
                + "rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr "
                  "nav_cmd_sub_, nav_smoothed_sub_, cmd_vel_watch_sub_;"
            )
            s = s[:m.start()] + repl + s[m.end():]

    # ------------------------------------------------------------
    # 3) declare_parameter 增加 nav settle 参数
    # ------------------------------------------------------------
    if 'declare_parameter<double>("nav_settle_quiet_sec"' not in s:
        anchor = 'ctx_=std::make_shared<SharedContext>();'
        insert = (
            f'/* {MARK}: parameters */'
            'declare_parameter<double>("nav_settle_quiet_sec",1.0);'
            'declare_parameter<double>("nav_settle_timeout_sec",8.0);'
            'declare_parameter<double>("nav_settle_speed_eps",0.04);'
        )

        if anchor not in s:
            print("[ERROR] cannot find ctx_=std::make_shared<SharedContext>();")
            sys.exit(1)

        s = s.replace(anchor, insert + anchor, 1)

    # ------------------------------------------------------------
    # 4) ctx 参数赋值
    # ------------------------------------------------------------
    if "ctx_->nav_settle_quiet_sec=get_parameter" not in s:
        anchor = 'ctx_->allow_demo_anchor_if_missing=get_parameter("allow_demo_anchor_if_missing").as_bool();'
        insert = (
            anchor
            + f'/* {MARK}: parameter copy */'
            'ctx_->nav_settle_quiet_sec=get_parameter("nav_settle_quiet_sec").as_double();'
            'ctx_->nav_settle_timeout_sec=get_parameter("nav_settle_timeout_sec").as_double();'
            'ctx_->nav_settle_speed_eps=get_parameter("nav_settle_speed_eps").as_double();'
        )

        if anchor not in s:
            print("[ERROR] cannot find allow_demo_anchor_if_missing assignment")
            print("请先执行：")
            print("grep -n \"allow_demo_anchor_if_missing\" /host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
            sys.exit(1)

        s = s.replace(anchor, insert, 1)

    # ------------------------------------------------------------
    # 5) 订阅 /cmd_vel_nav /cmd_vel_smoothed /cmd_vel
    #    这些是用来判断 Nav2 是否还在发速度
    # ------------------------------------------------------------
    if "nav_cmd_sub_=create_subscription<geometry_msgs::msg::Twist>" not in s:
        sub_code = (
            f'/* {MARK}: watch Nav2 velocity topics before OAK align */'
            'auto update_nav_settle_cmd=[this](const geometry_msgs::msg::Twist::SharedPtr msg,int src){'
            'double mag=std::abs(msg->linear.x)+std::abs(msg->linear.y)+std::abs(msg->angular.z);'
            'double now=steady_seconds();'
            'std::lock_guard<std::mutex> l(ctx_->mutex);'
            'if(src==0){ctx_->latest_nav_cmd_stamp=now;ctx_->latest_nav_cmd_mag=mag;}'
            'else if(src==1){ctx_->latest_nav_smoothed_stamp=now;ctx_->latest_nav_smoothed_mag=mag;}'
            'else{ctx_->latest_cmd_vel_watch_stamp=now;ctx_->latest_cmd_vel_watch_mag=mag;}'
            '};'
            'nav_cmd_sub_=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav",10,'
            '[update_nav_settle_cmd](geometry_msgs::msg::Twist::SharedPtr msg){update_nav_settle_cmd(msg,0);});'
            'nav_smoothed_sub_=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_smoothed",10,'
            '[update_nav_settle_cmd](geometry_msgs::msg::Twist::SharedPtr msg){update_nav_settle_cmd(msg,1);});'
            'cmd_vel_watch_sub_=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,'
            '[update_nav_settle_cmd](geometry_msgs::msg::Twist::SharedPtr msg){update_nav_settle_cmd(msg,2);});'
        )

        # 优先插在 datamatrix_sub_ 后，找不到就插在 factory register 前
        anchor = 'factory_.registerNodeType<ReceiveShelfGoal>("ReceiveShelfGoal");'
        if anchor not in s:
            print("[ERROR] cannot find factory register anchor")
            print("请先执行：")
            print("grep -n \"registerNodeType<ReceiveShelfGoal\" /host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
            sys.exit(1)

        s = s.replace(anchor, sub_code + anchor, 1)

    # ------------------------------------------------------------
    # 6) 新增 BT 节点 WaitNavDoneBeforeOak
    # ------------------------------------------------------------
    if "class WaitNavDoneBeforeOak" not in s:
        class_code = r'''
// WAIT_NAV_DONE_BEFORE_OAK_ALIGN_20260611
class WaitNavDoneBeforeOak:public BT::StatefulActionNode{
    double start_=0.0;
    double quiet_since_=0.0;
    double last_report_=0.0;

public:
    WaitNavDoneBeforeOak(const std::string&n,const BT::NodeConfiguration&c):BT::StatefulActionNode(n,c){}
    SIMPLE_PORTS

    BT::NodeStatus onStart()override{
        auto ctx=get_ctx(*this);
        start_=steady_seconds();
        quiet_since_=0.0;
        last_report_=0.0;
        ctx->set_state("WaitNavDoneBeforeOak");
        ctx->status("RUNNING","waiting Nav2 cmd_vel to finish before OAK align");
        ctx->trace(name(),"RUNNING");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()override{
        auto ctx=get_ctx(*this);
        double now=steady_seconds();

        double nav_age=999.0, smooth_age=999.0, cmd_age=999.0;
        double nav_mag=0.0, smooth_mag=0.0, cmd_mag=0.0;
        double quiet_sec=1.0, timeout_sec=8.0, eps=0.04;

        {
            std::lock_guard<std::mutex> l(ctx->mutex);
            quiet_sec=ctx->nav_settle_quiet_sec;
            timeout_sec=ctx->nav_settle_timeout_sec;
            eps=ctx->nav_settle_speed_eps;

            if(ctx->latest_nav_cmd_stamp>0.0) nav_age=now-ctx->latest_nav_cmd_stamp;
            if(ctx->latest_nav_smoothed_stamp>0.0) smooth_age=now-ctx->latest_nav_smoothed_stamp;
            if(ctx->latest_cmd_vel_watch_stamp>0.0) cmd_age=now-ctx->latest_cmd_vel_watch_stamp;

            nav_mag=ctx->latest_nav_cmd_mag;
            smooth_mag=ctx->latest_nav_smoothed_mag;
            cmd_mag=ctx->latest_cmd_vel_watch_mag;
        }

        bool nav_active=(nav_age<0.45 && nav_mag>eps);
        bool smooth_active=(smooth_age<0.45 && smooth_mag>eps);
        bool cmd_active=(cmd_age<0.45 && cmd_mag>eps);
        bool active=nav_active||smooth_active||cmd_active;

        if(active){
            quiet_since_=0.0;
            if(now-last_report_>0.5){
                last_report_=now;
                std::stringstream ss;
                ss<<"Nav2 still active before OAK: "
                  <<"nav_mag="<<nav_mag<<" age="<<nav_age
                  <<", smooth_mag="<<smooth_mag<<" age="<<smooth_age
                  <<", cmd_mag="<<cmd_mag<<" age="<<cmd_age;
                ctx->status("RUNNING",ss.str());
            }

            if(now-start_>timeout_sec){
                publish_stop(ctx);
                ctx->status("SUCCESS","Nav2 settle timeout; force stop then continue OAK align");
                ctx->trace(name(),"SUCCESS_TIMEOUT");
                return BT::NodeStatus::SUCCESS;
            }

            return BT::NodeStatus::RUNNING;
        }

        if(quiet_since_<=0.0){
            quiet_since_=now;
            ctx->status("RUNNING","Nav2 cmd_vel quiet, confirming settle");
            return BT::NodeStatus::RUNNING;
        }

        if(now-quiet_since_>=quiet_sec){
            publish_stop(ctx);
            ctx->status("SUCCESS","Nav2 cmd_vel settled; OAK may align now");
            ctx->trace(name(),"SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }

        if(now-start_>timeout_sec){
            publish_stop(ctx);
            ctx->status("SUCCESS","Nav2 settle timeout after quiet check; continue OAK align");
            ctx->trace(name(),"SUCCESS_TIMEOUT");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted()override{
        get_ctx(*this)->trace(name(),"HALTED");
    }
};
'''

        anchor = "class SearchShelfQR"
        if anchor not in s:
            print("[ERROR] cannot find class SearchShelfQR anchor")
            print("请先执行：")
            print("grep -n \"class SearchShelfQR\\|class WaitNavArrive\" /host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
            sys.exit(1)

        s = s.replace(anchor, class_code + "\n" + anchor, 1)

    # ------------------------------------------------------------
    # 7) 注册 BT 节点
    # ------------------------------------------------------------
    if 'registerNodeType<WaitNavDoneBeforeOak>("WaitNavDoneBeforeOak")' not in s:
        anchor = 'factory_.registerNodeType<WaitNavArrive>("WaitNavArrive");'
        insert = (
            anchor
            + f'/* {MARK}: register node */'
            'factory_.registerNodeType<WaitNavDoneBeforeOak>("WaitNavDoneBeforeOak");'
        )

        if anchor not in s:
            print("[ERROR] cannot find WaitNavArrive registration")
            print("请先执行：")
            print("grep -n \"WaitNavArrive\" /host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
            sys.exit(1)

        s = s.replace(anchor, insert, 1)

    if s == old:
        print(f"[NO CHANGE] {CPP}")
        return False

    write_checked(CPP, s, "wait_nav_done_before_oak")
    return True


def patch_xml_file(path: Path):
    if not path.exists():
        print(f"[SKIP] missing XML: {path}")
        return False

    s = path.read_text(errors="ignore")

    if "WaitNavDoneBeforeOak" in s:
        print(f"[NO CHANGE] XML already has WaitNavDoneBeforeOak: {path}")
        return False

    if "<WaitNavArrive/>" not in s:
        print(f"[SKIP] XML has no <WaitNavArrive/>: {path}")
        return False

    s2 = s.replace(
        "<WaitNavArrive/>",
        "<WaitNavArrive/>\n      <WaitNavDoneBeforeOak/>",
        1,
    )

    write_checked(path, s2, "wait_nav_done_before_oak_xml")
    return True


def patch_yaml_file(path: Path):
    if not path.exists():
        print(f"[SKIP] missing YAML: {path}")
        return False

    try:
        data = yaml.safe_load(path.read_text(errors="ignore"))
    except Exception as e:
        print(f"[SKIP] cannot parse YAML {path}: {e}")
        return False

    if data is None:
        return False

    changed = False

    def patch_obj(obj):
        nonlocal changed

        if isinstance(obj, dict):
            # 标准 ROS2 参数结构
            if "mission_btcpp_node" in obj and isinstance(obj["mission_btcpp_node"], dict):
                params = obj["mission_btcpp_node"].setdefault("ros__parameters", {})
                if isinstance(params, dict):
                    for k, v in NAV_SETTLE_PARAMS.items():
                        if params.get(k) != v:
                            params[k] = v
                            changed = True

            # 直接 ros__parameters 结构
            if "ros__parameters" in obj and isinstance(obj["ros__parameters"], dict):
                params = obj["ros__parameters"]
                if "bt_xml" in params or "tick_hz" in params or "task_status_topic" in params:
                    for k, v in NAV_SETTLE_PARAMS.items():
                        if params.get(k) != v:
                            params[k] = v
                            changed = True

            for v in obj.values():
                patch_obj(v)

        elif isinstance(obj, list):
            for item in obj:
                patch_obj(item)

    patch_obj(data)

    if not changed:
        print(f"[NO CHANGE] YAML: {path}")
        return False

    backup(path, "wait_nav_done_before_oak_yaml")
    path.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )
    )
    print(f"[OK] patched YAML params: {path}")
    return True


def main():
    count = 0

    if patch_cpp():
        count += 1

    for x in XML_FILES:
        if patch_xml_file(x):
            count += 1

    for y in YAML_FILES:
        if patch_yaml_file(y):
            count += 1

    print("")
    print("[DONE] Added WaitNavDoneBeforeOak after WaitNavArrive")
    print("[DONE] It waits until /cmd_vel_nav, /cmd_vel_smoothed and /cmd_vel are quiet")
    print("[DONE] Then OAK package search alignment can start")
    print(f"[DONE] patched items: {count}")


if __name__ == "__main__":
    main()
