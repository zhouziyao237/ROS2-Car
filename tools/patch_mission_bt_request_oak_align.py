#!/usr/bin/env python3
from pathlib import Path
import time
import shutil
import re
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

ROOTS = [
    Path("/host-home/src/Navigation/robot_logistics_task"),
    Path("/host-home/src/robot_logistics_task"),
]

PKG = None
for r in ROOTS:
    if (r / "src/mission_btcpp_node.cpp").exists():
        PKG = r
        break

if PKG is None:
    found = list(Path("/host-home/src").rglob("robot_logistics_task/src/mission_btcpp_node.cpp"))
    if found:
        PKG = found[0].parents[1]
    else:
        raise SystemExit("[ERROR] cannot find robot_logistics_task/src/mission_btcpp_node.cpp")

CPP = PKG / "src/mission_btcpp_node.cpp"
XML = PKG / "behavior_tree/pickup_task_three_node_main.xml"
CFG = PKG / "config/mission_task.yaml"
LAUNCH = PKG / "launch/mission_task.launch.py"


def backup(p: Path):
    if p.exists():
        b = p.with_suffix(p.suffix + f".bak_oak_pkg_req_{STAMP}")
        shutil.copy2(p, b)
        print(f"[BACKUP] {p} -> {b}")


def replace_once(s: str, old: str, new: str, tag: str):
    if old not in s:
        print(f"[SKIP/NOTFOUND] {tag}")
        return s, False
    print(f"[OK] {tag}")
    return s.replace(old, new, 1), True


def patch_cpp():
    backup(CPP)
    s = CPP.read_text(errors="ignore")
    changed = False

    # ------------------------------------------------------------
    # 1. Extend SharedContext
    # ------------------------------------------------------------
    if "package_search_request_topic" not in s:
        s, ok = replace_once(
            s,
            'package_scan_direction="left_to_right";double',
            'package_scan_direction="left_to_right",package_search_request_topic="/target_detection/package_search/request",package_search_status_topic="/target_detection/package_search/status";double',
            "SharedContext package_search topic strings",
        )
        changed |= ok

    if "package_search_timeout_sec" not in s:
        s, ok = replace_once(
            s,
            "package_scan_timeout_sec=25.0;bool",
            "package_scan_timeout_sec=25.0,package_search_timeout_sec=20.0;bool",
            "SharedContext package_search timeout",
        )
        changed |= ok

    if "latest_package_search_status" not in s:
        s, ok = replace_once(
            s,
            'latest_package_json,state="IDLE"',
            'latest_package_json,latest_package_search_json,latest_package_search_status,state="IDLE"',
            "SharedContext latest_package_search_status",
        )
        changed |= ok

    if "package_search_request_pub" not in s:
        s, ok = replace_once(
            s,
            "control_mode_pub;",
            "control_mode_pub,package_search_request_pub;",
            "SharedContext package_search_request_pub",
        )
        changed |= ok

    if "package_search_status_sub_" not in s:
        s, ok = replace_once(
            s,
            "datamatrix_sub_;",
            "datamatrix_sub_,package_search_status_sub_;",
            "MissionNode package_search_status_sub_",
        )
        changed |= ok

    # ------------------------------------------------------------
    # 2. Declare ROS params
    # ------------------------------------------------------------
    non_struct_part = re.sub(r"struct SharedContext\{.*?\};", "", s, flags=re.S)

    if "package_search_request_topic" not in non_struct_part:
        old = 'declare_parameter<std::string>("datamatrix_result_json_topic","/oak/datamatrix_3d_result_json");'
        new = (
            old
            + 'declare_parameter<std::string>("package_search_request_topic","/target_detection/package_search/request");'
            + 'declare_parameter<std::string>("package_search_status_topic","/target_detection/package_search/status");'
            + 'declare_parameter<double>("package_search_timeout_sec",20.0);'
        )
        s, ok = replace_once(s, old, new, "declare package_search params")
        changed |= ok

    # ------------------------------------------------------------
    # 3. Assign params into ctx
    # ------------------------------------------------------------
    if 'ctx_->package_search_request_topic=get_parameter("package_search_request_topic")' not in s:
        old = 'ctx_->package_scan_direction=get_parameter("package_scan_direction").as_string();'
        new = (
            old
            + 'ctx_->package_search_request_topic=get_parameter("package_search_request_topic").as_string();'
            + 'ctx_->package_search_status_topic=get_parameter("package_search_status_topic").as_string();'
        )
        s, ok = replace_once(s, old, new, "assign package_search topic params")
        changed |= ok

    if 'ctx_->package_search_timeout_sec=get_parameter("package_search_timeout_sec")' not in s:
        old = 'ctx_->package_scan_timeout_sec=get_parameter("package_scan_timeout_sec").as_double();'
        new = old + 'ctx_->package_search_timeout_sec=get_parameter("package_search_timeout_sec").as_double();'
        s, ok = replace_once(s, old, new, "assign package_search timeout")
        changed |= ok

    # ------------------------------------------------------------
    # 4. Create publisher for /target_detection/package_search/request
    # ------------------------------------------------------------
    if "ctx_->package_search_request_pub=create_publisher" not in s:
        old = 'ctx_->cmd_vel_pub=create_publisher<geometry_msgs::msg::Twist>(ctx_->cmd_vel_topic,10);'
        new = old + 'ctx_->package_search_request_pub=create_publisher<std_msgs::msg::String>(ctx_->package_search_request_topic,10);'
        s, ok = replace_once(s, old, new, "create package_search_request_pub")
        changed |= ok

    # ------------------------------------------------------------
    # 5. Create subscriber for /target_detection/package_search/status
    # ------------------------------------------------------------
    if "package_search_status_sub_=create_subscription" not in s:
        old = (
            'datamatrix_sub_=create_subscription<std_msgs::msg::String>'
            '(get_parameter("datamatrix_result_json_topic").as_string(),10,'
            '[this](std_msgs::msg::String::SharedPtr m){'
            'std::lock_guard<std::mutex>l(ctx_->mutex);'
            'ctx_->latest_datamatrix_json=m->data;'
            'ctx_->latest_datamatrix_stamp=wall_seconds(*this);'
            '});factory_'
        )

        new = (
            'datamatrix_sub_=create_subscription<std_msgs::msg::String>'
            '(get_parameter("datamatrix_result_json_topic").as_string(),10,'
            '[this](std_msgs::msg::String::SharedPtr m){'
            'std::lock_guard<std::mutex>l(ctx_->mutex);'
            'ctx_->latest_datamatrix_json=m->data;'
            'ctx_->latest_datamatrix_stamp=wall_seconds(*this);'
            '});'
            'package_search_status_sub_=create_subscription<std_msgs::msg::String>'
            '(ctx_->package_search_status_topic,10,'
            '[this](std_msgs::msg::String::SharedPtr m){'
            'std::lock_guard<std::mutex>l(ctx_->mutex);'
            'ctx_->latest_package_search_json=m->data;'
            'std::string st;'
            'if(get_string_json(m->data,"status",st))ctx_->latest_package_search_status=st;'
            '});factory_'
        )

        s, ok = replace_once(s, old, new, "create package_search_status_sub")
        changed |= ok

    # ------------------------------------------------------------
    # 6. Add BT node class: RequestOakPackageSearchAlign
    # ------------------------------------------------------------
    if "class RequestOakPackageSearchAlign" not in s:
        idx = s.find("class VerifyShelfQR")
        if idx < 0:
            idx = s.find("class MissionNode")
        if idx < 0:
            raise SystemExit("[ERROR] cannot find insertion point for RequestOakPackageSearchAlign class")

        cls = r'''

class RequestOakPackageSearchAlign:public BT::StatefulActionNode{
    double start_=0.0;
public:
    RequestOakPackageSearchAlign(const std::string&n,const BT::NodeConfiguration&c):BT::StatefulActionNode(n,c){}
    SIMPLE_PORTS

    BT::NodeStatus onStart()override{
        auto ctx=get_ctx(*this);
        ctx->set_state("RequestOakPackageSearchAlign");
        start_=steady_seconds();

        std::string req;
        {
            std::lock_guard<std::mutex> l(ctx->mutex);

            if(!ctx->task.active || ctx->task.shelf_id.empty()){
                ctx->status("FAILURE","cannot request OAK package search: no active task or shelf_id");
                ctx->trace(name(),"FAILURE");
                return BT::NodeStatus::FAILURE;
            }

            ctx->latest_package_search_status.clear();
            ctx->latest_package_search_json.clear();

            const std::string pickup = ctx->task.pickup_code.empty() ? ctx->task.cell_id : ctx->task.pickup_code;

            req = "{\"task_type\":\"package_search\""
                  ",\"pickup_code\":\"" + pickup + "\""
                  ",\"shelf_id\":\"" + ctx->task.shelf_id + "\""
                  ",\"layer_id\":\"" + ctx->task.layer_id + "\""
                  ",\"package_code\":\"" + ctx->task.package_code + "\""
                  ",\"source\":\"mission_btcpp_node\"}";
        }

        if(!ctx->package_search_request_pub){
            ctx->status("FAILURE","package_search_request_pub is null");
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }

        std_msgs::msg::String msg;
        msg.data=req;
        ctx->package_search_request_pub->publish(msg);

        ctx->status("RUNNING","published /target_detection/package_search/request; waiting for aligned/found");
        ctx->trace(name(),"RUNNING");

        RCLCPP_INFO(
            ctx->node->get_logger(),
            "[RequestOakPackageSearchAlign] request=%s",
            req.c_str()
        );

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()override{
        auto ctx=get_ctx(*this);

        std::string st;
        std::string json;
        {
            std::lock_guard<std::mutex> l(ctx->mutex);
            st=ctx->latest_package_search_status;
            json=ctx->latest_package_search_json;
        }

        if(st=="aligned" || st=="found"){
            ctx->status("SUCCESS","OAK package search alignment completed: "+st);
            ctx->trace(name(),"SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }

        if(st=="failed" || st=="failure" || st=="timeout"){
            ctx->status("FAILURE","OAK package search failed: "+json);
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }

        if(steady_seconds()-start_ > ctx->package_search_timeout_sec){
            ctx->status("FAILURE","OAK package search align timeout");
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted()override{
        auto ctx=get_ctx(*this);
        ctx->trace(name(),"HALTED");
    }
};
'''
        s = s[:idx] + cls + s[idx:]
        changed = True
        print("[OK] inserted RequestOakPackageSearchAlign class")

    # ------------------------------------------------------------
    # 7. Register BT node
    # ------------------------------------------------------------
    if "registerNodeType<RequestOakPackageSearchAlign>" not in s:
        old = 'factory_.registerNodeType<ScanTargetPackageOnShelf>("ScanTargetPackageOnShelf");'
        new = old + 'factory_.registerNodeType<RequestOakPackageSearchAlign>("RequestOakPackageSearchAlign");'
        s, ok = replace_once(s, old, new, "register RequestOakPackageSearchAlign")

        if not ok:
            old = 'factory_.registerNodeType<AlignOakToTargetShelfFront>("AlignOakToTargetShelfFront");'
            new = old + 'factory_.registerNodeType<RequestOakPackageSearchAlign>("RequestOakPackageSearchAlign");'
            s, ok = replace_once(s, old, new, "register RequestOakPackageSearchAlign fallback")

        changed |= ok

    if changed:
        CPP.write_text(s)
        print(f"[OK] patched {CPP}")
    else:
        print("[SKIP] C++ already patched or no matching changes")


def patch_xml():
    if not XML.exists():
        print(f"[SKIP] XML not found: {XML}")
        return

    backup(XML)
    s = XML.read_text(errors="ignore")

    if "<RequestOakPackageSearchAlign" in s:
        print("[SKIP] XML already has RequestOakPackageSearchAlign")
        return

    insert = '''


        <!-- D. 到达并验证目标货架后：触发 OAK 自动对正。
             这里不直接在 BT 里控制底盘，而是发布 /target_detection/package_search/request，
             等待 oak_package_search_node 返回 aligned / found。
             测试阶段请把 oak_package_search_node 的 align_only 设为 true。 -->
        <Sequence name="AlignOakByPackageSearchNode">
          <RequestOakPackageSearchAlign/>
        </Sequence>'''

    old = '''        </Fallback>

      </Sequence>


      <!-- 3. 当前阶段临时结束：先不进入找包 / 取件 -->'''

    new = '''        </Fallback>''' + insert + '''

      </Sequence>


      <!-- 3. 当前阶段临时结束：已加入 OAK 对正；测试阶段仍先不进入找包 / 取件 -->'''

    if old not in s:
        raise SystemExit("[ERROR] cannot find XML insertion point in pickup_task_three_node_main.xml")

    s = s.replace(old, new, 1)
    XML.write_text(s)
    print(f"[OK] patched XML: {XML}")


def patch_yaml():
    if not CFG.exists():
        print(f"[SKIP] YAML not found: {CFG}")
        return

    backup(CFG)
    cfg = yaml.safe_load(CFG.read_text()) or {}
    node = cfg.setdefault("mission_btcpp_node", {}).setdefault("ros__parameters", {})

    node.update({
        "package_search_request_topic": "/target_detection/package_search/request",
        "package_search_status_topic": "/target_detection/package_search/status",
        "package_search_timeout_sec": 20.0,
    })

    CFG.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
    print(f"[OK] patched YAML: {CFG}")


def patch_launch():
    if not LAUNCH.exists():
        print(f"[SKIP] launch not found: {LAUNCH}")
        return

    backup(LAUNCH)
    s = LAUNCH.read_text(errors="ignore")

    if "'package_search_request_topic'" in s:
        print("[SKIP] launch already patched")
        return

    old = "'current_shelf_topic': '/semantic/current_shelf',"
    new = (
        old
        + "\n                'package_search_request_topic': '/target_detection/package_search/request',"
        + "\n                'package_search_status_topic': '/target_detection/package_search/status',"
        + "\n                'package_search_timeout_sec': 20.0,"
    )

    if old not in s:
        print("[WARN] launch insertion point not found; YAML/source params are still patched")
        return

    s = s.replace(old, new, 1)
    LAUNCH.write_text(s)
    print(f"[OK] patched launch: {LAUNCH}")


def main():
    print("============================================================")
    print("[PATCH] mission BT -> oak_package_search_node align request")
    print("============================================================")
    print(f"[INFO] package root: {PKG}")

    patch_cpp()
    patch_xml()
    patch_yaml()
    patch_launch()

    print("============================================================")
    print("[DONE]")
    print("Next commands:")
    print("  cd /host-home")
    print("  source /opt/ros/humble/setup.bash")
    print("  colcon build --packages-select robot_logistics_task")
    print("  source install/setup.bash")
    print("============================================================")


if __name__ == "__main__":
    main()
