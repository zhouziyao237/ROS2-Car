#!/usr/bin/env bash
set -euo pipefail

cd /host-home

echo "========== [1/5] Patch oak_package_grabber: open_gripper must not block on feedback =========="

python3 - <<'PY'
from pathlib import Path
import time
import re

p = Path("/host-home/src/target_capture/oak_package_grabber/oak_package_grabber/oak_package_grabber_node.py")
if not p.exists():
    raise SystemExit(f"[error] missing file: {p}")

s = p.read_text()
bak = p.with_suffix(".py.bak_final_open_no_feedback_" + time.strftime("%Y%m%d_%H%M%S"))
bak.write_text(s)
print(f"[backup] {bak}")

def replace_stage_block(src: str, stage: str, new_block: str) -> str:
    key = f"        if self.stage == '{stage}':\n"
    start = src.find(key)
    if start < 0:
        print(f"[warn] stage block not found: {stage}")
        return src

    # 找下一个同级 stage block
    next_start = src.find("\n        if self.stage == '", start + len(key))
    if next_start < 0:
        # 找到方法后面一般的 timeout/fallback，尽量不要吃掉太多
        next_start = src.find("\n        self.fail(", start + len(key))
    if next_start < 0:
        raise SystemExit(f"[error] cannot determine block end for stage {stage}")

    print(f"[patch] replace stage block: {stage}")
    return src[:start] + new_block + src[next_start:]


ENSURE_GRIPPER_OPEN_BLOCK = """        if self.stage == 'ENSURE_GRIPPER_OPEN':
            # Requirement:
            # - Before real grab extension, make sure gripper is commanded OPEN.
            # - Do NOT block on /platform/current_status target_grab_mode feedback,
            #   because the feedback can be stale while the command path is valid.
            # - Keep publishing OPEN for open_wait_sec, then continue to EXTEND_SLIDE.
            self.publish_platform_target(
                self.actual_slide_x_mm,
                self.actual_screw_x_mm,
                self.grab_open_value,
            )
            self.publish_running(
                'ensure_gripper_open',
                'opening_gripper_before_extend_no_feedback_block'
            )

            if self.stage_elapsed() >= self.open_wait_sec:
                self.goto('EXTEND_SLIDE')
            return
"""

DEBUG_OPEN_GRIPPER_BLOCK = """        if self.stage == 'DEBUG_OPEN_GRIPPER':
            # Used by:
            # - manual debug command: open_gripper
            # - BT node ExecuteOakPackageOpenBeforeSearch
            #
            # This must behave like the user's manual ros2 topic pub command:
            # publish OPEN for a short duration, then report succeeded.
            # It must NOT fail the BT only because board feedback did not echo 0.
            self.publish_platform_target(
                self.actual_slide_x_mm,
                self.actual_screw_x_mm,
                self.grab_open_value,
            )
            self.publish_running(
                'debug_open_gripper',
                'opening_gripper_debug_no_feedback_block'
            )

            if self.stage_elapsed() >= self.open_wait_sec:
                self.succeed('debug_open_gripper_done')
            return
"""

DEBUG_CLOSE_GRIPPER_BLOCK = """        if self.stage == 'DEBUG_CLOSE_GRIPPER':
            # Same rule as open: command close for close_wait_sec, then succeed.
            # Do not block on feedback here.
            self.publish_platform_target(
                self.actual_slide_x_mm,
                self.actual_screw_x_mm,
                self.grab_close_value,
            )
            self.publish_running(
                'debug_close_gripper',
                'closing_gripper_debug_no_feedback_block'
            )

            if self.stage_elapsed() >= self.close_wait_sec:
                self.succeed('debug_close_gripper_done')
            return
"""

s = replace_stage_block(s, "ENSURE_GRIPPER_OPEN", ENSURE_GRIPPER_OPEN_BLOCK)
s = replace_stage_block(s, "DEBUG_OPEN_GRIPPER", DEBUG_OPEN_GRIPPER_BLOCK)
s = replace_stage_block(s, "DEBUG_CLOSE_GRIPPER", DEBUG_CLOSE_GRIPPER_BLOCK)

p.write_text(s)
print("[done] patched oak_package_grabber_node.py")
PY


echo "========== [2/5] Patch oak_package_grabber YAML timing =========="

python3 - <<'PY'
from pathlib import Path
import time
import re

files = [
    Path("/host-home/src/target_capture/oak_package_grabber/config/oak_package_grabber.yaml"),
    Path("/host-home/install/oak_package_grabber/share/oak_package_grabber/config/oak_package_grabber.yaml"),
]

for p in files:
    if not p.exists():
        print(f"[skip missing] {p}")
        continue

    s = p.read_text()
    bak = p.with_suffix(".yaml.bak_final_open_timing_" + time.strftime("%Y%m%d_%H%M%S"))
    bak.write_text(s)

    ns = s

    if re.search(r"open_wait_sec:\s*[0-9.]+", ns):
        ns = re.sub(r"open_wait_sec:\s*[0-9.]+", "open_wait_sec: 1.0", ns)
    else:
        ns += "\n    open_wait_sec: 1.0\n"

    if re.search(r"close_wait_sec:\s*[0-9.]+", ns):
        ns = re.sub(r"close_wait_sec:\s*[0-9.]+", "close_wait_sec: 1.0", ns)
    else:
        ns += "\n    close_wait_sec: 1.0\n"

    p.write_text(ns)
    print(f"[patch yaml] {p}")

print("[done] patched grabber yaml timing")
PY


echo "========== [3/5] Patch mission_btcpp_node.cpp: add/keep ExecuteOakPackageOpenBeforeSearch =========="

python3 - <<'PY'
from pathlib import Path
import time

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
if not CPP.exists():
    raise SystemExit(f"[error] missing file: {CPP}")

cpp = CPP.read_text()
bak = CPP.with_suffix(".cpp.bak_final_open_before_search_" + time.strftime("%Y%m%d_%H%M%S"))
bak.write_text(cpp)
print(f"[backup] {bak}")

CLASS_CODE = r'''
// OPEN_GRIPPER_BEFORE_SEARCH_FINAL
// Before entering OAK shelf/package search, make sure the gripper is open.
// This node does NOT publish /platform/target directly.
// It only calls the independent oak_package_grabber node by JSON request/status.
class ExecuteOakPackageOpenBeforeSearch:public BT::StatefulActionNode{
    std::shared_ptr<SharedContext> ctx_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;

    std::mutex status_mutex_;
    std::string request_id_;
    std::string latest_status_;
    std::string latest_reason_;
    std::string latest_request_id_;
    std::string latest_stage_;

    double start_=0.0;
    double timeout_=5.0;

public:
    ExecuteOakPackageOpenBeforeSearch(const std::string&n,const BT::NodeConfiguration&c)
    :BT::StatefulActionNode(n,c){}

    SIMPLE_PORTS

    BT::NodeStatus onStart()override{
        ctx_=get_ctx(*this);
        ctx_->set_state("ExecuteOakPackageOpenBeforeSearch");

        start_=steady_seconds();
        timeout_=5.0;
        request_id_="open_before_search_"+std::to_string((long long)(start_*1000.0));

        {
            std::lock_guard<std::mutex> lk(status_mutex_);
            latest_status_.clear();
            latest_reason_.clear();
            latest_request_id_.clear();
            latest_stage_.clear();
        }

        if(!request_pub_){
            request_pub_=ctx_->node->create_publisher<std_msgs::msg::String>(
                "/target_capture/oak_package_grabber/request",10);
        }

        if(!status_sub_){
            status_sub_=ctx_->node->create_subscription<std_msgs::msg::String>(
                "/target_capture/oak_package_grabber/status",10,
                [this](std_msgs::msg::String::SharedPtr m){
                    std::string st,reason,rid,stage;
                    get_string_json(m->data,"status",st);
                    get_string_json(m->data,"reason",reason);
                    get_string_json(m->data,"request_id",rid);
                    get_string_json(m->data,"stage",stage);

                    if(st.empty())return;

                    // Match current request_id. This avoids consuming old debug statuses.
                    if(!request_id_.empty() && !rid.empty() && rid!=request_id_)return;

                    std::lock_guard<std::mutex> lk(status_mutex_);
                    latest_status_=st;
                    latest_reason_=reason;
                    latest_request_id_=rid;
                    latest_stage_=stage;
                });
        }

        std::string pickup_code,package_code,shelf_id,layer_id;
        {
            std::lock_guard<std::mutex> l(ctx_->mutex);
            if(ctx_->task.active){
                pickup_code=ctx_->task.pickup_code;
                package_code=ctx_->task.package_code;
                shelf_id=ctx_->task.shelf_id;
                layer_id=ctx_->task.layer_id;
            }
        }

        std_msgs::msg::String msg;
        std::stringstream ss;
        ss<<"{\"command\":\"open_gripper\""
          <<",\"request_id\":\""<<request_id_<<"\""
          <<",\"pickup_code\":\""<<pickup_code<<"\""
          <<",\"package_code\":\""<<package_code<<"\""
          <<",\"shelf_id\":\""<<shelf_id<<"\""
          <<",\"layer_id\":\""<<layer_id<<"\""
          <<",\"timeout_sec\":"<<timeout_
          <<",\"source\":\"mission_btcpp_node_open_before_search\"}";
        msg.data=ss.str();

        request_pub_->publish(msg);

        RCLCPP_INFO(ctx_->node->get_logger(),
                    "[ExecuteOakPackageOpenBeforeSearch] request=%s",
                    msg.data.c_str());

        ctx_->status("RUNNING","opening gripper before OAK package search");
        ctx_->trace(name(),"RUNNING");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()override{
        auto ctx=ctx_?ctx_:get_ctx(*this);

        {
            std::lock_guard<std::mutex> lk(status_mutex_);

            if(latest_status_=="succeeded" || latest_status_=="success" || latest_status_=="done"){
                ctx->status("SUCCESS","gripper opened before OAK package search");
                ctx->trace(name(),"SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }

            if(latest_status_=="failed" || latest_status_=="failure" || latest_status_=="error"){
                ctx->status("FAILURE","failed to open gripper before search: "+latest_reason_);
                ctx->trace(name(),"FAILURE");
                return BT::NodeStatus::FAILURE;
            }
        }

        if(steady_seconds()-start_>timeout_){
            ctx->status("FAILURE","timeout opening gripper before OAK package search");
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted()override{
        auto ctx=ctx_?ctx_:get_ctx(*this);
        ctx->trace(name(),"HALTED");
    }
};
'''

if "class ExecuteOakPackageOpenBeforeSearch" not in cpp:
    insert_markers = [
        "class RequestOakPackageSearchAlign",
        "class ScanTargetPackageOnShelf",
        "class CacheFoundPackagePose",
    ]

    inserted = False
    for marker in insert_markers:
        idx = cpp.find(marker)
        if idx >= 0:
            cpp = cpp[:idx] + CLASS_CODE + "\n" + cpp[idx:]
            print(f"[patch] inserted ExecuteOakPackageOpenBeforeSearch before {marker}")
            inserted = True
            break

    if not inserted:
        raise SystemExit("[error] cannot find class insertion point")
else:
    print("[skip] ExecuteOakPackageOpenBeforeSearch class already exists")

reg = 'factory_.registerNodeType<ExecuteOakPackageOpenBeforeSearch>("ExecuteOakPackageOpenBeforeSearch");'

if reg not in cpp:
    candidates = [
        'factory_.registerNodeType<RequestOakPackageSearchAlign>("RequestOakPackageSearchAlign");',
        'factory_.registerNodeType<ScanTargetPackageOnShelf>("ScanTargetPackageOnShelf");',
        'factory_.registerNodeType<ExecuteOakPackageGrab>("ExecuteOakPackageGrab");',
    ]

    registered = False
    for cand in candidates:
        if cand in cpp:
            cpp = cpp.replace(cand, reg + "\n        " + cand, 1)
            print(f"[patch] registered ExecuteOakPackageOpenBeforeSearch before {cand}")
            registered = True
            break

    if not registered:
        raise SystemExit("[error] cannot find factory registration point")
else:
    print("[skip] ExecuteOakPackageOpenBeforeSearch already registered")

CPP.write_text(cpp)
print("[done] patched mission_btcpp_node.cpp")
PY


echo "========== [4/5] Patch behavior tree XML flow =========="

python3 - <<'PY'
from pathlib import Path
import time

XML = Path("/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml")
if not XML.exists():
    raise SystemExit(f"[error] missing XML: {XML}")

xml = XML.read_text()
bak = XML.with_suffix(".xml.bak_final_open_before_search_" + time.strftime("%Y%m%d_%H%M%S"))
bak.write_text(xml)
print(f"[backup] {bak}")

# Remove duplicate open-before-search nodes if any, then insert exactly once before the first RequestOakPackageSearchAlign.
xml = xml.replace("\n      <ExecuteOakPackageOpenBeforeSearch/>", "")
xml = xml.replace("\n          <ExecuteOakPackageOpenBeforeSearch/>", "")
xml = xml.replace("<ExecuteOakPackageOpenBeforeSearch/>", "")

insert_node = """      <!-- 搜索/对齐前只开爪，不推出，避免夹爪挡住包裹码 -->
      <ExecuteOakPackageOpenBeforeSearch/>

"""

if "<RequestOakPackageSearchAlign/>" in xml:
    xml = xml.replace("<RequestOakPackageSearchAlign/>", insert_node + "      <RequestOakPackageSearchAlign/>", 1)
    print("[patch] inserted ExecuteOakPackageOpenBeforeSearch before first RequestOakPackageSearchAlign")
else:
    raise SystemExit("[error] cannot find <RequestOakPackageSearchAlign/> in XML")

# Ensure real grab node is after guided pickup and before lift down.
if "<ExecuteOakPackageGrab/>" not in xml:
    if "<ExecuteOakGuidedPickup/>" in xml:
        xml = xml.replace(
            "<ExecuteOakGuidedPickup/>",
            "<ExecuteOakGuidedPickup/>\n\n      <!-- 升降视觉对齐成功后才执行真实推出/夹取 -->\n      <ExecuteOakPackageGrab/>",
            1
        )
        print("[patch] inserted ExecuteOakPackageGrab after ExecuteOakGuidedPickup")
    elif "<ExecuteOakGuidedLiftDown/>" in xml:
        xml = xml.replace(
            "<ExecuteOakGuidedLiftDown/>",
            "<!-- 升降视觉对齐成功后才执行真实推出/夹取 -->\n      <ExecuteOakPackageGrab/>\n\n      <ExecuteOakGuidedLiftDown/>",
            1
        )
        print("[patch] inserted ExecuteOakPackageGrab before ExecuteOakGuidedLiftDown")
    else:
        raise SystemExit("[error] cannot find place for ExecuteOakPackageGrab")
else:
    print("[skip] XML already contains ExecuteOakPackageGrab")

XML.write_text(xml)
print("[done] patched BT XML")
PY


echo "========== [5/5] Build packages =========="

set +u
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash
set -u

colcon build --symlink-install --packages-select oak_package_grabber robot_logistics_task

set +u
source /host-home/install/setup.bash
set -u

echo "========== DONE =========="
echo "[done] final flow patched:"
echo "  WaitNavSettle"
echo "  -> ExecuteOakPackageOpenBeforeSearch  only open gripper"
echo "  -> RequestOakPackageSearchAlign"
echo "  -> ScanTargetPackageOnShelf"
echo "  -> RequestOakServoApproach"
echo "  -> ExecuteOakGuidedPickup"
echo "  -> ExecuteOakPackageGrab              extend slide/screw + close + pullback"
echo "  -> ExecuteOakGuidedLiftDown"
