#!/usr/bin/env bash
set -euo pipefail

cd /host-home

python3 - <<'PY'
from pathlib import Path
import re
import time

CPP = Path('/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp')
XML = Path('/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml')

if not CPP.exists():
    raise SystemExit(f'[error] missing C++ file: {CPP}')
if not XML.exists():
    raise SystemExit(f'[error] missing BT XML file: {XML}')

stamp = time.strftime('%Y%m%d_%H%M%S')

cpp = CPP.read_text()
cpp_bak = CPP.with_suffix('.cpp.bak_open_gripper_before_search_' + stamp)
cpp_bak.write_text(cpp)
print(f'[backup] {cpp_bak}')

CLASS_CODE = r'''
// OPEN_GRIPPER_BEFORE_SEARCH_20260618
// Before entering OAK shelf/package search, make sure the gripper is open.
// This BT node does not publish /platform/target directly. It only calls the
// independent oak_package_grabber node through its JSON request/status topics.
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
    ExecuteOakPackageOpenBeforeSearch(const std::string&n,const BT::NodeConfiguration&c):BT::StatefulActionNode(n,c){}
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

                    // 优先匹配 request_id。
                    // 如果旧版 grabber 没回 request_id，则只接受 open_gripper/ensure_gripper_open 阶段。
                    if(!request_id_.empty() && !rid.empty() && rid!=request_id_)return;
                    if(!stage.empty() && stage!="open_gripper" && stage!="ensure_gripper_open"){
                        if(rid.empty())return;
                    }

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

        RCLCPP_INFO(ctx_->node->get_logger(),"[ExecuteOakPackageOpenBeforeSearch] request=%s",msg.data.c_str());
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

if 'class ExecuteOakPackageOpenBeforeSearch' not in cpp:
    markers = [
        'class RequestOakPackageSearchAlign',
        'class ScanTargetPackageOnShelf',
        'class CacheFoundPackagePose',
    ]
    inserted = False
    for marker in markers:
        idx = cpp.find(marker)
        if idx >= 0:
            cpp = cpp[:idx] + CLASS_CODE + '\n' + cpp[idx:]
            print(f'[patch] inserted ExecuteOakPackageOpenBeforeSearch before {marker}')
            inserted = True
            break
    if not inserted:
        raise SystemExit('[error] cannot find insertion point for ExecuteOakPackageOpenBeforeSearch')
else:
    print('[skip] ExecuteOakPackageOpenBeforeSearch class already exists')

if 'registerNodeType<ExecuteOakPackageOpenBeforeSearch>' not in cpp:
    reg_line = 'factory_.registerNodeType<ExecuteOakPackageOpenBeforeSearch>("ExecuteOakPackageOpenBeforeSearch");'
    candidates = [
        'factory_.registerNodeType<RequestOakPackageSearchAlign>("RequestOakPackageSearchAlign");',
        'factory_.registerNodeType<ScanTargetPackageOnShelf>("ScanTargetPackageOnShelf");',
    ]
    registered = False
    for cand in candidates:
        if cand in cpp:
            cpp = cpp.replace(cand, reg_line + cand, 1)
            print(f'[patch] registered ExecuteOakPackageOpenBeforeSearch before {cand}')
            registered = True
            break
    if not registered:
        marker = 'blackboard_=BT::Blackboard::create();'
        if marker not in cpp:
            raise SystemExit('[error] cannot find registration point')
        cpp = cpp.replace(marker, reg_line + marker, 1)
        print('[patch] registered ExecuteOakPackageOpenBeforeSearch before blackboard creation')
else:
    print('[skip] ExecuteOakPackageOpenBeforeSearch already registered')

CPP.write_text(cpp)

xml = XML.read_text()
xml_bak = XML.with_suffix('.xml.bak_open_gripper_before_search_' + stamp)
xml_bak.write_text(xml)
print(f'[backup] {xml_bak}')

if '<ExecuteOakPackageOpenBeforeSearch/>' not in xml:
    if '<WaitNavSettle/>' in xml:
        xml = xml.replace(
            '<WaitNavSettle/>',
            '<WaitNavSettle/>\n\n      <!-- 进入 OAK 搜索模式前先开夹，避免夹爪挡住包裹码 -->\n      <ExecuteOakPackageOpenBeforeSearch/>',
            1,
        )
        print('[patch] XML inserted ExecuteOakPackageOpenBeforeSearch after WaitNavSettle')
    elif '<WaitNavArrive/>' in xml:
        xml = xml.replace(
            '<WaitNavArrive/>',
            '<WaitNavArrive/>\n\n      <!-- 进入 OAK 搜索模式前先开夹，避免夹爪挡住包裹码 -->\n      <ExecuteOakPackageOpenBeforeSearch/>',
            1,
        )
        print('[patch] XML inserted ExecuteOakPackageOpenBeforeSearch after WaitNavArrive')
    elif '<RequestOakPackageSearchAlign/>' in xml:
        xml = xml.replace(
            '<RequestOakPackageSearchAlign/>',
            '<!-- 进入 OAK 搜索模式前先开夹，避免夹爪挡住包裹码 -->\n      <ExecuteOakPackageOpenBeforeSearch/>\n      <RequestOakPackageSearchAlign/>',
            1,
        )
        print('[patch] XML inserted ExecuteOakPackageOpenBeforeSearch before first RequestOakPackageSearchAlign')
    else:
        raise SystemExit('[error] cannot find XML insertion point')
else:
    print('[skip] XML already contains ExecuteOakPackageOpenBeforeSearch')

XML.write_text(xml)

print('[done] patch source files complete')
PY

set +u
source /opt/ros/humble/setup.bash
source /host-home/install/setup.bash
set -u

colcon build --symlink-install --packages-select robot_logistics_task

set +u
source /host-home/install/setup.bash
set -u

echo '[done] built robot_logistics_task with ExecuteOakPackageOpenBeforeSearch'
