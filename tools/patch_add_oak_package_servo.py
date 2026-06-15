#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Incrementally add OAK package servo support to current robot2 workspace.

Run from /host-home after unzipping this bundle:
  python3 tools/patch_add_oak_package_servo.py

This script is intentionally incremental:
  - it DOES NOT replace the existing BT XML file path;
  - it inserts new BT nodes into pickup_task_scan_finally_oak_realign.xml;
  - it registers small request/wait BT nodes in mission_btcpp_node.cpp;
  - it adds oak_package_servo to the existing task bringup mode.
"""

from __future__ import annotations

import re
import shutil
import sys
import time
from pathlib import Path

STAMP = time.strftime('%Y%m%d_%H%M%S')


def backup(path: Path) -> None:
    if path.exists():
        bak = path.with_suffix(path.suffix + f'.bak_oak_pkg_servo_{STAMP}')
        shutil.copy2(path, bak)
        print(f'[backup] {path} -> {bak}')


def read(path: Path) -> str:
    return path.read_text(encoding='utf-8')


def write(path: Path, data: str) -> None:
    path.write_text(data, encoding='utf-8')
    print(f'[write] {path}')


def patch_bringup(root: Path) -> None:
    path = root / 'src/Navigation/robot2_bringup/config/bringup_modes.yaml'
    data = read(path)
    orig = data

    if '  oak_package_servo:' not in data:
        module = '''
  oak_package_servo:
    type: node
    package: oak_package_servo
    executable: oak_package_servo_node
    name: oak_package_servo_node
    output: screen
    delay: 18.0
    respawn: true
    respawn_delay: 2.0
    parameters:
    - /host-home/src/target_detection/oak_package_servo/config/oak_package_servo.yaml
    - request_topic: /target_detection/package_servo/request
      status_topic: /target_detection/package_servo/status
      datamatrix_topic: /oak/datamatrix_3d_result_json
      cmd_vel_topic: /cmd_vel_auto
      base_frame: base_link
      oak_frame: oak-cam-Link
      map_frame: map
      scan_topic: /scan
      map_topic: /map
      semantic_shelf_topic: /semantic/current_shelf
'''
        data = data.replace('\nmodes:\n', module + '\nmodes:\n')
        print('[patch] added modules.oak_package_servo')
    else:
        print('[skip] modules.oak_package_servo already exists')

    if re.search(r'(?m)^    - oak_package_servo\s*$', data) is None:
        data = data.replace('    - oak_package_search\n', '    - oak_package_search\n    - oak_package_servo\n', 1)
        print('[patch] added oak_package_servo to modes.task.enable')
    else:
        print('[skip] oak_package_servo already in modes.task.enable')

    # Add mission_bt parameters, but do not change bt_xml.
    # The current project uses package_search_timeout_sec: 45.0. Match any value.
    if 'oak_servo_center_timeout_sec:' not in data:
        def add_servo_params(match):
            line = match.group(0)
            return (
                line
                + '      package_servo_request_topic: /target_detection/package_servo/request\n'
                + '      package_servo_status_topic: /target_detection/package_servo/status\n'
                + '      oak_servo_center_timeout_sec: 12.0\n'
                + '      oak_servo_approach_timeout_sec: 18.0\n'
                + '      oak_servo_fine_timeout_sec: 8.0\n'
            )
        data, n = re.subn(r'(?m)^      package_search_timeout_sec: [^\n]+\n', add_servo_params, data, count=1)
        if n:
            print('[patch] added mission_bt package_servo parameters')
        else:
            print('[warn] did not find package_search_timeout_sec line for mission_bt servo params')
    else:
        print('[skip] mission_bt package_servo parameters already exist')

    if data != orig:
        backup(path)
        write(path, data)


def patch_bt_xml(root: Path) -> None:
    path = root / 'src/Navigation/robot_logistics_task/behavior_tree/pickup_task_scan_finally_oak_realign.xml'
    data = read(path)
    orig = data
    if '<RequestOakServoCenterAlign/>' in data:
        print('[skip] BT XML already contains OAK servo nodes')
        return

    pattern = '''        <Sequence name="ScanSuccess_ReAlign_ThenDone">
          <ScanTargetPackageOnShelf/>
          <RequestOakPackageSearchAlign/>
          <WaitAfterOakAlign/>
          <PublishTaskDone/>
        </Sequence>'''
    replacement = '''        <Sequence name="ScanSuccess_ReAlign_ThenDone">
          <ScanTargetPackageOnShelf/>

          <!-- 扫描找到目标后，先缓存目标；不在扫描角度下取件 -->
          <CacheFoundPackagePose/>

          <!-- 保留原逻辑：扫描结束后先让 OAK / 车体重新正对货架 -->
          <RequestOakPackageSearchAlign/>
          <WaitAfterOakAlign/>

          <!-- 新增：OAK + URDF 解算 base_link 控制量，输出标准 cmd_vel -->
          <RequestOakServoCenterAlign/>
          <RequestOakServoApproach/>
          <RequestOakServoFineAlign/>

          <PublishTaskDone/>
        </Sequence>'''
    if pattern not in data:
        raise RuntimeError('Could not find expected ScanSuccess_ReAlign_ThenDone block in BT XML. Please patch manually.')
    data = data.replace(pattern, replacement, 1)
    if data != orig:
        backup(path)
        write(path, data)


def patch_mission_cpp(root: Path) -> None:
    path = root / 'src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp'
    data = read(path)
    orig = data

    if 'RequestOakServoCenterAlign' in data:
        print('[skip] mission_btcpp_node.cpp already contains OAK servo BT nodes')
        return

    # SharedContext string fields.
    data = data.replace(
        'package_search_request_topic="/target_detection/package_search/request",package_search_status_topic="/target_detection/package_search/status";double session_pose_max_age_sec',
        'package_search_request_topic="/target_detection/package_search/request",package_search_status_topic="/target_detection/package_search/status",package_servo_request_topic="/target_detection/package_servo/request",package_servo_status_topic="/target_detection/package_servo/status",latest_package_servo_json,latest_package_servo_status;double oak_servo_center_timeout_sec=12.0,oak_servo_approach_timeout_sec=18.0,oak_servo_fine_timeout_sec=8.0;double session_pose_max_age_sec',
        1,
    )
    # Publisher field.
    data = data.replace(
        'package_search_request_pub;',
        'package_search_request_pub,package_servo_request_pub;',
        1,
    )
    # Subscriber field.
    data = data.replace(
        'package_search_status_sub_;',
        'package_search_status_sub_,package_servo_status_sub_;',
        1,
    )
    # Declarations.
    data = data.replace(
        'declare_parameter<std::string>("package_search_status_topic","/target_detection/package_search/status");declare_parameter<double>("package_search_timeout_sec",20.0);',
        'declare_parameter<std::string>("package_search_status_topic","/target_detection/package_search/status");declare_parameter<double>("package_search_timeout_sec",20.0);declare_parameter<std::string>("package_servo_request_topic","/target_detection/package_servo/request");declare_parameter<std::string>("package_servo_status_topic","/target_detection/package_servo/status");declare_parameter<double>("oak_servo_center_timeout_sec",12.0);declare_parameter<double>("oak_servo_approach_timeout_sec",18.0);declare_parameter<double>("oak_servo_fine_timeout_sec",8.0);',
        1,
    )
    # Context parameter copy.
    data = data.replace(
        'ctx_->package_search_request_topic=get_parameter("package_search_request_topic").as_string();ctx_->package_search_status_topic=get_parameter("package_search_status_topic").as_string();ctx_->oak_align_yaw_tolerance_deg',
        'ctx_->package_search_request_topic=get_parameter("package_search_request_topic").as_string();ctx_->package_search_status_topic=get_parameter("package_search_status_topic").as_string();ctx_->package_servo_request_topic=get_parameter("package_servo_request_topic").as_string();ctx_->package_servo_status_topic=get_parameter("package_servo_status_topic").as_string();ctx_->oak_servo_center_timeout_sec=get_parameter("oak_servo_center_timeout_sec").as_double();ctx_->oak_servo_approach_timeout_sec=get_parameter("oak_servo_approach_timeout_sec").as_double();ctx_->oak_servo_fine_timeout_sec=get_parameter("oak_servo_fine_timeout_sec").as_double();ctx_->oak_align_yaw_tolerance_deg',
        1,
    )
    # Publisher creation.
    data = data.replace(
        'ctx_->package_search_request_pub=create_publisher<std_msgs::msg::String>(ctx_->package_search_request_topic,10);',
        'ctx_->package_search_request_pub=create_publisher<std_msgs::msg::String>(ctx_->package_search_request_topic,10);ctx_->package_servo_request_pub=create_publisher<std_msgs::msg::String>(ctx_->package_servo_request_topic,10);',
        1,
    )
    # Subscriber creation.
    data = data.replace(
        'package_search_status_sub_=create_subscription<std_msgs::msg::String>(ctx_->package_search_status_topic,10,[this](std_msgs::msg::String::SharedPtr m){std::lock_guard<std::mutex>l(ctx_->mutex);ctx_->latest_package_search_json=m->data;std::string st;if(get_string_json(m->data,"status",st))ctx_->latest_package_search_status=st;});',
        'package_search_status_sub_=create_subscription<std_msgs::msg::String>(ctx_->package_search_status_topic,10,[this](std_msgs::msg::String::SharedPtr m){std::lock_guard<std::mutex>l(ctx_->mutex);ctx_->latest_package_search_json=m->data;std::string st;if(get_string_json(m->data,"status",st))ctx_->latest_package_search_status=st;});package_servo_status_sub_=create_subscription<std_msgs::msg::String>(ctx_->package_servo_status_topic,10,[this](std_msgs::msg::String::SharedPtr m){std::lock_guard<std::mutex>l(ctx_->mutex);ctx_->latest_package_servo_json=m->data;std::string st;if(get_string_json(m->data,"status",st))ctx_->latest_package_servo_status=st;});',
        1,
    )

    servo_classes = r'''
class CacheFoundPackagePose:public BT::SyncActionNode{public:CacheFoundPackagePose(const std::string&n,const BT::NodeConfiguration&c):BT::SyncActionNode(n,c){}SIMPLE_PORTS BT::NodeStatus tick()override{auto ctx=get_ctx(*this);ctx->set_state("CacheFoundPackagePose");std::lock_guard<std::mutex>l(ctx->mutex);if(!ctx->task.active){ctx->status("FAILURE","no active task for package cache");ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}std_msgs::msg::String msg;std::stringstream ss;ss<<"{\"command\":\"cache\",\"package_code\":\""<<ctx->task.package_code<<"\",\"pickup_code\":\""<<ctx->task.pickup_code<<"\",\"shelf_id\":\""<<ctx->task.shelf_id<<"\",\"layer_id\":\""<<ctx->task.layer_id<<"\",\"shelf_front_yaw\":"<<ctx->resolved_pose.yaw<<",\"source\":\"mission_btcpp_node\"}";msg.data=ss.str();if(ctx->package_servo_request_pub)ctx->package_servo_request_pub->publish(msg);ctx->status("SUCCESS","cached package target request for oak servo");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}};

class RequestOakServoStage:public BT::StatefulActionNode{std::string stage_;double timeout_=12.0,start_=0.0;public:RequestOakServoStage(const std::string&n,const BT::NodeConfiguration&c,const std::string&s,double t):BT::StatefulActionNode(n,c),stage_(s),timeout_(t){}SIMPLE_PORTS BT::NodeStatus onStart()override{auto ctx=get_ctx(*this);ctx->set_state("RequestOakServo"+stage_);start_=steady_seconds();std::lock_guard<std::mutex>l(ctx->mutex);if(!ctx->task.active){ctx->status("FAILURE","no active task for oak servo");ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}double use_timeout=timeout_;if(stage_=="center")use_timeout=ctx->oak_servo_center_timeout_sec;else if(stage_=="approach")use_timeout=ctx->oak_servo_approach_timeout_sec;else if(stage_=="fine")use_timeout=ctx->oak_servo_fine_timeout_sec;timeout_=use_timeout;ctx->latest_package_servo_status.clear();ctx->latest_package_servo_json.clear();std_msgs::msg::String msg;std::stringstream ss;ss<<"{\"command\":\""<<stage_<<"\",\"package_code\":\""<<ctx->task.package_code<<"\",\"pickup_code\":\""<<ctx->task.pickup_code<<"\",\"shelf_id\":\""<<ctx->task.shelf_id<<"\",\"layer_id\":\""<<ctx->task.layer_id<<"\",\"shelf_front_yaw\":"<<ctx->resolved_pose.yaw<<",\"timeout_sec\":"<<use_timeout<<",\"source\":\"mission_btcpp_node\"}";msg.data=ss.str();if(!ctx->package_servo_request_pub){ctx->status("FAILURE","package_servo_request_pub is null");ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}ctx->package_servo_request_pub->publish(msg);ctx->status("RUNNING","published oak package servo request: "+stage_);ctx->trace(name(),"RUNNING");return BT::NodeStatus::RUNNING;}BT::NodeStatus onRunning()override{auto ctx=get_ctx(*this);std::lock_guard<std::mutex>l(ctx->mutex);std::string st,sg;get_string_json(ctx->latest_package_servo_json,"status",st);get_string_json(ctx->latest_package_servo_json,"stage",sg);if(sg==stage_&&st=="succeeded"){ctx->status("SUCCESS","oak servo "+stage_+" reached");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}if(sg==stage_&&st=="failed"){ctx->status("FAILURE","oak servo "+stage_+" failed");ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}if(steady_seconds()-start_>timeout_){std_msgs::msg::String msg;msg.data="{\"command\":\"stop\",\"source\":\"mission_btcpp_node_timeout\"}";if(ctx->package_servo_request_pub)ctx->package_servo_request_pub->publish(msg);ctx->status("FAILURE","oak servo "+stage_+" timeout");ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}return BT::NodeStatus::RUNNING;}void onHalted()override{auto ctx=get_ctx(*this);std_msgs::msg::String msg;msg.data="{\"command\":\"stop\",\"source\":\"mission_btcpp_node_halted\"}";if(ctx->package_servo_request_pub)ctx->package_servo_request_pub->publish(msg);ctx->trace(name(),"HALTED");}};
class RequestOakServoCenterAlign:public RequestOakServoStage{public:RequestOakServoCenterAlign(const std::string&n,const BT::NodeConfiguration&c):RequestOakServoStage(n,c,"center",12.0){}};
class RequestOakServoApproach:public RequestOakServoStage{public:RequestOakServoApproach(const std::string&n,const BT::NodeConfiguration&c):RequestOakServoStage(n,c,"approach",18.0){}};
class RequestOakServoFineAlign:public RequestOakServoStage{public:RequestOakServoFineAlign(const std::string&n,const BT::NodeConfiguration&c):RequestOakServoStage(n,c,"fine",8.0){}};
'''
    data = data.replace('class SendPlatformTarget:public BT::SyncActionNode', servo_classes + '\nclass SendPlatformTarget:public BT::SyncActionNode', 1)

    data = data.replace(
        'factory_.registerNodeType<ScanTargetPackageOnShelf>("ScanTargetPackageOnShelf");factory_.registerNodeType<RequestOakPackageSearchAlign>("RequestOakPackageSearchAlign");factory_.registerNodeType<SendPlatformTarget>("SendPlatformTarget");',
        'factory_.registerNodeType<ScanTargetPackageOnShelf>("ScanTargetPackageOnShelf");factory_.registerNodeType<RequestOakPackageSearchAlign>("RequestOakPackageSearchAlign");factory_.registerNodeType<CacheFoundPackagePose>("CacheFoundPackagePose");factory_.registerNodeType<RequestOakServoCenterAlign>("RequestOakServoCenterAlign");factory_.registerNodeType<RequestOakServoApproach>("RequestOakServoApproach");factory_.registerNodeType<RequestOakServoFineAlign>("RequestOakServoFineAlign");factory_.registerNodeType<SendPlatformTarget>("SendPlatformTarget");',
        1,
    )

    if data == orig:
        raise RuntimeError('mission_btcpp_node.cpp was not patched; expected patterns not found.')
    backup(path)
    write(path, data)


def main() -> int:
    root = Path.cwd()
    if not (root / 'src').exists():
        print('[error] run this script from workspace root, e.g. /host-home', file=sys.stderr)
        return 2
    pkg = root / 'src/target_detection/oak_package_servo'
    if not pkg.exists():
        print(f'[error] package directory not found: {pkg}. Did you unzip the bundle at workspace root?', file=sys.stderr)
        return 2
    patch_bringup(root)
    patch_bt_xml(root)
    patch_mission_cpp(root)
    print('\n[done] OAK package servo incremental patch applied.')
    print('Next: colcon build --symlink-install --packages-select oak_package_servo robot_logistics_task robot2_bringup')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
