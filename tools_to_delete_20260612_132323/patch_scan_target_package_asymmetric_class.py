#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import re
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

CPP = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")

YAML_FILES = [
    Path("/host-home/src/Navigation/robot_logistics_task/config/mission_task.yaml"),
    Path("/host-home/install/robot_logistics_task/share/robot_logistics_task/config/mission_task.yaml"),
]

LEFT_DEG = 12.0
RIGHT_DEG = 8.0
STEP_DEG = 4.0
WAIT_SEC = 0.05
TIMEOUT_SEC = 12.0


def backup(p: Path, tag: str):
    bak = p.with_suffix(p.suffix + f".bak_{tag}_{STAMP}")
    shutil.copy2(p, bak)
    print(f"[BAK] {p} -> {bak}")


def replace_between(s: str, start_key: str, end_keys):
    start = s.find(start_key)
    if start < 0:
        raise RuntimeError(f"cannot find start key: {start_key}")

    end_candidates = []
    for key in end_keys:
        pos = s.find(key, start + len(start_key))
        if pos >= 0:
            end_candidates.append(pos)

    if not end_candidates:
        raise RuntimeError(f"cannot find any end key after: {start_key}")

    end = min(end_candidates)
    return start, end


NEW_CLASS = r'''
class ScanTargetPackageOnShelf:public BT::StatefulActionNode{
    double start_=0.0;
    double center_yaw_=0.0;
    double target_yaw_=0.0;
    size_t idx_=0;
    std::vector<double> offsets_;

    static double yaw_from_quat(const geometry_msgs::msg::Quaternion& q){
        return std::atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        );
    }

    bool get_base_yaw(std::shared_ptr<SharedContext> ctx,double& yaw){
        try{
            auto tf=ctx->tf_buffer->lookupTransform(
                ctx->map_frame,
                ctx->base_frame,
                tf2::TimePointZero
            );
            yaw=yaw_from_quat(tf.transform.rotation);
            return true;
        }catch(const std::exception& e){
            return false;
        }
    }

    bool target_found(std::shared_ptr<SharedContext> ctx){
        const std::string json=ctx->latest_datamatrix_json;
        if(json.empty()) return false;

        if(!ctx->task.pickup_code.empty() && json.find(ctx->task.pickup_code)!=std::string::npos){
            return true;
        }

        if(!ctx->task.package_code.empty() && json.find(ctx->task.package_code)!=std::string::npos){
            if(ctx->task.layer_id.empty() || json.find(ctx->task.layer_id)!=std::string::npos){
                return true;
            }
        }

        return false;
    }

    void stop(std::shared_ptr<SharedContext> ctx){
        geometry_msgs::msg::Twist cmd;
        ctx->cmd_vel_pub->publish(cmd);
    }

    void build_offsets(std::shared_ptr<SharedContext> ctx){
        offsets_.clear();

        double left=ctx->package_scan_left_deg;
        double right=ctx->package_scan_right_deg;
        double step=ctx->package_scan_step_deg;

        if(left<0.0) left=0.0;
        if(right<0.0) right=0.0;
        if(step<0.1) step=0.1;

        // 你的定义：
        // 货架正面 / OAK 对正后的方向 = 0°
        // 然后按中心优先：0, -step, +step, -2step, +2step...
        offsets_.push_back(0.0);

        double max_lr = left > right ? left : right;

        for(double d=step; d<=max_lr+1e-6; d+=step){
            if(d<=left+1e-6){
                offsets_.push_back(-d);
            }
            if(d<=right+1e-6){
                offsets_.push_back(d);
            }
        }

        idx_=0;
    }

    static double clamp_abs(double v,double max_abs){
        if(max_abs<0.01) max_abs=0.01;
        if(v>max_abs) return max_abs;
        if(v<-max_abs) return -max_abs;
        return v;
    }

public:
    ScanTargetPackageOnShelf(const std::string& n,const BT::NodeConfiguration& c)
    :BT::StatefulActionNode(n,c){}

    SIMPLE_PORTS

    BT::NodeStatus onStart()override{
        start_=steady_seconds();
        auto ctx=get_ctx(*this);

        ctx->set_state("ScanTargetPackageOnShelf");

        {
            std::lock_guard<std::mutex> l(ctx->mutex);
            build_offsets(ctx);

            if(!get_base_yaw(ctx,center_yaw_)){
                center_yaw_=ctx->resolved_pose.yaw;
            }

            if(offsets_.empty()){
                ctx->status("FAILURE","no scan offsets generated");
                ctx->trace(name(),"FAILURE");
                return BT::NodeStatus::FAILURE;
            }

            target_yaw_=norm_angle(center_yaw_ + offsets_[idx_]*M_PI/180.0);

            std::stringstream ss;
            ss<<"asymmetric package scan: center=0 deg, left="
              <<ctx->package_scan_left_deg
              <<" deg, right="<<ctx->package_scan_right_deg
              <<" deg, step="<<ctx->package_scan_step_deg
              <<" deg";
            ctx->status("RUNNING",ss.str());
            ctx->trace(name(),"RUNNING");
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()override{
        auto ctx=get_ctx(*this);

        std::lock_guard<std::mutex> l(ctx->mutex);

        if(target_found(ctx)){
            stop(ctx);
            ctx->status("SUCCESS","target package Data Matrix found");
            ctx->trace(name(),"SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }

        if(steady_seconds()-start_ > ctx->package_scan_timeout_sec){
            stop(ctx);
            ctx->status("FAILURE","target package not found in asymmetric scan timeout");
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }

        if(idx_>=offsets_.size()){
            stop(ctx);
            ctx->status("FAILURE","target package not found in asymmetric scan range");
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }

        double yaw=0.0;
        if(!get_base_yaw(ctx,yaw)){
            stop(ctx);
            ctx->status("RUNNING","waiting TF for package scan yaw");
            return BT::NodeStatus::RUNNING;
        }

        double err=norm_angle(target_yaw_-yaw);
        const double tol=1.5*M_PI/180.0;

        if(std::fabs(err)<=tol){
            stop(ctx);

            std::stringstream ss;
            ss<<"[ScanPackage] reached index="<<idx_
              <<" offset="<<offsets_[idx_]<<" deg";
            RCLCPP_INFO(ctx->node->get_logger(),"%s",ss.str().c_str());

            if(ctx->package_scan_wait_sec>0.001){
                // 保留参数，但不在这里阻塞；下一轮 tick 会继续。
            }

            idx_++;

            if(idx_>=offsets_.size()){
                return BT::NodeStatus::RUNNING;
            }

            target_yaw_=norm_angle(center_yaw_ + offsets_[idx_]*M_PI/180.0);
            return BT::NodeStatus::RUNNING;
        }

        geometry_msgs::msg::Twist cmd;

        // 扫描阶段用低速旋转，避免卡顿和冲击。
        double wz=1.0*err;
        wz=clamp_abs(wz,0.22);

        // 给一个很小的最小速度，避免误差大但车不动。
        if(std::fabs(wz)<0.08){
            wz = wz>=0.0 ? 0.08 : -0.08;
        }

        cmd.angular.z=wz;
        ctx->cmd_vel_pub->publish(cmd);

        return BT::NodeStatus::RUNNING;
    }

    void onHalted()override{
        auto ctx=get_ctx(*this);
        stop(ctx);
        ctx->trace(name(),"HALTED");
    }
};
'''


def patch_cpp():
    if not CPP.exists():
        raise FileNotFoundError(CPP)

    s = CPP.read_text(errors="ignore")
    old = s

    # 1. SharedContext 增加 left/right 参数
    if "package_scan_left_deg" not in s:
        s = re.sub(
            r"double\s+package_scan_range_deg\s*=\s*([^,;]+)\s*,\s*package_scan_step_deg",
            r"double package_scan_range_deg=\1,package_scan_left_deg=\1,package_scan_right_deg=\1,package_scan_step_deg",
            s,
            count=1,
        )

    # 2. declare_parameter 增加 left/right
    if 'declare_parameter<double>("package_scan_left_deg"' not in s:
        s = s.replace(
            'declare_parameter<double>("package_scan_range_deg",25.0);',
            'declare_parameter<double>("package_scan_range_deg",25.0);'
            f'declare_parameter<double>("package_scan_left_deg",{LEFT_DEG});'
            f'declare_parameter<double>("package_scan_right_deg",{RIGHT_DEG});',
            1,
        )

    # 3. get_parameter 读取 left/right
    if "ctx_->package_scan_left_deg=get_parameter" not in s:
        s = s.replace(
            'ctx_->package_scan_range_deg=get_parameter("package_scan_range_deg").as_double();',
            'ctx_->package_scan_range_deg=get_parameter("package_scan_range_deg").as_double();'
            'ctx_->package_scan_left_deg=get_parameter("package_scan_left_deg").as_double();'
            'ctx_->package_scan_right_deg=get_parameter("package_scan_right_deg").as_double();',
            1,
        )

    # 4. 整体替换 ScanTargetPackageOnShelf 类
    start, end = replace_between(
        s,
        "class ScanTargetPackageOnShelf",
        [
            "class SendPlatformTarget",
            "class WaitPickupDone",
            "class PublishTaskDone",
            "class MissionNode",
        ],
    )

    s = s[:start] + NEW_CLASS + s[end:]

    if s == old:
        print("[NO CHANGE] cpp")
        return False

    backup(CPP,"asymmetric_scan_class")
    CPP.write_text(s)
    print(f"[OK] patched cpp: {CPP}")
    return True


def patch_yaml_file(p: Path):
    if not p.exists():
        print(f"[SKIP] missing yaml: {p}")
        return False

    data = yaml.safe_load(p.read_text(errors="ignore"))
    if not isinstance(data,dict):
        print(f"[SKIP] not yaml dict: {p}")
        return False

    changed=False

    def walk(obj):
        nonlocal changed
        if isinstance(obj,dict):
            if "ros__parameters" in obj and isinstance(obj["ros__parameters"],dict):
                params=obj["ros__parameters"]

                if (
                    "package_scan_range_deg" in params
                    or "package_scan_step_deg" in params
                    or "bt_xml" in params
                    or "task_status_topic" in params
                ):
                    desired={
                        "package_scan_range_deg": max(LEFT_DEG,RIGHT_DEG),
                        "package_scan_left_deg": LEFT_DEG,
                        "package_scan_right_deg": RIGHT_DEG,
                        "package_scan_step_deg": STEP_DEG,
                        "package_scan_wait_sec": WAIT_SEC,
                        "package_scan_timeout_sec": TIMEOUT_SEC,
                    }

                    for k,v in desired.items():
                        if params.get(k)!=v:
                            params[k]=v
                            changed=True

            for v in obj.values():
                walk(v)

        elif isinstance(obj,list):
            for item in obj:
                walk(item)

    walk(data)

    if not changed:
        print(f"[NO CHANGE] yaml: {p}")
        return False

    backup(p,"asymmetric_scan_yaml")
    p.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )
    )
    print(f"[OK] patched yaml: {p}")
    return True


def main():
    cpp_changed=patch_cpp()

    yaml_count=0
    for p in YAML_FILES:
        if patch_yaml_file(p):
            yaml_count+=1

    print("")
    print("[DONE] asymmetric scan applied")
    print(f"  center: OAK aligned shelf front direction")
    print(f"  left_deg:  {LEFT_DEG}")
    print(f"  right_deg: {RIGHT_DEG}")
    print(f"  step_deg:  {STEP_DEG}")
    print(f"  wait_sec:  {WAIT_SEC}")
    print(f"  timeout:   {TIMEOUT_SEC}")
    print(f"  cpp_changed={cpp_changed}, yaml_changed={yaml_count}")


if __name__=="__main__":
    main()
