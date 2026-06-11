#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time
import re
import subprocess
import yaml

STAMP = time.strftime("%Y%m%d_%H%M%S")

# ============================================================
# 我们自己的机器人参数，不照搬哨兵车体参数
# ============================================================
FOOTPRINT = "[[0.35, 0.35], [0.35, -0.35], [-0.35, -0.35], [-0.35, 0.35]]"
ROBOT_RADIUS = 0.35
INFLATION_RADIUS = 0.60
COST_SCALING_FACTOR = 3.0

# 参考 RoboMaster 哨兵 recovery 思路，但按货架车缩小动作幅度
BACKUP_DIST = "0.12"
BACKUP_SPEED = "0.05"
WAIT_DURATION = "1"

# 保持你现在确认正确的底盘方向映射
KEEP_SWAP_XY = "true"
KEEP_INVERT_X = "true"
KEEP_INVERT_Y = "false"
KEEP_INVERT_Z = "true"

# ============================================================
# 路径
# ============================================================
ROOT = Path("/host-home")

ROBOT2_BRINGUP_SRC = ROOT / "src/Navigation/robot2_bringup"
ROBOT2_BRINGUP_INS = ROOT / "install/robot2_bringup/share/robot2_bringup"

TASK_SRC = ROOT / "src/Navigation/robot_logistics_task"
TASK_INS = ROOT / "install/robot_logistics_task/share/robot_logistics_task"

TASK_CPP_SRC = TASK_SRC / "src/mission_btcpp_node.cpp"
TASK_CPP_INS = ROOT / "install/robot_logistics_task/src/mission_btcpp_node.cpp"

MISSION_BT_SRC_DIR = TASK_SRC / "behavior_tree"
MISSION_BT_INS_DIR = TASK_INS / "behavior_tree"

SCAN_ONLY_BT_SRC = MISSION_BT_SRC_DIR / "pickup_task_nav_oak_scan_only.xml"
SCAN_ONLY_BT_INS = MISSION_BT_INS_DIR / "pickup_task_nav_oak_scan_only.xml"

DEFAULT_TO_POSE = Path("/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml")
DEFAULT_THROUGH_POSES = Path("/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml")

NAV_BT_SRC_DIR = ROBOT2_BRINGUP_SRC / "config"
NAV_BT_INS_DIR = ROBOT2_BRINGUP_INS / "config"

NAV_TO_POSE_BT_SRC = NAV_BT_SRC_DIR / "navigate_to_pose_pickup_sentry_style.xml"
NAV_TO_POSE_BT_INS = NAV_BT_INS_DIR / "navigate_to_pose_pickup_sentry_style.xml"

NAV_THROUGH_BT_SRC = NAV_BT_SRC_DIR / "navigate_through_poses_pickup_sentry_style.xml"
NAV_THROUGH_BT_INS = NAV_BT_INS_DIR / "navigate_through_poses_pickup_sentry_style.xml"

OAK_SEARCH_YAMLS = [
    ROOT / "src/target_detection/oak_package_search/config/oak_package_search.yaml",
    ROOT / "install/oak_package_search/share/oak_package_search/config/oak_package_search.yaml",
]

BRINGUP_YAMLS = [
    ROBOT2_BRINGUP_SRC / "config/bringup_modes.yaml",
    ROBOT2_BRINGUP_INS / "config/bringup_modes.yaml",
]

MISSION_YAMLS = [
    TASK_SRC / "config/mission_task.yaml",
    TASK_INS / "config/mission_task.yaml",
]


# ============================================================
# 工具函数
# ============================================================
def backup(p: Path):
    if p.exists():
        b = p.with_suffix(p.suffix + f".bak_final_scan_only_{STAMP}")
        shutil.copy2(p, b)
        print(f"[BAK] {p} -> {b}")
        return b
    return None


def is_backup_file(p: Path) -> bool:
    n = p.name
    return (
        ".bak" in n
        or ".broken" in n
        or n.endswith("~")
        or ".before_manual_rollback_" in n
    )


def yaml_ok(p: Path) -> bool:
    code = f"""
import yaml
with open({str(p)!r}, 'r', encoding='utf-8') as f:
    yaml.safe_load(f)
"""
    r = subprocess.run(
        ["python3", "-c", code],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if r.returncode != 0:
        print(f"[YAML FAIL] {p}")
        print(r.stderr)
        return False
    return True


def write_text_with_backup(p: Path, text: str):
    p.parent.mkdir(parents=True, exist_ok=True)
    backup(p)
    p.write_text(text)
    print(f"[OK] wrote {p}")


def load_yaml(p: Path):
    if not p.exists():
        return None
    try:
        return yaml.safe_load(p.read_text(errors="ignore"))
    except Exception:
        return None


def dump_yaml(p: Path, data, bak):
    p.write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )
    )
    if not yaml_ok(p):
        if bak:
            shutil.copy2(bak, p)
            print(f"[RESTORE] invalid YAML restored: {p}")
        return False
    return True


# ============================================================
# 0. 修复可能残留的 clear_goal_pub 编译问题
# ============================================================
def patch_clear_goal_compile_guard():
    p = TASK_CPP_SRC
    if not p.exists():
        print(f"[SKIP] no mission cpp: {p}")
        return

    s = p.read_text(errors="ignore")
    old = s

    # 如果代码里用了 std_msgs::msg::Empty，但缺 include，补 include。
    if "std_msgs::msg::Empty" in s and "#include <std_msgs/msg/empty.hpp>" not in s:
        if "#include <std_msgs/msg/string.hpp>" in s:
            s = s.replace(
                "#include <std_msgs/msg/string.hpp>",
                "#include <std_msgs/msg/string.hpp>\n#include <std_msgs/msg/empty.hpp>",
                1,
            )
        else:
            s = "#include <std_msgs/msg/empty.hpp>\n" + s

    # 如果函数/初始化里用了 clear_goal_pub，但 SharedContext 没有成员，补成员。
    if "clear_goal_pub" in s and "Publisher<std_msgs::msg::Empty>::SharedPtr clear_goal_pub" not in s:
        candidates = [
            "package_search_request_pub;",
            "control_mode_pub;",
            "explore_pub;",
        ]
        inserted = False
        for key in candidates:
            if key in s:
                s = s.replace(
                    key,
                    key + "rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_goal_pub;",
                    1,
                )
                inserted = True
                break
        if not inserted:
            print("[WARN] clear_goal_pub used but could not auto insert SharedContext member")

    if s != old:
        backup(p)
        p.write_text(s)
        print(f"[OK] patched clear_goal compile guard: {p}")
    else:
        print("[NO CHANGE] clear_goal compile guard")


# ============================================================
# 1. 生成 Nav2 sentry-style no-spin recovery BT
# ============================================================
def make_nav2_sentry_bt(default_xml: Path, dst: Path):
    if not default_xml.exists():
        print(f"[SKIP] default Nav2 BT not found: {default_xml}")
        return

    s = default_xml.read_text(errors="ignore")

    # 只替换 BT XML 里的 Spin 调用，不删 behavior_server 的 spin 插件。
    # 这样 bt_navigator 不会因为找不到 spin action server 起不来。
    replacement = (
        f'<Sequence name="SmallBackUpAndWait">'
        f'<BackUp backup_dist="{BACKUP_DIST}" backup_speed="{BACKUP_SPEED}"/>'
        f'<Wait wait_duration="{WAIT_DURATION}"/>'
        f'</Sequence>'
    )

    s2 = re.sub(
        r"<Spin\b[^>]*/>",
        replacement,
        s,
        flags=re.DOTALL,
    )

    write_text_with_backup(dst, s2)


# ============================================================
# 2. 生成只做到 OAK 扫码的 mission BT
# ============================================================
def cpp_text() -> str:
    if TASK_CPP_SRC.exists():
        return TASK_CPP_SRC.read_text(errors="ignore")
    return ""


def cpp_has_node(node_name: str) -> bool:
    return node_name in cpp_text()


def make_scan_only_mission_bt():
    """
    目标流程：
    ReceiveShelfGoal
    QueryShelfPose
    NavigateToShelf
    WaitNavArrive
    RequestOakPackageSearchAlign / AlignOakToTargetShelfFront
    WaitAfterOakAlign，可选
    ScanTargetPackageOnShelf
    PublishTaskDone

    注意：
    这里不会放 SendPlatformTarget / WaitPickupDone。
    """

    s = cpp_text()

    if "ScanTargetPackageOnShelf" not in s:
        raise SystemExit(
            "[ERROR] mission_btcpp_node.cpp 中没有 ScanTargetPackageOnShelf，"
            "当前代码不支持 OAK 扫码阶段。"
        )

    if "RequestOakPackageSearchAlign" in s:
        oak_align_node = "RequestOakPackageSearchAlign"
        print("[INFO] mission BT uses RequestOakPackageSearchAlign -> oak_package_search_node")
    elif "AlignOakToTargetShelfFront" in s:
        oak_align_node = "AlignOakToTargetShelfFront"
        print("[INFO] mission BT fallback uses AlignOakToTargetShelfFront")
    else:
        raise SystemExit(
            "[ERROR] mission_btcpp_node.cpp 里既没有 RequestOakPackageSearchAlign，"
            "也没有 AlignOakToTargetShelfFront。"
        )

    nodes = [
        "ReceiveShelfGoal",
        "QueryShelfPose",
        "NavigateToShelf",
        "WaitNavArrive",
        oak_align_node,
    ]

    if "WaitAfterOakAlign" in s:
        nodes.append("WaitAfterOakAlign")

    nodes.append("ScanTargetPackageOnShelf")

    if "PublishTaskDone" in s:
        nodes.append("PublishTaskDone")

    xml_lines = []
    xml_lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    xml_lines.append('<root BTCPP_format="4" main_tree_to_execute="MainTree">')
    xml_lines.append('  <BehaviorTree ID="MainTree">')
    xml_lines.append('    <Sequence name="PickupNavOakScanOnly">')
    for n in nodes:
        xml_lines.append(f"      <{n}/>")
    xml_lines.append('    </Sequence>')
    xml_lines.append('  </BehaviorTree>')
    xml_lines.append('</root>')
    xml = "\n".join(xml_lines) + "\n"

    write_text_with_backup(SCAN_ONLY_BT_SRC, xml)

    MISSION_BT_INS_DIR.mkdir(parents=True, exist_ok=True)
    backup(SCAN_ONLY_BT_INS)
    shutil.copy2(SCAN_ONLY_BT_SRC, SCAN_ONLY_BT_INS)
    print(f"[OK] copied scan-only BT to install: {SCAN_ONLY_BT_INS}")


# ============================================================
# 3. patch mission_task.yaml
# ============================================================
def patch_mission_task_yaml(p: Path) -> bool:
    data = load_yaml(p)
    if not isinstance(data, dict):
        return False

    if "mission_btcpp_node" not in data:
        return False

    params = data["mission_btcpp_node"].setdefault("ros__parameters", {})
    if not isinstance(params, dict):
        return False

    changed = False

    # mission 用 scan-only BT，源路径即可，launch 原本也是用 /host-home/src 路径。
    target_bt = str(SCAN_ONLY_BT_SRC)
    if params.get("bt_xml") != target_bt:
        params["bt_xml"] = target_bt
        changed = True

    safe_values = {
        # 基本 topic
        "cmd_vel_topic": "/cmd_vel",
        "datamatrix_result_json_topic": "/oak/datamatrix_3d_result_json",
        "package_search_request_topic": "/target_detection/package_search/request",
        "package_search_status_topic": "/target_detection/package_search/status",

        # TF frame：你的 URDF 日志里实际有 oak-cam-Link
        "oak_frame": "oak-cam-Link",

        # OAK 对正保守参数，仅 fallback 直接对正节点会用
        "oak_align_yaw_tolerance_deg": 5.0,
        "oak_align_timeout_sec": 8.0,
        "oak_align_kp": 0.8,
        "oak_align_max_wz": 0.10,
        "oak_align_min_wz": 0.04,

        # 扫包裹码阶段小范围扫描
        "package_scan_range_deg": 12.0,
        "package_scan_step_deg": 3.0,
        "package_scan_wait_sec": 0.25,
        "package_scan_timeout_sec": 25.0,
        "package_search_timeout_sec": 25.0,
    }

    for k, v in safe_values.items():
        if params.get(k) != v:
            params[k] = v
            changed = True

    if not changed:
        return False

    bak = backup(p)
    if dump_yaml(p, data, bak):
        print(f"[OK] patched mission yaml: {p}")
        return True
    return False


# ============================================================
# 4. patch bringup_modes.yaml
# ============================================================
def patch_bringup_modes_text(p: Path) -> bool:
    if not p.exists() or is_backup_file(p):
        return False

    s = p.read_text(errors="ignore")
    old = s

    # mission_bt 的 bt_xml 改到 scan-only
    s = re.sub(
        r"(bt_xml:\s*)/host-home/src/Navigation/robot_logistics_task/behavior_tree/[A-Za-z0-9_./-]+\.xml",
        rf"\1{SCAN_ONLY_BT_SRC}",
        s,
    )

    # 保持底盘方向映射
    lines = s.splitlines(True)
    out = []
    in_block = False
    block_indent = None

    for line in lines:
        stripped = line.strip()
        indent = len(line) - len(line.lstrip(" "))

        if re.match(r"^\s*cmd_vel_mux:\s*$", line):
            in_block = True
            block_indent = indent
            out.append(line)
            continue

        if in_block and stripped and indent <= block_indent and not stripped.startswith("#"):
            in_block = False
            block_indent = None

        if in_block:
            line = re.sub(r"(\bswap_xy:\s*)(true|false|True|False)", rf"\g<1>{KEEP_SWAP_XY}", line)
            line = re.sub(r"(\binvert_x:\s*)(true|false|True|False)", rf"\g<1>{KEEP_INVERT_X}", line)
            line = re.sub(r"(\binvert_y:\s*)(true|false|True|False)", rf"\g<1>{KEEP_INVERT_Y}", line)
            line = re.sub(r"(\binvert_z:\s*)(true|false|True|False)", rf"\g<1>{KEEP_INVERT_Z}", line)

        out.append(line)

    s = "".join(out)

    if s == old:
        return False

    bak = backup(p)
    p.write_text(s)

    if not yaml_ok(p):
        if bak:
            shutil.copy2(bak, p)
            print(f"[RESTORE] invalid YAML restored: {p}")
        return False

    print(f"[OK] patched bringup modes: {p}")
    return True


# ============================================================
# 5. patch Nav2 yaml
# ============================================================
def ensure_behavior_plugins(params: dict) -> bool:
    changed = False

    plugins = params.get("behavior_plugins", [])
    if plugins is None:
        plugins = []
    if isinstance(plugins, str):
        plugins = [plugins]
    if not isinstance(plugins, list):
        plugins = []

    # 保留 spin 插件，但新的 BT 不调用 Spin。
    required = ["spin", "backup", "wait"]
    new_plugins = []

    for x in required:
        if x not in new_plugins:
            new_plugins.append(x)

    for x in plugins:
        if x not in new_plugins:
            new_plugins.append(x)

    if params.get("behavior_plugins") != new_plugins:
        params["behavior_plugins"] = new_plugins
        changed = True

    if not isinstance(params.get("spin"), dict) or params["spin"].get("plugin") != "nav2_behaviors/Spin":
        params["spin"] = {"plugin": "nav2_behaviors/Spin"}
        changed = True

    if not isinstance(params.get("backup"), dict) or "plugin" not in params["backup"]:
        params["backup"] = {"plugin": "nav2_behaviors/BackUp"}
        changed = True

    if not isinstance(params.get("wait"), dict) or "plugin" not in params["wait"]:
        params["wait"] = {"plugin": "nav2_behaviors/Wait"}
        changed = True

    return changed


def patch_bt_navigator(params: dict) -> bool:
    changed = False

    target_to_pose = str(NAV_TO_POSE_BT_INS)
    target_through = str(NAV_THROUGH_BT_INS)

    if params.get("default_nav_to_pose_bt_xml") != target_to_pose:
        params["default_nav_to_pose_bt_xml"] = target_to_pose
        changed = True

    if params.get("default_nav_through_poses_bt_xml") != target_through:
        params["default_nav_through_poses_bt_xml"] = target_through
        changed = True

    if "default_bt_xml_filename" in params and params.get("default_bt_xml_filename") != target_to_pose:
        params["default_bt_xml_filename"] = target_to_pose
        changed = True

    return changed


def is_costmap_params(d: dict) -> bool:
    keys = set(d.keys())
    return (
        "global_frame" in keys
        and "robot_base_frame" in keys
        and ("plugins" in keys or "resolution" in keys)
    )


def patch_costmap_recursive(node) -> bool:
    changed = False

    if isinstance(node, dict):
        if is_costmap_params(node):
            if node.get("footprint") != FOOTPRINT:
                node["footprint"] = FOOTPRINT
                changed = True

            # 如果原文件有 robot_radius，统一成 0.35；有 footprint 时 Nav2 主要按 footprint。
            if "robot_radius" in node and node.get("robot_radius") != ROBOT_RADIUS:
                node["robot_radius"] = ROBOT_RADIUS
                changed = True

        plugin = str(node.get("plugin", ""))
        if "InflationLayer" in plugin or "inflation_radius" in node or "cost_scaling_factor" in node:
            if node.get("inflation_radius") != INFLATION_RADIUS:
                node["inflation_radius"] = INFLATION_RADIUS
                changed = True
            if node.get("cost_scaling_factor") != COST_SCALING_FACTOR:
                node["cost_scaling_factor"] = COST_SCALING_FACTOR
                changed = True

        for v in list(node.values()):
            if patch_costmap_recursive(v):
                changed = True

    elif isinstance(node, list):
        for item in node:
            if patch_costmap_recursive(item):
                changed = True

    return changed


def patch_nav2_yaml(p: Path) -> bool:
    if not p.exists() or is_backup_file(p):
        return False

    data = load_yaml(p)
    if not isinstance(data, dict):
        return False

    # 只改真正带 nav2 配置的 yaml，避免误伤普通配置。
    if not any(k in data for k in ["bt_navigator", "behavior_server", "local_costmap", "global_costmap"]):
        return False

    changed = False

    if isinstance(data.get("bt_navigator"), dict):
        params = data["bt_navigator"].setdefault("ros__parameters", {})
        if isinstance(params, dict) and patch_bt_navigator(params):
            changed = True

    if isinstance(data.get("behavior_server"), dict):
        params = data["behavior_server"].setdefault("ros__parameters", {})
        if isinstance(params, dict) and ensure_behavior_plugins(params):
            changed = True

    if patch_costmap_recursive(data):
        changed = True

    if not changed:
        return False

    bak = backup(p)
    if dump_yaml(p, data, bak):
        print(f"[OK] patched Nav2 yaml: {p}")
        return True
    return False


# ============================================================
# 6. patch oak_package_search yaml
# ============================================================
def patch_oak_package_search_yaml(p: Path) -> bool:
    data = load_yaml(p)
    if not isinstance(data, dict):
        return False

    if "oak_package_search_node" not in data:
        return False

    params = data["oak_package_search_node"].setdefault("ros__parameters", {})
    if not isinstance(params, dict):
        return False

    changed = False

    safe_values = {
        "request_topic": "/target_detection/package_search/request",
        "status_topic": "/target_detection/package_search/status",
        "datamatrix_topic": "/oak/datamatrix_3d_result_json",
        "semantic_shelf_topic": "/semantic/current_shelf",
        "cmd_vel_topic": "/cmd_vel",
        "fixed_frame": "map",
        "oak_camera_frame": "oak-cam-Link",

        # 真实执行 OAK 对正/扫描。如果想先 dry-run，可手动改成 false。
        "enable_motion": True,
        "require_tf_for_alignment": True,

        # 对正阶段小角度、低速
        "yaw_control_sign": 1.0,
        "align_yaw_tolerance_deg": 5.0,
        "scan_start_yaw_tolerance_deg": 5.0,
        "yaw_deadband_deg": 2.0,
        "yaw_settle_count_required": 3,
        "kp_wz": 0.8,
        "min_wz": 0.04,
        "max_wz": 0.10,

        # 扫描阶段只小范围扫
        "scan_range_deg": 12.0,
        "scan_speed_wz": 0.06,
        "scan_timeout_sec": 25.0,
        "max_scan_passes": 1,

        # DataMatrix 匹配
        "match_only_in_scan": True,
        "ignore_detection_before_request": True,
        "min_search_time_before_match_sec": 0.5,
        "min_confirm_matches": 2,
        "datamatrix_fresh_timeout_sec": 1.2,

        # 如果当前代码支持 guard 参数，会生效；不支持也不会影响启动。
        "search_already_ok_deg": 8.0,
        "search_start_align_deg": 12.0,
        "same_shelf_skip_sec": 12.0,
        "search_align_only": True,
    }

    for k, v in safe_values.items():
        if params.get(k) != v:
            params[k] = v
            changed = True

    if not changed:
        return False

    bak = backup(p)
    if dump_yaml(p, data, bak):
        print(f"[OK] patched OAK package search yaml: {p}")
        return True
    return False


# ============================================================
# 主程序
# ============================================================
def main():
    print("========== 0. FIX POSSIBLE COMPILE GUARD ==========")
    patch_clear_goal_compile_guard()

    print("")
    print("========== 1. CREATE NAV2 SENTRY-STYLE NO-SPIN BT ==========")
    make_nav2_sentry_bt(DEFAULT_TO_POSE, NAV_TO_POSE_BT_SRC)
    make_nav2_sentry_bt(DEFAULT_THROUGH_POSES, NAV_THROUGH_BT_SRC)

    NAV_BT_INS_DIR.mkdir(parents=True, exist_ok=True)

    if NAV_TO_POSE_BT_SRC.exists():
        backup(NAV_TO_POSE_BT_INS)
        shutil.copy2(NAV_TO_POSE_BT_SRC, NAV_TO_POSE_BT_INS)
        print(f"[OK] copied {NAV_TO_POSE_BT_INS}")

    if NAV_THROUGH_BT_SRC.exists():
        backup(NAV_THROUGH_BT_INS)
        shutil.copy2(NAV_THROUGH_BT_SRC, NAV_THROUGH_BT_INS)
        print(f"[OK] copied {NAV_THROUGH_BT_INS}")

    print("")
    print("========== 2. CREATE MISSION SCAN-ONLY BT ==========")
    make_scan_only_mission_bt()

    print("")
    print("========== 3. PATCH MISSION YAML ==========")
    for p in MISSION_YAMLS:
        patch_mission_task_yaml(p)

    print("")
    print("========== 4. PATCH BRINGUP MODES ==========")
    for p in BRINGUP_YAMLS:
        patch_bringup_modes_text(p)

    print("")
    print("========== 5. PATCH NAV2 YAML ==========")
    changed_nav2 = 0
    for root in [ROOT / "src/Navigation", ROOT / "install"]:
        if not root.exists():
            continue
        for p in list(root.rglob("*.yaml")) + list(root.rglob("*.yml")):
            try:
                if patch_nav2_yaml(p):
                    changed_nav2 += 1
            except Exception as e:
                print(f"[ERR] patch Nav2 yaml failed {p}: {e}")

    print("")
    print("========== 6. PATCH OAK PACKAGE SEARCH YAML ==========")
    for p in OAK_SEARCH_YAMLS:
        try:
            patch_oak_package_search_yaml(p)
        except Exception as e:
            print(f"[ERR] patch OAK yaml failed {p}: {e}")

    print("")
    print("========== DONE ==========")
    print("[DONE] mission BT stops at OAK DataMatrix scan stage.")
    print("[DONE] no SendPlatformTarget / WaitPickupDone in scan-only BT.")
    print("[DONE] Nav2 recovery = ClearCostmap + SmallBackUp + Wait + Retry.")
    print("[DONE] behavior_server keeps spin/backup/wait plugins, but custom BT does not call Spin.")
    print("[DONE] cmd_vel axis kept: swap_xy=true, invert_x=true, invert_y=false, invert_z=true.")
    print("[DONE] costmap footprint=0.70m square, inflation_radius=0.60, cost_scaling_factor=3.0.")
    print("[DONE] OAK package search uses oak-cam-Link and low-speed small-angle parameters.")
    print("")
    print("[NEXT] build:")
    print("cd /host-home && source /opt/ros/humble/setup.bash && source install/setup.bash")
    print("PKGS=$(colcon list --names-only | grep -E '^(robot2_bringup|robot_logistics_task|robot_logistics_semantic|oak_package_search)$' | tr '\\n' ' ')")
    print("colcon build --packages-select $PKGS")
    print("source install/setup.bash")


if __name__ == "__main__":
    main()
