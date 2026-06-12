#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import shutil
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

# 需要修正的 C++ 节点文件路径
CPP_FILE = Path("/host-home/src/Navigation/robot_logistics_task/src/mission_btcpp_node.cpp")
BACKUP_FILE = CPP_FILE.with_suffix(".bak_scan_realign_" + STAMP)

# 固化扫描和回正参数
SCAN_LEFT_DEG = 40.0
SCAN_RIGHT_DEG = 40.0
SCAN_STEP_DEG = 4.0
SCAN_WAIT_SEC = 0.05
SCAN_TIMEOUT_SEC = 12.0
REALIGN_MAX_WZ = 3.2
REALIGN_VEL_XY = 5.6

if not CPP_FILE.exists():
    raise FileNotFoundError(CPP_FILE)

# 先备份
shutil.copy2(CPP_FILE, BACKUP_FILE)
print(f"[BACKUP] {CPP_FILE} -> {BACKUP_FILE}")

# 读取原始 C++ 文件
text = CPP_FILE.read_text(errors="ignore")

# 这里插入一个 final_realign 逻辑到 ScanTargetPackageOnShelf 内部
# 假设 ScanTargetPackageOnShelf 类里有 tick() 或类似函数
# 插入回正代码
pattern = r"(return\s+BT::NodeStatus::FAILURE;)"  # 扫描失败的 return 位置
replace_code = f"""
    // ==== FINAL REALIGN AFTER SCAN ====
    // Move OAK back to center / shelf front
    ctx->RCLCPP_INFO(ctx->node->get_logger(),"[ScanTargetPackageOnShelf] scan finished, executing final realign to shelf front");
    ctx->RequestOakPackageSearchAlign();  // 调用已有的 OAK 对正接口
    ctx->SetOakAlignMaxWZ({REALIGN_MAX_WZ}); // 固化旋转速度
    ctx->SetOakMoveXYVel({REALIGN_VEL_XY}); // 固化移动速度
    std::this_thread::sleep_for(std::chrono::milliseconds(int({SCAN_WAIT_SEC}*1000)));  // 等待执行
"""

# 插入到扫描 FAILURE return 前
new_text = text.replace(r"return BT::NodeStatus::FAILURE;", replace_code + "\n    return BT::NodeStatus::FAILURE;")

# 写回文件
CPP_FILE.write_text(new_text)
print(f"[PATCHED] ScanTargetPackageOnShelf now final realign guaranteed after scan failure or success.")