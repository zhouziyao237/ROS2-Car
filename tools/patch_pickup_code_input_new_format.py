from pathlib import Path
import time
import re

p = Path("/host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")

if not p.exists():
    raise SystemExit(f"[ERROR] file not found: {p}")

stamp = time.strftime("%Y%m%d_%H%M%S")
bak = p.with_suffix(p.suffix + f".bak_new_pickup_code_format_{stamp}")
s = p.read_text(errors="ignore")
bak.write_text(s)
print(f"[BACKUP] {p} -> {bak}")

# ------------------------------------------------------------
# 1. 更新提示文字：旧 S03-L1-A -> 新 S03-L1-0301
# ------------------------------------------------------------
replace_pairs = {
    "S03-L1-A": "S03-L1-0301",
    "S02-L2-B": "S03-L2-0311",
    "输入示例: S03-L1-A": "输入示例: S03-L1-0301",
    "Input example: S03-L1-A": "Input example: S03-L1-0301",
    "S03-L1-A      发布取件任务": "S03-L1-0301   发布取件任务",
    "S02-L2-B      发布取件任务": "S03-L2-0311   发布取件任务",
}

for old, new in replace_pairs.items():
    if old in s:
        s = s.replace(old, new)
        print(f"[OK] replaced text: {old} -> {new}")

# ------------------------------------------------------------
# 2. 如果帮助文本里没有说明新格式，就补充说明
# ------------------------------------------------------------
if "完整取件码格式" not in s:
    s = s.replace(
        "======== 取件码输入工具 ========",
        "======== 取件码输入工具 ========\\n完整取件码格式：<货架编号>-<层号>-<4位取件码>，例如 S03-L1-0301"
    )
    print("[OK] added full pickup code format description")

# ------------------------------------------------------------
# 3. 确保任务 JSON 里有 package_code
#    你的当前脚本已经能解析 S03-L1-0301，并发布 layer_id / row / col。
#    这里尽量做安全补丁：在出现 layer_id 后补 package_code。
# ------------------------------------------------------------

if '"package_code"' in s:
    print("[SKIP] package_code already exists in publish payload")
else:
    patched = False

    # 常见格式：
    # "layer_id": layer_id,
    # "row": row,
    # "col": package_code,
    pattern = r'("layer_id"\s*:\s*layer_id\s*,\s*)("row"\s*:)'
    if re.search(pattern, s):
        s = re.sub(
            pattern,
            r'\1"package_code": package_code,\n            \2',
            s,
            count=1
        )
        patched = True
        print("[OK] inserted package_code after layer_id")

    # 另一种可能：用 parsed["layer_id"]
    if not patched:
        pattern = r'("layer_id"\s*:\s*parsed\["layer_id"\]\s*,\s*)("row"\s*:)'
        if re.search(pattern, s):
            s = re.sub(
                pattern,
                r'\1"package_code": parsed["package_code"],\n            \2',
                s,
                count=1
            )
            patched = True
            print("[OK] inserted package_code after parsed layer_id")

    # 如果脚本是先构造 dict 再赋值，尝试在 cell_id 后补一行
    if not patched:
        pattern = r'("cell_id"\s*:\s*pickup_code\s*,\s*)("layer_id"\s*:)'
        if re.search(pattern, s):
            s = re.sub(
                pattern,
                r'\1\2',
                s,
                count=1
            )
            patched = True
            print("[WARN] found cell_id/layer_id pattern but package_code insertion may need manual check")

    if not patched:
        print("[WARN] Could not automatically insert package_code.")
        print("[WARN] The help text was updated, but please check publish payload manually.")

p.write_text(s)

print("")
print("[DONE] pickup_code_input.py updated for new full pickup code format.")
print("Expected input example: S03-L1-0301")
print("Expected published fields:")
print("  pickup_code  = S03-L1-0301")
print("  shelf_id     = S03")
print("  layer_id     = L1")
print("  package_code = 0301")
print("  cell_id      = S03-L1-0301")
print("  row          = 1")
print("  col          = 0301")
