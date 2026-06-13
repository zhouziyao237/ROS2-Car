from pathlib import Path
import time

p = Path("/host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")

if not p.exists():
    raise SystemExit(f"[ERROR] file not found: {p}")

stamp = time.strftime("%Y%m%d_%H%M%S")
s = p.read_text(errors="ignore")
bak = p.with_suffix(p.suffix + f".bak_fix_package_code_v2_{stamp}")
bak.write_text(s)
print(f"[BACKUP] {p} -> {bak}")

# 确保有 import re
if "import re" not in s:
    if "import json" in s:
        s = s.replace("import json", "import json\nimport re", 1)
        print("[OK] added import re after import json")
    else:
        s = "import re\n" + s
        print("[OK] added import re at file top")

lines = s.splitlines(keepends=True)

start = None
for i, line in enumerate(lines):
    if line.startswith("    def parse_pickup_code("):
        start = i
        break

if start is None:
    raise SystemExit("[ERROR] 找不到以 4 个空格开头的 def parse_pickup_code(...)")

end = None
for j in range(start + 1, len(lines)):
    if lines[j].startswith("    def ") and not lines[j].startswith("    def parse_pickup_code("):
        end = j
        break

if end is None:
    raise SystemExit("[ERROR] 找到了 parse_pickup_code，但找不到下一个函数边界")

new_func = '''    def parse_pickup_code(self, text: str):
        """
        解析完整取件码。
        格式：<货架编号>-<层号>-<4位取件码>
        示例：S03-L1-0301
        """
        raw = str(text).strip().upper()

        # 兼容空格、下划线、斜杠等输入
        norm = raw.replace("_", "-").replace("/", "-")
        norm = re.sub(r"\\s+", "-", norm)
        norm = re.sub(r"[^A-Z0-9-]+", "-", norm)
        norm = re.sub(r"-+", "-", norm).strip("-")

        m = re.search(r"([A-Z0-9]{1,4})-(L[0-9]{1,3})-([A-Z0-9]{4})", norm)
        if not m:
            raise ValueError(
                f"取件码格式错误：{text}，应为类似 S03-L1-0301 的完整取件码"
            )

        shelf_id = m.group(1)
        layer_id = m.group(2)
        package_code = m.group(3)
        pickup_code = f"{shelf_id}-{layer_id}-{package_code}"

        try:
            row = int(layer_id[1:])
        except Exception:
            row = 0

        return {
            "task_type": "pickup",
            "pickup_code": pickup_code,
            "shelf_id": shelf_id,
            "cell_id": pickup_code,
            "layer_id": layer_id,
            "package_code": package_code,

            # 兼容旧字段：
            # row 表示层号数字，col 暂时放 4 位包裹码
            "row": row,
            "col": package_code,
        }

'''

lines = lines[:start] + [new_func] + lines[end:]
new_s = "".join(lines)

p.write_text(new_s)
print("[OK] parse_pickup_code replaced by full-code parser.")
print("[DONE]")
