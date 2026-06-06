from pathlib import Path
import re
import time

p = Path("/host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")

if not p.exists():
    raise SystemExit(f"[ERROR] file not found: {p}")

stamp = time.strftime("%Y%m%d_%H%M%S")
bak = p.with_suffix(p.suffix + f".bak_fix_package_code_{stamp}")
s = p.read_text(errors="ignore")
bak.write_text(s)
print(f"[BACKUP] {p} -> {bak}")

# 确保有 import re
if "import re" not in s:
    if "import json" in s:
        s = s.replace("import json", "import json\nimport re", 1)
        print("[OK] added import re")
    else:
        s = "import re\n" + s
        print("[OK] inserted import re at file top")

new_func = r'''    def parse_pickup_code(self, text: str):
        """
        Parse full pickup code from the parcel label DataMatrix format.

        Expected format:
            <shelf_id>-<layer_id>-<4-char package_code>

        Example:
            S03-L1-0301

        Parsed result:
            pickup_code  = S03-L1-0301
            shelf_id     = S03
            layer_id     = L1
            package_code = 0301
            cell_id      = S03-L1-0301
            row          = 1
            col          = 0301
        """
        raw = str(text).strip().upper()

        # Normalize common separators.
        norm = raw.replace("_", "-").replace("/", "-")
        norm = re.sub(r"\s+", "-", norm)
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

pattern = r"    def parse_pickup_code\(self, text.*?\):\n(?:        .*\n)+?(?=    def publish_pickup_task\(self, text\):)"

m = re.search(pattern, s)
if not m:
    raise SystemExit("[ERROR] 找不到 parse_pickup_code 函数，请先 grep 查看文件结构。")

s = s[:m.start()] + new_func + s[m.end():]

p.write_text(s)

print("[OK] parse_pickup_code fixed.")
print("[DONE]")
print("")
print("Now test with:")
print("  python /host-home/src/Navigation/robot2_bringup/scripts/pickup_code_input.py")
print("  input: S03-L1-0301")
