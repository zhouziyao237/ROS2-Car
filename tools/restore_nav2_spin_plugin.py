#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import re
import shutil
import subprocess
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")

FILES = []
for root in [
    Path("/host-home/src/Navigation"),
    Path("/host-home/install"),
]:
    if root.exists():
        FILES += list(root.rglob("*.yaml"))
        FILES += list(root.rglob("*.yml"))

def is_backup(p: Path) -> bool:
    return ".bak" in p.name or ".before_manual_rollback_" in p.name

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

def patch_file(p: Path) -> bool:
    if is_backup(p):
        return False

    s = p.read_text(errors="ignore")
    old = s

    if "behavior_server" not in s and "nav2_behaviors" not in s and "behavior_plugins" not in s:
        return False

    # 把 behavior_plugins inline 恢复为包含 spin
    s = re.sub(
        r'behavior_plugins:\s*\[[^\]]*\]',
        'behavior_plugins: ["spin", "backup", "wait"]',
        s,
    )

    # 如果是多行 behavior_plugins 但缺少 spin，就在 behavior_plugins: 后补上 - spin
    lines = s.splitlines(True)
    out = []
    i = 0
    while i < len(lines):
        line = lines[i]
        out.append(line)

        if re.match(r"^\s*behavior_plugins:\s*$", line):
            indent = len(line) - len(line.lstrip(" "))
            block = []
            j = i + 1
            while j < len(lines):
                nxt = lines[j]
                stripped = nxt.strip()
                nxt_indent = len(nxt) - len(nxt.lstrip(" "))
                if stripped and nxt_indent <= indent and not stripped.startswith("#"):
                    break
                block.append(nxt)
                j += 1

            if not any(re.match(r"^\s*-\s*spin\s*$", b) for b in block):
                out.append(" " * (indent + 2) + "- spin\n")

            out.extend(block)
            i = j
            continue

        i += 1

    s = "".join(out)

    # 如果之前把 Spin plugin 改成 Wait，恢复回来。
    # 只恢复名字为 spin 的 block 附近比较困难，这里直接确保有 spin block。
    if "spin:" not in s and "nav2_behaviors/Spin" not in s:
        # 在 behavior_server ros__parameters 下补一个 spin block。
        # 这段只对常见 Nav2 YAML 有效。
        pattern = r"(behavior_server:\s*\n\s*ros__parameters:\s*\n)"
        m = re.search(pattern, s)
        if m:
            insert = (
                m.group(1)
                + "    spin:\n"
                + '      plugin: "nav2_behaviors/Spin"\n'
            )
            s = s[:m.start()] + insert + s[m.end():]

    # 把任何 nav2_behaviors/Wait 恢复成 Spin 只针对 spin block 不好判断，
    # 所以如果文件里有 spin: block 但 plugin 是 Wait，则改回 Spin。
    s = re.sub(
        r"(spin:\s*\n\s*plugin:\s*)[\"']?nav2_behaviors/Wait[\"']?",
        r'\1"nav2_behaviors/Spin"',
        s,
    )

    if s == old:
        return False

    bak = p.with_suffix(p.suffix + f".bak_restore_spin_{STAMP}")
    shutil.copy2(p, bak)
    p.write_text(s)

    if not yaml_ok(p):
        shutil.copy2(bak, p)
        print(f"[RESTORE] invalid YAML restored: {p}")
        return False

    print(f"[OK] restored spin in {p}")
    print(f"     backup: {bak}")
    return True

changed = 0
for p in FILES:
    if patch_file(p):
        changed += 1

print(f"[DONE] changed files: {changed}")
