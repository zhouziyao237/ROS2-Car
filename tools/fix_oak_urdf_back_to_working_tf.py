#!/usr/bin/env python3
from pathlib import Path
import time
import shutil
import re

URDF = Path("/host-home/src/Diverse/robot2/urdf/robot2.urdf")
STAMP = time.strftime("%Y%m%d_%H%M%S")

if not URDF.exists():
    raise SystemExit(f"[ERROR] not found: {URDF}")

s = URDF.read_text(errors="ignore")
bak = URDF.with_suffix(URDF.suffix + f".bak_oak_tf_restore_{STAMP}")
shutil.copy2(URDF, bak)
print(f"[BACKUP] {URDF} -> {bak}")

# Remove old OAK application block if it exists.
s = re.sub(
    r"\n\s*<!--\s*OAK application TF frames.*?<!--\s*/OAK application TF frames\s*-->\s*\n",
    "\n",
    s,
    flags=re.S,
)

# Also remove duplicated standalone oak_camera_link blocks if previously inserted.
s = re.sub(
    r"\n\s*<link name=\"oak_camera_link\"\s*/>\s*"
    r"<joint name=\"oak_camera_link_joint\" type=\"fixed\">.*?</joint>\s*"
    r"<link name=\"oak_camera_forward_link\"\s*/>\s*"
    r"<joint name=\"oak_camera_forward_joint\" type=\"fixed\">.*?</joint>\s*",
    "\n",
    s,
    flags=re.S,
)

block = r'''
  <!-- OAK application TF frames.
       Restore to the previously working convention:
       oak_camera_link +X is used as OAK forward direction.
       oak_camera_forward_link is 1m in front of oak_camera_link.
  -->
  <link name="oak_camera_link" />
  <joint name="oak_camera_link_joint" type="fixed">
    <parent link="oak-cam-Link" />
    <child link="oak_camera_link" />
    <origin xyz="0 0 0" rpy="0 0 1.57079632679489" />
  </joint>

  <link name="oak_camera_forward_link" />
  <joint name="oak_camera_forward_joint" type="fixed">
    <parent link="oak_camera_link" />
    <child link="oak_camera_forward_link" />
    <origin xyz="1.0 0.0 0.0" rpy="0 0 0" />
  </joint>
  <!-- /OAK application TF frames -->
'''

m = re.search(r"(<joint\s+name=\"oak-cam-Joint\"\s+type=\"fixed\">.*?</joint>)", s, flags=re.S)
if not m:
    raise SystemExit("[ERROR] cannot find oak-cam-Joint block")

s = s[:m.end()] + block + s[m.end():]
URDF.write_text(s)
print("[OK] restored OAK URDF TF to previous working version")
