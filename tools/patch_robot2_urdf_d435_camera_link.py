from pathlib import Path
import time
import xml.etree.ElementTree as ET
from collections import Counter

p = Path("/host-home/src/Diverse/robot2/urdf/robot2.urdf")

if not p.exists():
    raise SystemExit(f"[ERROR] URDF not found: {p}")

stamp = time.strftime("%Y%m%d_%H%M%S")
bak = p.with_suffix(p.suffix + f".bak_d435_camera_link_{stamp}")
bak.write_text(p.read_text(errors="ignore"))
print(f"[BACKUP] {p} -> {bak}")

parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
tree = ET.parse(str(p), parser=parser)
root = tree.getroot()

# 这些 RealSense 内部 frame 应该由 D435 / RealSense driver 发布，
# 不要在 URDF 里重复定义，否则容易出现重复 TF 或两棵树。
internal_realsense_links = {
    "camera_color_frame",
    "camera_depth_frame",
    "camera_color_optical_frame",
    "camera_depth_optical_frame",
    "camera_infra1_frame",
    "camera_infra1_optical_frame",
    "camera_infra2_frame",
    "camera_infra2_optical_frame",
}

# camera_link 只保留一个，并作为 d435-cam-Link 的 child。
remove_child_links = set(internal_realsense_links)
remove_child_links.add("camera_link")

# 删除旧的 camera_link 相关 joint，以及 URDF 里可能误加的 RealSense 内部 frame joint。
removed_joints = []
for joint in list(root.findall("joint")):
    child = joint.find("child")
    if child is None:
        continue

    child_link = child.attrib.get("link", "")
    if child_link in remove_child_links:
        removed_joints.append(joint.attrib.get("name", "UNKNOWN"))
        root.remove(joint)

# 删除旧的 camera_link link 以及 URDF 里可能误加的 RealSense 内部 frame link。
removed_links = []
for link in list(root.findall("link")):
    name = link.attrib.get("name", "")
    if name == "camera_link" or name in internal_realsense_links:
        removed_links.append(name)
        root.remove(link)

# 确认 d435-cam-Link 存在。
all_links = {link.attrib.get("name", "") for link in root.findall("link")}
if "d435-cam-Link" not in all_links:
    raise SystemExit("[ERROR] d435-cam-Link not found in URDF. Check your robot2.urdf first.")

# 添加 camera_link，作为 D435 机械安装 link 的兼容相机 frame。
root.append(ET.Comment(
    " D435 compatibility frame: d435-cam-Link is mechanical mount; camera_link is used by RealSense/AprilTag/semantic packages. "
))

camera_link = ET.Element("link", {"name": "camera_link"})
root.append(camera_link)

joint = ET.Element("joint", {
    "name": "d435_camera_link_joint",
    "type": "fixed",
})
ET.SubElement(joint, "origin", {
    "xyz": "0 0 0",
    "rpy": "0 0 0",
})
ET.SubElement(joint, "parent", {
    "link": "d435-cam-Link",
})
ET.SubElement(joint, "child", {
    "link": "camera_link",
})
root.append(joint)

# 验证重复 link / joint。
links = [x.attrib.get("name", "") for x in root.findall("link")]
joints = [x.attrib.get("name", "") for x in root.findall("joint")]

dup_links = [k for k, v in Counter(links).items() if v > 1]
dup_joints = [k for k, v in Counter(joints).items() if v > 1]

if dup_links:
    raise SystemExit(f"[ERROR] duplicated link names after patch: {dup_links}")

if dup_joints:
    raise SystemExit(f"[ERROR] duplicated joint names after patch: {dup_joints}")

# 验证 camera_link parent。
parent_ok = False
for j in root.findall("joint"):
    parent = j.find("parent")
    child = j.find("child")
    if parent is not None and child is not None:
        if parent.attrib.get("link") == "d435-cam-Link" and child.attrib.get("link") == "camera_link":
            parent_ok = True
            break

if not parent_ok:
    raise SystemExit("[ERROR] camera_link is not attached under d435-cam-Link.")

ET.indent(tree, space="  ")
tree.write(str(p), encoding="utf-8", xml_declaration=True)

print("[OK] URDF patched.")
print("")
print("Removed old camera-related links:")
for x in removed_links:
    print(f"  - {x}")

print("")
print("Removed old camera-related joints:")
for x in removed_joints:
    print(f"  - {x}")

print("")
print("Added:")
print("  d435-cam-Link -> camera_link")
print("")
print("[IMPORTANT]")
print("Do not define camera_color_frame / camera_color_optical_frame in URDF.")
print("Let the RealSense driver publish those internal camera frames.")
