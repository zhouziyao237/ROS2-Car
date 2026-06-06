from pathlib import Path
import time
import re

ROOT = Path("/host-home/src/Navigation/robot_logistics_task")
CPP = ROOT / "src" / "mission_btcpp_node.cpp"
CMAKE = ROOT / "CMakeLists.txt"
SCRIPTS = ROOT / "scripts"
PUBLISH_TOOL = SCRIPTS / "send_pickup_task.py"

stamp = time.strftime("%Y%m%d_%H%M%S")


def backup(path: Path):
    if path.exists():
        bak = path.with_suffix(path.suffix + f".bak_pickup_code_parser_{stamp}")
        bak.write_text(path.read_text(errors="ignore"))
        print(f"[BACKUP] {path} -> {bak}")


if not CPP.exists():
    raise SystemExit(f"[ERROR] mission_btcpp_node.cpp not found: {CPP}")

if not CMAKE.exists():
    raise SystemExit(f"[ERROR] CMakeLists.txt not found: {CMAKE}")

SCRIPTS.mkdir(parents=True, exist_ok=True)

backup(CPP)
backup(CMAKE)
if PUBLISH_TOOL.exists():
    backup(PUBLISH_TOOL)

s = CPP.read_text(errors="ignore")

# ============================================================
# 1. Extend TaskData
# ============================================================
old_struct = 'struct TaskData{bool active=false;std::string raw,task_type="pickup",shelf_id,cell_id,col,pickup_code;int row=0;bool command_has_pose=false;Pose2D target_pose;};'
new_struct = 'struct TaskData{bool active=false;std::string raw,task_type="pickup",shelf_id,cell_id,col,pickup_code,layer_id,package_code;int row=0;bool command_has_pose=false;Pose2D target_pose;};'

if old_struct in s:
    s = s.replace(old_struct, new_struct)
    print("[OK] Extended TaskData with layer_id and package_code")
elif "layer_id,package_code" in s:
    print("[SKIP] TaskData already contains layer_id and package_code")
else:
    print("[WARN] Could not find exact TaskData struct. Please check manually later.")


# ============================================================
# 2. Add parser helper functions
# ============================================================
helper_marker = "parse_pickup_full_code_anywhere"

helper_code = r'''
// ============================================================
// Pickup full-code parser
// Label DataMatrix format:
//   <shelf_id>-<layer_id>-<4-char package_code>
// Example:
//   S03-L1-0301
// ============================================================
struct ParsedPickupFullCode{
    bool valid=false;
    std::string pickup_full;
    std::string shelf_id;
    std::string layer_id;
    std::string package_code;
    int layer_index=0;
};

static std::string normalize_pickup_full_code_text(std::string s){
    std::string out;
    bool last_dash=false;

    for(unsigned char ch: s){
        if(std::isalnum(ch)){
            out.push_back((char)std::toupper(ch));
            last_dash=false;
        }else{
            // Treat spaces, underscores, slashes, JSON punctuation and newlines as separators.
            if(!out.empty() && !last_dash){
                out.push_back('-');
                last_dash=true;
            }
        }
    }

    while(!out.empty() && out.front()=='-') out.erase(out.begin());
    while(!out.empty() && out.back()=='-') out.pop_back();

    std::string collapsed;
    last_dash=false;
    for(char c: out){
        if(c=='-'){
            if(!last_dash) collapsed.push_back(c);
            last_dash=true;
        }else{
            collapsed.push_back(c);
            last_dash=false;
        }
    }

    return collapsed;
}

static bool token_is_alnum_len(const std::string& s, size_t min_len, size_t max_len){
    if(s.size()<min_len || s.size()>max_len) return false;
    for(unsigned char c: s){
        if(!std::isalnum(c)) return false;
    }
    return true;
}

static bool token_is_layer_id(const std::string& s){
    if(s.size()<2 || s.size()>4) return false;
    if(s[0] != 'L') return false;
    for(size_t i=1;i<s.size();++i){
        if(!std::isdigit((unsigned char)s[i])) return false;
    }
    return true;
}

static std::vector<std::string> split_dash_tokens(const std::string& s){
    std::vector<std::string> tokens;
    std::string cur;
    for(char c: s){
        if(c=='-'){
            if(!cur.empty()){
                tokens.push_back(cur);
                cur.clear();
            }
        }else{
            cur.push_back(c);
        }
    }
    if(!cur.empty()) tokens.push_back(cur);
    return tokens;
}

static ParsedPickupFullCode parse_pickup_full_code_anywhere(const std::string& text){
    ParsedPickupFullCode out;
    auto norm = normalize_pickup_full_code_text(text);
    auto tokens = split_dash_tokens(norm);

    // Find any triplet:
    //   shelf_id      : 1-4 alnum, e.g. S03
    //   layer_id      : L + digits, e.g. L1
    //   package_code  : exactly 4 alnum, e.g. 0301
    for(size_t i=0; i+2<tokens.size(); ++i){
        const auto& shelf = tokens[i];
        const auto& layer = tokens[i+1];
        const auto& pkg = tokens[i+2];

        if(!token_is_alnum_len(shelf, 1, 4)) continue;
        if(!token_is_layer_id(layer)) continue;
        if(!token_is_alnum_len(pkg, 4, 4)) continue;

        out.valid=true;
        out.shelf_id=shelf;
        out.layer_id=layer;
        out.package_code=pkg;
        out.pickup_full=shelf + "-" + layer + "-" + pkg;

        try{
            out.layer_index = std::stoi(layer.substr(1));
        }catch(...){
            out.layer_index = 0;
        }

        return out;
    }

    return out;
}
'''

if helper_marker not in s:
    insert_after = 'static bool get_bool_json(const std::string& json,const std::string& key,bool& out){auto p=json.find("\\""+key+"\\"");if(p==std::string::npos)return false;p=json.find(":",p);if(p==std::string::npos)return false;p++;while(p<json.size()&&std::isspace((unsigned char)json[p]))p++;if(json.compare(p,4,"true")==0){out=true;return true;}if(json.compare(p,5,"false")==0){out=false;return true;}return false;}'
    if insert_after not in s:
        raise SystemExit("[ERROR] Could not find get_bool_json insertion point.")
    s = s.replace(insert_after, insert_after + helper_code)
    print("[OK] Added pickup full-code parser helpers")
else:
    print("[SKIP] Parser helpers already exist")


# ============================================================
# 3. Replace ReceiveShelfGoal parsing block
# ============================================================
old_block = r'''    get_string_json(m->data,"task_type",t.task_type);
    get_string_json(m->data,"shelf_id",t.shelf_id);
    get_string_json(m->data,"cell_id",t.cell_id);
    get_string_json(m->data,"target_cell_id",t.cell_id);
    get_string_json(m->data,"pickup_code",t.pickup_code);
    if(t.pickup_code.empty())t.pickup_code=t.cell_id;
    if(t.cell_id.empty()&&!t.pickup_code.empty())t.cell_id=t.pickup_code;
    get_string_json(m->data,"col",t.col);

    if(t.shelf_id.empty()){
        std::string target_shelf;
        get_string_json(m->data,"target_shelf_id",target_shelf);
        t.shelf_id=target_shelf;
    }

    if(t.shelf_id.empty()&&!t.pickup_code.empty()){
        auto d=t.pickup_code.find("-");
        t.shelf_id=d==std::string::npos?t.pickup_code:t.pickup_code.substr(0,d);
    }
    if(t.shelf_id.empty()){
        auto d=m->data.find("-");
        t.shelf_id=d==std::string::npos?m->data:m->data.substr(0,d);
    }
'''

new_block = r'''    get_string_json(m->data,"task_type",t.task_type);

    std::string json_shelf_id;
    std::string json_target_shelf_id;
    std::string json_cell_id;
    std::string json_target_cell_id;
    std::string json_pickup_code;
    std::string json_layer_id;
    std::string json_package_code;
    std::string json_col;

    get_string_json(m->data,"shelf_id",json_shelf_id);
    get_string_json(m->data,"target_shelf_id",json_target_shelf_id);
    get_string_json(m->data,"cell_id",json_cell_id);
    get_string_json(m->data,"target_cell_id",json_target_cell_id);
    get_string_json(m->data,"pickup_code",json_pickup_code);
    get_string_json(m->data,"layer_id",json_layer_id);
    get_string_json(m->data,"package_code",json_package_code);
    get_string_json(m->data,"col",json_col);

    // Main input rule:
    //   pickup_code should be the complete DataMatrix content, e.g. S03-L1-0301.
    // Support both JSON input and raw string input.
    std::string code_source;
    if(!json_pickup_code.empty()) code_source=json_pickup_code;
    else if(!json_cell_id.empty()) code_source=json_cell_id;
    else if(!json_target_cell_id.empty()) code_source=json_target_cell_id;
    else code_source=m->data;

    auto parsed = parse_pickup_full_code_anywhere(code_source);

    if(parsed.valid){
        // The full DataMatrix code is the source of truth.
        // It defines shelf_id, layer_id and package_code.
        if(!json_shelf_id.empty() && normalize_pickup_full_code_text(json_shelf_id) != parsed.shelf_id){
            RCLCPP_WARN(get_logger(),
                "shelf_id in JSON [%s] does not match pickup_code shelf [%s]. Using pickup_code parsed shelf.",
                json_shelf_id.c_str(), parsed.shelf_id.c_str());
        }

        t.pickup_code = parsed.pickup_full;
        t.cell_id = parsed.pickup_full;
        t.shelf_id = parsed.shelf_id;
        t.layer_id = parsed.layer_id;
        t.package_code = parsed.package_code;

        // Compatibility with old row/col fields:
        // row = layer index, col = 4-char package code.
        t.row = parsed.layer_index;
        t.col = parsed.package_code;
    }else{
        // Backward-compatible fallback for old task format.
        t.shelf_id = !json_shelf_id.empty() ? json_shelf_id : json_target_shelf_id;
        t.cell_id = !json_cell_id.empty() ? json_cell_id : json_target_cell_id;
        t.pickup_code = json_pickup_code;
        if(t.pickup_code.empty()) t.pickup_code=t.cell_id;
        if(t.cell_id.empty() && !t.pickup_code.empty()) t.cell_id=t.pickup_code;
        t.col = json_col;
        t.layer_id = json_layer_id;
        t.package_code = json_package_code;

        if(t.shelf_id.empty() && !t.pickup_code.empty()){
            auto d=t.pickup_code.find("-");
            t.shelf_id=d==std::string::npos?t.pickup_code:t.pickup_code.substr(0,d);
        }
        if(t.shelf_id.empty()){
            auto d=m->data.find("-");
            t.shelf_id=d==std::string::npos?m->data:m->data.substr(0,d);
        }
    }
'''

if old_block in s:
    s = s.replace(old_block, new_block)
    print("[OK] Replaced task parsing block")
elif "parse_pickup_full_code_anywhere(code_source)" in s:
    print("[SKIP] Task parsing block already patched")
else:
    raise SystemExit("[ERROR] Could not find old task parsing block. File may have changed.")


# ============================================================
# 4. Make row parsing old-format only
# ============================================================
old_row_block = r'''    double row=0.0;
    if(get_number_json(m->data,"row",row)){
        t.row=(int)row;
    }
'''

new_row_block = r'''    double row=0.0;
    if(!parsed.valid && get_number_json(m->data,"row",row)){
        t.row=(int)row;
    }
'''

if old_row_block in s:
    s = s.replace(old_row_block, new_row_block)
    print("[OK] Row field now only overrides old-format tasks")
elif "if(!parsed.valid && get_number_json(m->data,\"row\",row))" in s:
    print("[SKIP] Row parsing already patched")
else:
    print("[WARN] Could not find row parsing block. Please check manually.")


# ============================================================
# 5. Improve log
# ============================================================
old_log = 'RCLCPP_INFO(get_logger(),"Task received: %s",m->data.c_str());'
new_log = 'RCLCPP_INFO(get_logger(),"Task received: raw=%s | parsed pickup_code=%s shelf_id=%s layer_id=%s package_code=%s",m->data.c_str(),t.pickup_code.c_str(),t.shelf_id.c_str(),t.layer_id.c_str(),t.package_code.c_str());'

if old_log in s:
    s = s.replace(old_log, new_log)
    print("[OK] Improved task received log")
elif "parsed pickup_code=" in s:
    print("[SKIP] Log already patched")
else:
    print("[WARN] Could not find old task received log line.")

CPP.write_text(s)


# ============================================================
# 6. Create task publishing tool
# ============================================================
publish_tool_code = r'''#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import re
import sys
import time
import unicodedata

import rclpy
from std_msgs.msg import String


def normalize_pickup_code(text: str) -> str:
    text = unicodedata.normalize("NFKC", str(text)).strip().upper()
    text = re.sub(r"[\s_/]+", "-", text)
    text = re.sub(r"[^A-Z0-9-]+", "-", text)
    text = re.sub(r"-+", "-", text).strip("-")
    return text


def parse_pickup_code(text: str):
    norm = normalize_pickup_code(text)
    m = re.search(r"(^|[^A-Z0-9])([A-Z0-9]{1,4})-(L\d{1,3})-([A-Z0-9]{4})([^A-Z0-9]|$)", norm)
    if not m:
        raise ValueError(
            f"Invalid pickup code: {text}. Expected format like S03-L1-0301"
        )

    shelf_id = m.group(2)
    layer_id = m.group(3)
    package_code = m.group(4)
    pickup_code = f"{shelf_id}-{layer_id}-{package_code}"

    return {
        "pickup_code": pickup_code,
        "shelf_id": shelf_id,
        "layer_id": layer_id,
        "package_code": package_code,
        "cell_id": pickup_code,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Publish a logistics pickup task using a full DataMatrix pickup code, e.g. S03-L1-0301."
    )
    parser.add_argument("pickup_code", help="Full pickup code, e.g. S03-L1-0301")
    parser.add_argument("--topic", default="/task/shelf_goal", help="Task topic. Default: /task/shelf_goal")
    parser.add_argument("--raw", action="store_true", help="Publish raw code string instead of JSON")
    parser.add_argument("--include-parsed", action="store_true", help="Include parsed shelf_id/layer_id/package_code fields in JSON")
    parser.add_argument("--repeat", type=int, default=3, help="Publish repeat count. Default: 3")
    parser.add_argument("--interval", type=float, default=0.2, help="Seconds between repeated publishes. Default: 0.2")
    args = parser.parse_args()

    try:
        parsed = parse_pickup_code(args.pickup_code)
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(2)

    if args.raw:
        payload = parsed["pickup_code"]
    else:
        data = {
            "task_type": "pickup",
            "pickup_code": parsed["pickup_code"],
        }
        if args.include_parsed:
            data.update({
                "shelf_id": parsed["shelf_id"],
                "layer_id": parsed["layer_id"],
                "package_code": parsed["package_code"],
                "cell_id": parsed["cell_id"],
            })
        payload = json.dumps(data, ensure_ascii=False)

    rclpy.init()
    node = rclpy.create_node("send_pickup_task")
    pub = node.create_publisher(String, args.topic, 10)

    # Give DDS discovery a short moment.
    end = time.time() + 0.8
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    msg = String()
    msg.data = payload

    for i in range(max(1, args.repeat)):
        pub.publish(msg)
        print(f"[PUBLISH {i+1}/{max(1, args.repeat)}] topic={args.topic} data={payload}")
        rclpy.spin_once(node, timeout_sec=max(0.01, args.interval))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
'''

PUBLISH_TOOL.write_text(publish_tool_code)
PUBLISH_TOOL.chmod(0o755)
print(f"[OK] Created publish tool: {PUBLISH_TOOL}")


# ============================================================
# 7. Install publish tool through CMakeLists.txt
# ============================================================
cmake = CMAKE.read_text(errors="ignore")

if "scripts/send_pickup_task.py" not in cmake:
    if "scripts/send_shelf_goal.py" in cmake:
        cmake = cmake.replace(
            "  scripts/send_shelf_goal.py\n",
            "  scripts/send_shelf_goal.py\n  scripts/send_pickup_task.py\n",
            1
        )
        CMAKE.write_text(cmake)
        print("[OK] Added send_pickup_task.py to CMake install(PROGRAMS)")
    else:
        print("[WARN] Could not find scripts/send_shelf_goal.py in CMakeLists.txt. Please add scripts/send_pickup_task.py manually.")
else:
    print("[SKIP] send_pickup_task.py already listed in CMakeLists.txt")


# ============================================================
# 8. Final validation hints
# ============================================================
print("")
print("[DONE]")
print("Modified:")
print(f"  - {CPP}")
print(f"  - {PUBLISH_TOOL}")
print(f"  - {CMAKE}")
print("")
print("Next commands:")
print("  cd /host-home")
print("  source /opt/ros/humble/setup.bash")
print("  colcon build --packages-select robot_logistics_task")
print("  source install/setup.bash")
print("")
print("Publish task after build:")
print("  ros2 run robot_logistics_task send_pickup_task.py S03-L1-0301")
print("")
print("Or publish raw string:")
print("  ros2 run robot_logistics_task send_pickup_task.py S03-L1-0301 --raw")
