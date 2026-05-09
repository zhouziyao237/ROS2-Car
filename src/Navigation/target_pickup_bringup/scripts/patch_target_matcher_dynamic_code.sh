change_target_code.sh#!/usr/bin/env bash
set -e

PY_FILE=$(find /host-home/src -path "*target_matcher*" -name "target_matcher_node.py" | head -n 1)
if [ -z "$PY_FILE" ]; then
  echo "ERROR: cannot find target_matcher_node.py"
  exit 1
fi

echo "Using: $PY_FILE"

python3 - "$PY_FILE" <<'PY'
from pathlib import Path
import sys
import re

path = Path(sys.argv[1])
text = path.read_text()

backup = path.with_suffix('.py.bak_dynamic_target')
if not backup.exists():
    backup.write_text(text)
    print(f'Backup created: {backup}')

if 'from std_msgs.msg import' in text:
    m = re.search(r'from std_msgs\.msg import ([^\n]+)', text)
    if m and 'String' not in m.group(1):
        old = m.group(0)
        text = text.replace(old, old + ', String')
        print('Added String to std_msgs import')
else:
    insert_after = 'from rclpy.node import Node\n'
    if insert_after in text:
        text = text.replace(insert_after, insert_after + 'from std_msgs.msg import String\n')
    else:
        text = 'from std_msgs.msg import String\n' + text
    print('Inserted std_msgs String import')

if 'def dynamic_target_code_callback' not in text:
    class_match = re.search(r'class\s+TargetMatcherNode\s*\(', text)
    if not class_match:
        class_match = re.search(r'class\s+TargetMatcherNode\s*:', text)
    if not class_match:
        raise SystemExit('ERROR: Cannot find class TargetMatcherNode')

    method_match = re.search(r'\n    def\s+', text[class_match.end():])
    if not method_match:
        raise SystemExit('ERROR: Cannot find method insertion point in TargetMatcherNode')
    insert_pos = class_match.end() + method_match.start() + 1

    methods = r'''
    def reset_target_lock_for_new_code(self):
        """Clear old locked/matched target when /target_code changes."""
        try:
            self.get_logger().info("Reset target lock/cache for new target code")
        except Exception:
            pass

        none_fields = [
            "locked_target_id", "locked_track_id", "locked_index",
            "matched_index", "matched_target", "matched_detection",
            "current_matched_target", "current_matched_detection",
            "best_match", "best_target", "locked_target",
            "locked_detection", "locked_detection_array",
            "last_matched_detection", "last_matched_target",
        ]
        for name in none_fields:
            if hasattr(self, name):
                try:
                    setattr(self, name, None)
                except Exception:
                    pass

        zero_fields = [
            "stable_match_count", "match_count", "matched_count",
            "lost_count", "lock_count", "same_target_count",
        ]
        for name in zero_fields:
            if hasattr(self, name):
                try:
                    setattr(self, name, 0)
                except Exception:
                    pass

        clear_fields = [
            "ocr_cache", "pending_ocr", "track_ocr_results",
            "ocr_results_cache", "requested_ocr_ids", "ocr_pending",
            "track_text_cache", "target_text_cache", "ocr_request_cache",
            "ocr_result_cache", "track_last_ocr_time", "track_ocr_text",
            "track_ocr_score", "track_match_score",
        ]
        for name in clear_fields:
            if hasattr(self, name):
                obj = getattr(self, name)
                try:
                    obj.clear()
                except Exception:
                    try:
                        setattr(self, name, {})
                    except Exception:
                        pass

        for name in ["state", "current_state", "matcher_state"]:
            if hasattr(self, name):
                try:
                    setattr(self, name, "SEARCH")
                except Exception:
                    pass

        try:
            if hasattr(self, "matched_detection_pub"):
                from yolo_msg.msg import YoloDetectionArray
                msg = YoloDetectionArray()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "oak_rgb_camera_optical_frame"
                self.matched_detection_pub.publish(msg)
        except Exception:
            pass

    def dynamic_target_code_callback(self, msg):
        """Extra /target_code callback for dynamic target switching."""
        new_code = str(msg.data).strip()
        if not new_code:
            return

        old_code = str(getattr(self, "target_code", "")).strip()
        try:
            self.target_code = new_code
        except Exception:
            pass
        try:
            self.target_text = new_code
        except Exception:
            pass

        if new_code != old_code:
            try:
                self.get_logger().info(f"Target code changed: {old_code} -> {new_code}")
            except Exception:
                pass
            self.reset_target_lock_for_new_code()
        else:
            try:
                self.get_logger().info(f"Target code refreshed: {new_code}")
            except Exception:
                pass

'''
    text = text[:insert_pos] + methods + text[insert_pos:]
    print('Inserted dynamic target code methods')
else:
    print('dynamic_target_code_callback already exists')

if 'dynamic_target_code_sub' not in text:
    init_match = re.search(r'def\s+__init__\s*\(\s*self\s*(?:,[^)]*)?\)\s*:', text)
    if not init_match:
        raise SystemExit('ERROR: Cannot find __init__')
    init_start = init_match.end()
    next_method = re.search(r'\n    def\s+', text[init_start:])
    if not next_method:
        raise SystemExit('ERROR: Cannot find end of __init__')
    insert_pos = init_start + next_method.start()

    subscription_code = r'''
        # Extra dynamic target-code subscription. This is intentionally added
        # in addition to the original target-code callback, because different
        # matcher versions use different callback names.
        try:
            self.dynamic_target_code_sub = self.create_subscription(
                String,
                "/target_code",
                self.dynamic_target_code_callback,
                10
            )
            self.get_logger().info("Dynamic target code callback enabled on /target_code")
        except Exception as e:
            self.get_logger().error(f"Failed to create dynamic target code subscription: {e}")

'''
    text = text[:insert_pos] + subscription_code + text[insert_pos:]
    print('Inserted dynamic_target_code_sub subscription')
else:
    print('dynamic_target_code_sub already exists')

path.write_text(text)
print(f'Patched successfully: {path}')
PY

echo "Done."
