std_msgs/Header header

string target_id
int32 line_id

string raw_text
float32 confidence

int32[] indices
bool has_text

# YOLO/Fusion provided ROI box in RGB image coordinates.
yolo_msg/Box2D roi_box

# OCR det text-line box mapped back to RGB image coordinates.
yolo_msg/Box2D text_box
