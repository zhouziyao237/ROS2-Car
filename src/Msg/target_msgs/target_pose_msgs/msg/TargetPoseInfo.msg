std_msgs/Header header

bool valid
string target_id
string ocr_text
float32 score

# bbox in RGB image, unit: pixel
float32 bbox_center_u
float32 bbox_center_v
float32 bbox_width_px
float32 bbox_height_px

# raw target position in camera optical frame, unit: meter
geometry_msgs/Point camera_point

# target position in output frame such as base_link / arm_base_link, unit: meter
geometry_msgs/Point target_point
string target_frame

# depth and simple size estimation, unit: meter
float32 depth_m
float32 distance_3d_m
float32 width_m
float32 height_m
