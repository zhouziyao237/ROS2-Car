std_msgs/Header header
string qr_type        # shelf_anchor / shelf_front / shelf_side / column / layer / package
string qr_no          # raw decoded id, e.g. S01 / C3 / L2 / A1234
string shelf_id
string face           # front / side_left / side_right / unknown
string column_id
string layer_id
string package_code
int32 row
int32 col
float64 map_x
float64 map_y
float64 map_yaw
float32 confidence
bool is_match_target
