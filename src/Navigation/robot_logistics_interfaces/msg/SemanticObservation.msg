std_msgs/Header header

string obs_type
# shelf_anchor / shelf_front / shelf_side / column_qr / layer_qr / package_qr / cell

string shelf_id
string face
string column_id
string layer_id
string package_code

int32 row
int32 col

geometry_msgs/PoseStamped pose
float32 confidence
bool has_pose
