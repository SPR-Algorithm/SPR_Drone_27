# ENU/FLU; header.frame_id must be world.
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float32 yaw
float32 yaw_rate
# Duration after header.stamp; zero requests the configured consumer timeout.
float32 valid_until_s
string constraints_json
