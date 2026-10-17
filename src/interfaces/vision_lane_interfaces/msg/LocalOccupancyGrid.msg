# The nested grid is local to base_link at header.stamp:
# -1 unknown, 0 free, 100 occupied. Intermediate values are confidence only.
std_msgs/Header header
string camera_id
geometry_msgs/Pose2D observer_pose
bool pose_valid
nav_msgs/OccupancyGrid grid
