# Local vectorizer output uses base_link coordinates and embeds the odometry pose
# interpolated at header.stamp. The fused map uses map coordinates.
std_msgs/Header header
string camera_id
geometry_msgs/Pose2D observer_pose
bool pose_valid
vision_lane_interfaces/MarkingSegment[] segments
