# A measured, time-aligned lane observation recovered from the mapper queue.
# header.stamp is the original image acquisition stamp; header.frame_id is base_link.
std_msgs/Header header
string session_id
uint64 epoch
uint64 observation_id
string camera_id
string source_frame
string calibration_id
# Only "exact" or "interpolated" poses are publishable on this contract.
string pose_match
geometry_msgs/Pose2D observer_pose
vision_lane_interfaces/MarkingSegment[] segments
