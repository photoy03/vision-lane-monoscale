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
# Per-frame completion (docs/C1_CRITERIA_KO.md), map-mode observation stream only.
# lane_detection sends exactly one message per numbered image, in receive order.
# frame_sequence: per camera and epoch, 1, 2, 3, ... without holes (0 = not numbered, older sender).
# observation_id is a different counter (process lifetime, ledger id).
uint8 FRAME_UNSPECIFIED=0
uint8 FRAME_OBSERVED=1
uint8 FRAME_DISCARDED=2
uint64 frame_sequence
uint8 frame_outcome
# warmup / quality / not_vectorized / queue_overflow / decode / calibration / empty_image / no_pose
string discard_reason
