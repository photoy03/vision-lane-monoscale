# Where one /localization/kinematic_state pose came from. Same header.stamp as that
# pose; published right after it on <pose topic>/provenance.
uint8 UNKNOWN=0
uint8 RAW=1            # a raw MonoScale measurement passed through
uint8 INTERPOLATED=2   # blended between two raw measurements (stabilizer: short, moving, stationary, straight)
uint8 IMU_FILL=3       # shaped by IMU yaw between raw measurements (imu_fill, imu_straight, imu_stationary)
uint8 HELD=4           # last value held; no path produces it today (reserved)
std_msgs/Header header
uint8 kind
string resample_kind               # the stabilizer's own name; "raw" for a passthrough
bool covariance_valid              # the standard pose.covariance means something only when true (unknown: zero matrix)
builtin_interfaces/Time raw_lower  # the raw measurements this pose was made from (equal for RAW)
builtin_interfaces/Time raw_upper
# Cameras whose own solve produced the raw measurement(s) behind this pose ("front,rear",
# "rear", ...); for a blend the weaker end (fewer cameras). "unknown" when the odometry did
# not say within 0.5 s; empty from an odometry that does not report it (docs/C2_CRITERIA_KO.md 7.2-6).
string raw_cameras
