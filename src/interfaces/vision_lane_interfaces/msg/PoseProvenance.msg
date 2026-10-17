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
bool covariance_valid              # false: the pose covariance is unknown (pose.covariance[0] = -1)
builtin_interfaces/Time raw_lower  # the raw measurements this pose was made from (equal for RAW)
builtin_interfaces/Time raw_upper
