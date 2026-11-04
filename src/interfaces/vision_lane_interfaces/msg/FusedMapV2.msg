# Layout of FusedMap up to snapshot_format 2 (before pose provenance), kept so that
# tools can still read recordings made with it. Not published by any node.
# Lane map + occupancy grid that are both exact at one capture time T
# (docs/STRUCTURE_KO.md "통합 지도"). Published only when T moves forward.
std_msgs/Header header                 # stamp: T; frame: map frame
uint64 sequence
uint64 clock_generation                 # time line of T; both layers carry the same one
# true (strict): every layer satisfies content_max <= T <= complete_through.
# false only in the explicit incomplete mode; stale_layers names the layers
# whose latest snapshot is used although it is not exact at T.
bool complete
string[] stale_layers
LaneMapSnapshotV1 lanes
OccupancySnapshotV1 occupancy
# Vehicle pose at T: a measured pose or a blend of the two measured poses
# around T (never across a gap longer than the limit, never extrapolated).
# The blend uses the later pose, i.e. pose information after T.
bool pose_valid
string pose_match                       # exact | interpolated | gap_too_large | stamp_after_latest | ...
geometry_msgs/Pose2D pose
builtin_interfaces/Time pose_lower
builtin_interfaces/Time pose_upper
