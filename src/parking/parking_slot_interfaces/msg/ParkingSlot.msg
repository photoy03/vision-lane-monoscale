# Persistent parking-space geometry in the map frame.
uint8 DETECTION_CANDIDATE=0
uint8 DETECTION_CONFIRMED=1
uint8 OCCUPANCY_UNKNOWN=0
uint8 OCCUPANCY_FREE=1
uint8 OCCUPANCY_OCCUPIED=2

uint64 slot_id
uint8 detection_state
uint8 occupancy_state

# Rectangle order: entrance-left, entrance-right, rear-right, rear-left.
geometry_msgs/Point[4] corners
# Centre of the rectangle. Its +X axis points from the entrance into the slot.
geometry_msgs/Pose slot_pose
float32 width
float32 depth

float32 confidence
float32 occupied_ratio
uint32 observation_count
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
