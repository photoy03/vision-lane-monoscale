# Time contract of one published map snapshot (docs/STRUCTURE_KO.md, "지도 스냅샷").
# A snapshot is the exact map state at time T for every
#   content_max <= T <= complete_through   (valid == true).
# Snapshots with valid == false are for diagnosis only and must not be fused.
string layer                              # "lane" | "occupancy"
string session_id                         # producer instance; differs after a node restart
uint64 epoch                              # producer's own map reset count (diagnostic)
# Backward jumps of the ROS clock seen by the producer (bag restart = +1). The
# same /clock drives every node, so layers of one time line share this number;
# never combine snapshots whose clock_generation differs.
uint64 clock_generation
uint64 content_version                    # +1 whenever the content may have changed (add, merge, prune, expire, reset)
uint64 snapshot_sequence                  # +1 on every publish, also when only complete_through moved
builtin_interfaces/Time content_max       # newest capture time of any input that changed this content
builtin_interfaces/Time complete_through  # every input captured <= this was applied or finally discarded
bool valid                                # complete_through set and content_max <= complete_through
string[] discard_reasons                  # inputs finally discarded, cumulative per reason
uint64[] discard_counts
# Changes applied for an input captured at or before an already published
# complete_through. Must stay 0; anything else breaks the contract.
uint64 late_changes
# Cross-layer dependency: the lane map was pruned with an occupancy snapshot.
bool depends_on_occupancy
builtin_interfaces/Time occupancy_content_max_used
# Cameras left out of complete_through because they stopped delivering
# (occupancy camera liveness, C-2 6). Empty: every configured camera counted.
# The area only an excluded camera sees is not being updated.
string[] excluded_cameras
