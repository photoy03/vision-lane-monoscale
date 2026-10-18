# Layout of OccupancySnapshot before MapSnapshotMeta.excluded_cameras, kept so that
# tools can still read recordings made with it. Not published by any node.
# The accumulated occupancy grid and its time contract, taken in one step.
std_msgs/Header header                    # frame: map frame; stamp: meta.content_max
MapSnapshotMetaV1 meta
nav_msgs/OccupancyGrid grid               # the map exact at complete_through (per-camera grid versions);
                                          # equals /perception/occupancy_grid_map unless a camera swept past it
