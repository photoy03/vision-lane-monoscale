# The accumulated occupancy grid and its time contract, taken in one step.
std_msgs/Header header                    # frame: map frame; stamp: meta.content_max
MapSnapshotMeta meta
nav_msgs/OccupancyGrid grid               # same content as /perception/occupancy_grid_map
