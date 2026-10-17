# The accumulated lane map and its time contract, taken in one step.
std_msgs/Header header                    # frame: map frame; stamp: meta.content_max
MapSnapshotMeta meta
MarkingSegmentArray lanes                 # same content as /parking/cpp/global_marking_segments_fused
