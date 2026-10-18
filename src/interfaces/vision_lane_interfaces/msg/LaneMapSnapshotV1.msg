# Layout of LaneMapSnapshot before MapSnapshotMeta.excluded_cameras, kept so that
# tools can still read recordings made with it. Not published by any node.
# The accumulated lane map and its time contract, taken in one step.
std_msgs/Header header                    # frame: map frame; stamp: meta.content_max
MapSnapshotMetaV1 meta
MarkingSegmentArray lanes                 # same content as /parking/cpp/global_marking_segments_fused
