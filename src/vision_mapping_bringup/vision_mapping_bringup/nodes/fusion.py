"""Slot for the time-matched map fusion node (docs/design/FUSION_NODE_DESIGN_KO.md).

It will combine the lane map, the occupancy grid and the pose that are all
confirmed up to the same capture time T into one fused map. Not implemented
yet, so it contributes no nodes; the pipeline already calls it in its final
position so adding the node only touches this module.
"""


def nodes(ctx):
    return []
