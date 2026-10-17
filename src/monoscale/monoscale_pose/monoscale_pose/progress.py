"""Python side of the common /pipeline/progress format (see vision_lane_interfaces
msg/PipelineProgress.msg and include/vision_lane_interfaces/pipeline_progress.hpp)."""

TOPIC = '/pipeline/progress'


def watermark_from(oldest_unfinished_ns, newest_seen_ns):
    """Just before the oldest unfinished input, never past the newest input seen."""
    newest = max(newest_seen_ns, 0)
    if oldest_unfinished_ns > 0:
        return min(oldest_unfinished_ns - 1, newest)
    return newest


class MonotonicWatermark:
    """Never moves backwards; lower candidates are ignored and counted."""

    def __init__(self):
        self.value = 0
        self.regressions = 0

    def update(self, candidate_ns):
        if candidate_ns > self.value:
            self.value = candidate_ns
        elif 0 < candidate_ns < self.value:
            self.regressions += 1
        return self.value


def make_progress(message_type, now_ns, node, output_topic, watermark_ns,
                  queue_size, processed, dropped, regressions=0):
    """Fill a vision_lane_interfaces/msg/PipelineProgress (type passed in, so this module stays ROS-free)."""
    out = message_type()
    out.header.stamp.sec, out.header.stamp.nanosec = divmod(int(now_ns), 1_000_000_000)
    out.node = node
    out.output_topic = output_topic
    out.watermark_valid = watermark_ns > 0
    out.watermark.sec, out.watermark.nanosec = divmod(int(max(watermark_ns, 0)), 1_000_000_000)
    out.latency_ms = (now_ns - watermark_ns) * 1e-6 if watermark_ns > 0 else -1.0
    out.queue_size = int(queue_size)
    out.processed_total = int(processed)
    out.dropped_total = int(dropped)
    out.watermark_regressions = int(regressions)
    return out
