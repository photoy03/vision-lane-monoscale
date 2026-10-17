"""Python side of /pipeline/progress (monoscale_pose/progress.py), ROS-free."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('progress', ROOT / 'src/monoscale/monoscale_pose/monoscale_pose/progress.py')
progress = importlib.util.module_from_spec(spec)
spec.loader.exec_module(progress)


class FakeMessage:
    def __init__(self):
        self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))
        self.watermark = SimpleNamespace(sec=0, nanosec=0)


def test_rules_match_the_cpp_header():
    assert progress.watermark_from(0, 0) == 0
    assert progress.watermark_from(0, 500) == 500
    assert progress.watermark_from(300, 500) == 299
    assert progress.watermark_from(20, 10) == 10  # held input newer than a slower input


def test_monotonic_and_message():
    mark = progress.MonotonicWatermark()
    assert mark.update(100) == 100 and mark.update(90) == 100 and mark.regressions == 1
    assert mark.update(0) == 100 and mark.regressions == 1
    msg = progress.make_progress(FakeMessage, 2_000_000_000, 'pose', '/out', 1_500_000_000, 3, 10, 2)
    assert msg.watermark_valid and (msg.watermark.sec, msg.watermark.nanosec) == (1, 500_000_000)
    assert abs(msg.latency_ms - 500.0) < 1e-9 and msg.queue_size == 3 and msg.watermark_regressions == 0
    assert progress.make_progress(FakeMessage, 2, 'pose', '/out', 1, 0, 0, 0, 4).watermark_regressions == 4
    assert not progress.make_progress(FakeMessage, 1, 'pose', '/out', 0, 0, 0, 0).watermark_valid
