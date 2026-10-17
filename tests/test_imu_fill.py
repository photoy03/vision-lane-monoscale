"""IMU-rate pose fill between two accepted raw odometry updates (ROS-free)."""
import importlib.util
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'odom_filter', ROOT / 'src/monoscale/monoscale_pose/monoscale_pose/odom_filter.py')
odom_filter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(odom_filter)
Pose2, fill = odom_filter.Pose2, odom_filter.imu_fill_poses


def imu(yaw_of_t, start=0.0, end=1.0, hz=100.0):
    count = int(round((end - start) * hz))
    return [(start + k / hz, yaw_of_t(start + k / hz)) for k in range(-5, count + 6)]


def test_straight_line_is_linear_and_keeps_endpoints_out():
    poses = fill(Pose2(0, 0, 0), Pose2(1, 0, 0), 0.0, 1.0, [0.0, 0.25, 0.5, 1.0, 1.4],
                 imu(lambda t: 0.0))
    assert sorted(poses) == [0.25, 0.5]
    assert math.isclose(poses[0.5].x, 0.5, abs_tol=1e-9) and abs(poses[0.5].y) < 1e-9


def test_turn_follows_the_arc_not_the_chord():
    radius = 2.0
    end = Pose2(radius, radius, math.pi / 2)  # quarter circle to the left
    poses = fill(Pose2(0, 0, 0), end, 0.0, 1.0, [0.5], imu(lambda t: t * math.pi / 2))
    middle = poses[0.5]
    assert math.isclose(middle.x, radius * math.sin(math.pi / 4), abs_tol=0.01)
    assert math.isclose(middle.y, radius * (1 - math.cos(math.pi / 4)), abs_tol=0.01)
    assert math.isclose(middle.yaw, math.pi / 4, abs_tol=1e-6)


def test_heading_keeps_the_corrected_offset_policy():
    # IMU reads 10 deg high; the accepted (corrected) endpoints define the offset.
    bias = math.radians(10.0)
    poses = fill(Pose2(0, 0, 0), Pose2(1, 0, 0), 0.0, 1.0, [0.5], imu(lambda t: bias))
    assert abs(poses[0.5].yaw) < 1e-9


def test_reversing_moves_against_heading():
    poses = fill(Pose2(0, 0, 0), Pose2(-1, 0, 0), 0.0, 1.0, [0.5], imu(lambda t: 0.0))
    assert math.isclose(poses[0.5].x, -0.5, abs_tol=1e-9)


def test_missing_imu_or_inconsistent_path_is_rejected():
    assert fill(Pose2(0, 0, 0), Pose2(1, 0, 0), 0.0, 1.0, [0.5], imu(lambda t: 0.0, 0.0, 0.3)) is None
    gappy = [(t, y) for t, y in imu(lambda t: 0.0) if not 0.4 < t < 0.6]
    assert fill(Pose2(0, 0, 0), Pose2(1, 0, 0), 0.0, 1.0, [0.5], gappy) is None
    # IMU says a 90 deg turn, the visual endpoints say a straight metre.
    assert fill(Pose2(0, 0, 0), Pose2(1, 0, math.pi / 2), 0.0, 1.0, [0.5],
                imu(lambda t: t * math.pi / 2)) is None
