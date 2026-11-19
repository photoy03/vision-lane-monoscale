"""순수 함수. ROS 없이 돈다."""

import math

import numpy as np
import pytest

from hero_coach.core import (
    DRIVE, NEUTRAL, PARK, REVERSE, UNKNOWN,
    Footprint, SegmentTracker, VoicePolicy, Waypoint, bicycle_from_front_left,
    advised_gear, densify, gear_from_autoware, grid_obstacles, nearest_index,
    OdometryWatch,
    path_blocked, planning_costmap, ribbon_points, split_segments, threat_of,
    travel_blocked,
    to_base,
)

PRIUS = Footprint(front=3.656, rear=-0.858, half_width=1.0)


# ------------------------------------------------------------------ 기어

@pytest.mark.parametrize('report, gear', [
    (0, UNKNOWN), (1, NEUTRAL), (2, DRIVE), (19, DRIVE), (20, REVERSE),
    (21, REVERSE), (22, PARK), (23, DRIVE), (24, DRIVE), (99, UNKNOWN),
])
def test_gear_from_autoware(report, gear):
    assert gear_from_autoware(report) == gear


# ------------------------------------------------------------------ 좌표

def test_to_base_rotates_into_heading():
    bx, by = to_base(np.array([1.0]), np.array([1.0]), 1.0, 0.0, math.pi / 2)
    assert bx[0] == pytest.approx(1.0)
    assert by[0] == pytest.approx(0.0, abs=1e-9)


# --------------------------------------------------------------- 격자→장애물

def grid_with(cells, width=200, height=200, res=0.1, origin=(-10.0, -10.0), fill=0):
    data = np.full((height, width), fill, dtype=np.int8)
    for (x, y, value) in cells:
        col = int((x - origin[0]) / res)
        row = int((y - origin[1]) / res)
        data[row, col] = value
    return data, res, origin


def test_obstacle_ahead_is_reported_in_base_link():
    data, res, origin = grid_with([(6.05, 0.05, 100)])
    obstacles = grid_obstacles(data, res, origin, (0.0, 0.0, 0.0), PRIUS)
    assert len(obstacles) == 1
    o = obstacles[0]
    assert o.x == pytest.approx(6.05, abs=0.06)
    assert o.y == pytest.approx(0.05, abs=0.06)
    assert o.distance == pytest.approx(6.05 - 3.656, abs=0.06)


def test_unknown_and_free_cells_are_not_obstacles():
    data, res, origin = grid_with([(6.05, 0.05, -1), (5.05, 0.05, 20)])
    assert grid_obstacles(data, res, origin, (0.0, 0.0, 0.0), PRIUS) == []


def test_cells_inside_the_footprint_are_dropped():
    # 남기면 코리도 표본 0 에 영구 정지선이 선다.
    data, res, origin = grid_with([(1.05, 0.05, 100)])
    assert grid_obstacles(data, res, origin, (0.0, 0.0, 0.0), PRIUS) == []


def test_cells_outside_the_window_are_dropped():
    data, res, origin = grid_with([(0.05, 8.05, 100)])
    assert grid_obstacles(data, res, origin, (0.0, 0.0, 0.0), PRIUS) == []


def test_neighbouring_cells_merge_into_one_bin():
    cells = [(6.05, 0.05, 100), (6.15, 0.05, 100), (6.05, 0.15, 100)]
    data, res, origin = grid_with(cells)
    obstacles = grid_obstacles(data, res, origin, (0.0, 0.0, 0.0), PRIUS, bin_m=0.3)
    assert len(obstacles) == 1


def test_obstacles_follow_the_ego_pose():
    # 자차가 (0, 0) 에서 +y 를 본다. 월드 (0, 6) 은 base_link 정면 6 m.
    data, res, origin = grid_with([(0.05, 6.05, 100)])
    obstacles = grid_obstacles(data, res, origin, (0.0, 0.0, math.pi / 2), PRIUS)
    assert len(obstacles) == 1
    assert obstacles[0].x == pytest.approx(6.05, abs=0.1)
    assert obstacles[0].y == pytest.approx(0.0, abs=0.1)


@pytest.mark.parametrize('distance, threat', [(0.2, 3), (1.0, 2), (2.0, 1), (5.0, 0)])
def test_threat_by_distance(distance, threat):
    assert threat_of(distance) == threat


# ------------------------------------------------------------------ 계획 창

def test_planning_costmap_pastes_known_and_frees_unknown():
    data, res, origin = grid_with([(2.05, 2.05, 100), (3.05, 3.05, -1)], fill=-1)
    out, out_origin = planning_costmap(data, res, origin, [(0.0, 0.0), (20.0, 0.0)], 5.0)
    col = int(round((2.05 - out_origin[0]) / res - 0.5))
    row = int(round((2.05 - out_origin[1]) / res - 0.5))
    assert out[row, col] == 100
    assert (out >= 0).all()
    # 목표가 원래 격자 밖이어도 창이 덮는다.
    assert out_origin[0] + out.shape[1] * res >= 25.0 - 1e-6


def test_planning_costmap_stays_on_the_source_lattice():
    data, res, origin = grid_with([], origin=(-10.03, -10.07))
    _, out_origin = planning_costmap(data, res, origin, [(0.0, 0.0)], 3.0)
    assert ((out_origin[0] - origin[0]) / res) == pytest.approx(
        round((out_origin[0] - origin[0]) / res), abs=1e-6)


def test_planning_costmap_clears_the_ego_footprint():
    data, res, origin = grid_with([(1.05, 0.05, 100)])
    out, out_origin = planning_costmap(
        data, res, origin, [(0.0, 0.0)], 5.0, ego=(0.0, 0.0, 0.0), footprint=PRIUS)
    assert (out == 0).all()


def test_path_blocked_uses_the_body_box_not_a_circle():
    # 차체 옆 0.15 m 에 붙은 점유는 막힘이 아니다. A* 가 같은 상자로 통과시킨 경로다.
    data, res, origin = grid_with([(1.05, 1.15, 100)])
    path = [Waypoint(float(x), 0.0, 0.0, False) for x in np.arange(-3.0, 3.0, 0.25)]
    assert not path_blocked(data, res, origin, path, PRIUS)
    data, res, origin = grid_with([(1.05, 0.95, 100)])
    assert path_blocked(data, res, origin, path, PRIUS)


def test_path_blocked_only_looks_at_the_remaining_path():
    data, res, origin = grid_with([(-5.05, 0.05, 100)])
    path = [Waypoint(float(x), 0.0, 0.0, False) for x in np.arange(-8.0, 8.0, 0.25)]
    assert path_blocked(data, res, origin, path, PRIUS)
    assert not path_blocked(data, res, origin, path[20:], PRIUS)


# ------------------------------------------------------------ 궤적→구간·리본

def straight(x0, x1, n, is_back, y=0.0):
    yaw = 0.0
    return [Waypoint(x0 + (x1 - x0) * i / (n - 1), y, yaw, is_back) for i in range(n)]


def test_split_segments_starts_each_run_at_the_switch_point():
    wps = straight(0.0, 5.0, 6, False) + straight(4.0, 1.0, 4, True)
    segments = split_segments(wps)
    assert len(segments) == 2
    assert segments[1][0].x == pytest.approx(5.0)
    assert segments[1][0].is_back is True
    assert all(w.is_back for w in segments[1])


def test_tracker_advances_at_the_switch_and_finishes_at_goal():
    wps = straight(0.0, 5.0, 6, False) + straight(4.0, 1.0, 4, True)
    tracker = SegmentTracker(split_segments(wps), arrive_m=0.5)
    tracker.update(2.0, 0.0)
    assert tracker.index == 0 and not tracker.finished
    assert advised_gear(tracker) == DRIVE
    tracker.update(4.8, 0.0)
    assert tracker.index == 1
    assert advised_gear(tracker) == REVERSE
    tracker.update(1.2, 0.0)
    assert tracker.finished
    assert advised_gear(tracker) == PARK


def test_nearest_index_does_not_go_backwards():
    seg = straight(0.0, 5.0, 11, False)
    assert nearest_index(seg, 3.0, 0.0, start=0) == 6
    assert nearest_index(seg, 0.0, 0.0, start=6) == 6


def test_ribbon_is_in_base_link_from_the_ego():
    seg = straight(0.0, 5.0, 11, False, y=1.0)
    points = ribbon_points(seg, 2, (1.0, 1.0, 0.0))
    assert points[0] == pytest.approx((0.0, 0.0), abs=1e-9)
    assert points[-1] == pytest.approx((4.0, 0.0), abs=1e-9)


def test_reverse_ribbon_extends_behind_the_ego():
    seg = straight(5.0, 1.0, 5, True)
    points = ribbon_points(seg, 0, (5.0, 0.0, 0.0))
    assert points[-1][0] == pytest.approx(-4.0)


def test_densify_fills_gaps_without_crossing_direction_changes():
    wps = [Waypoint(0.0, 0.0, 0.0, False), Waypoint(1.0, 0.0, 0.0, False),
           Waypoint(0.0, 0.0, 0.0, True)]
    dense = densify(wps, 0.25)
    forward = [w for w in dense if not w.is_back]
    assert [w.x for w in forward] == pytest.approx([0.0, 0.25, 0.5, 0.75, 1.0])
    assert dense[-1] == wps[-1]
    assert split_segments(dense)[1][0].x == pytest.approx(1.0)


def test_densify_interpolates_heading_across_the_wrap():
    wps = [Waypoint(0.0, 0.0, math.radians(170.0), False),
           Waypoint(-1.0, 0.0, math.radians(-170.0), False)]
    mid = densify(wps, 0.5)[1]
    assert abs(math.remainder(mid.yaw - math.pi, 2.0 * math.pi)) < 1e-9


# ----------------------------------------------------------- 오도메트리 신선도

def test_odometry_is_fresh_within_the_floor_regardless_of_motion():
    watch = OdometryWatch()
    watch.on_odometry(0.0)
    assert watch.ok(0.4)


def test_standstill_waits_for_the_ceiling():
    # 정지 중에는 시차가 안 쌓여 monoscale 이 ~2 s 마다 푼다.
    watch = OdometryWatch()
    watch.on_odometry(0.0)
    for k in range(40):
        watch.on_speed(k * 0.1, 0.0)
    assert watch.ok(4.0)
    assert not watch.ok(5.1)


def test_pulling_away_is_not_stale_until_the_car_has_travelled():
    # 오래된 정지 pose 에서 출발한 순간 끊겼다고 하면 HUD 가 출발마다 깜빡인다.
    watch = OdometryWatch()
    watch.on_odometry(0.0)
    watch.on_speed(1.5, 0.0)
    watch.on_speed(1.6, 1.0)
    assert watch.ok(1.6)        # 0.1 m
    watch.on_speed(2.3, 1.0)
    assert not watch.ok(2.3)    # 0.8 m 를 pose 없이


def test_a_new_pose_resets_the_travel():
    watch = OdometryWatch()
    watch.on_odometry(0.0)
    watch.on_speed(0.0, 1.0)
    watch.on_speed(1.0, 1.0)
    watch.on_odometry(1.0)
    assert watch.ok(1.4)


def test_without_can_only_the_floor_counts():
    watch = OdometryWatch()
    watch.on_odometry(0.0)
    assert not watch.ok(0.6)
    watch.on_speed(0.0, 0.0)
    assert not watch.ok(0.6 + OdometryWatch().can_timeout_sec)


# -------------------------------------------------------------- 조향 보정

# CARLA Prius 실측(2026-09-17): 축거 2.819, 앞 윤거 1.629. steer_cmd +1 → FL +1.222 rad
# (안쪽 70°), -1 → FL -0.815 rad(바깥쪽 46.7°).
@pytest.mark.parametrize('front_left', [1.222, -0.815])
def test_front_left_wheel_reduces_to_the_same_bicycle_angle_both_ways(front_left):
    delta = bicycle_from_front_left(front_left, wheelbase_m=2.819, half_track_m=0.8145)
    assert abs(delta) == pytest.approx(0.9925, abs=0.003)
    assert math.copysign(1.0, delta) == math.copysign(1.0, front_left)


def test_straight_wheel_is_straight():
    assert bicycle_from_front_left(0.0, 2.819, 0.8145) == 0.0


# --------------------------------------------------------------- 음성 안내

def test_travel_blocked_only_looks_where_the_car_is_going():
    ahead = [(2.0, 3)]
    behind = [(-2.0, 3)]
    assert travel_blocked(ahead, DRIVE)
    assert not travel_blocked(behind, DRIVE)
    assert travel_blocked(behind, REVERSE)
    assert not travel_blocked(ahead, REVERSE)
    assert not travel_blocked(ahead, PARK)


def test_only_critical_obstacles_speak():
    assert not travel_blocked([(2.0, 2)], DRIVE)


def test_voice_says_the_advised_gear_once():
    voice = VoicePolicy()
    assert voice.decide(0.0, DRIVE, False) == '전진'
    assert voice.decide(0.1, DRIVE, False) is None
    assert voice.decide(2.0, DRIVE, False) is None


def test_voice_speaks_the_change_after_the_gap():
    voice = VoicePolicy(min_gap_sec=1.0)
    assert voice.decide(0.0, DRIVE, False) == '전진'
    assert voice.decide(0.5, REVERSE, False) is None   # 너무 잦으면 말이 겹친다
    assert voice.decide(1.2, REVERSE, False) == '후진'


def test_blocked_overrides_the_gear_and_repeats():
    voice = VoicePolicy(min_gap_sec=1.0, repeat_sec=3.0)
    assert voice.decide(0.0, DRIVE, False) == '전진'
    assert voice.decide(1.5, DRIVE, True) == '정지'
    assert voice.decide(2.0, DRIVE, True) is None
    assert voice.decide(4.6, DRIVE, True) == '정지'     # 급한 것만 되풀이한다


def test_park_advice_says_stop_and_does_not_nag():
    voice = VoicePolicy(min_gap_sec=1.0, repeat_sec=3.0)
    assert voice.decide(0.0, PARK, False) == '정지'
    assert voice.decide(5.0, PARK, False) is None


def test_silence_resets_so_the_next_advice_speaks():
    voice = VoicePolicy(min_gap_sec=1.0)
    assert voice.decide(0.0, DRIVE, False) == '전진'
    assert voice.decide(2.0, UNKNOWN, False) is None
    assert voice.decide(4.0, DRIVE, False) == '전진'
