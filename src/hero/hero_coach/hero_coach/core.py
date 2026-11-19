"""코치의 순수 계산. ROS 도 Autoware 도 모른다 — 그래서 단독으로 테스트된다.

좌표 약속: 월드는 격자의 `map` 프레임, 자차 pose 는 (x, y, yaw) 로 후륜축 접지점.
base_link 는 x 전방, y 좌측.
"""

from dataclasses import dataclass
import math
from typing import List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

# hero_hud/msg/VehicleState 의 기어 상수.
UNKNOWN, PARK, REVERSE, NEUTRAL, DRIVE = 0, 1, 2, 3, 4

# autoware_vehicle_msgs/msg/GearReport.
_AW_NEUTRAL, _AW_DRIVE_FIRST, _AW_DRIVE_LAST = 1, 2, 19
_AW_REVERSE, _AW_REVERSE_2, _AW_PARK, _AW_LOW, _AW_LOW_2 = 20, 21, 22, 23, 24


def gear_from_autoware(report: int) -> int:
    if _AW_DRIVE_FIRST <= report <= _AW_DRIVE_LAST or report in (_AW_LOW, _AW_LOW_2):
        return DRIVE
    if report in (_AW_REVERSE, _AW_REVERSE_2):
        return REVERSE
    if report == _AW_PARK:
        return PARK
    if report == _AW_NEUTRAL:
        return NEUTRAL
    return UNKNOWN


def bicycle_from_front_left(angle: float, wheelbase_m: float, half_track_m: float) -> float:
    """왼쪽 앞바퀴 각도 → 자전거 모델 등가각. 좌회전 양수.

    autoware_carla_interface 는 FL 바퀴 하나를 보고한다. CARLA 는 Ackermann 으로 꺾으므로
    그 바퀴는 좌회전에서 안쪽(크게), 우회전에서 바깥쪽(작게)이다. HUD 의 자전거 모델에
    그대로 넣으면 steer 0.5 좌회전에서 곡률이 13% 과장된다.
    tan a = L / (R - h)  →  tan d = L / R = L tan a / (L + h tan a). 부호까지 성립한다.
    """
    t = math.tan(angle)
    return math.atan(wheelbase_m * t / (wheelbase_m + half_track_m * t))


def to_base(xs, ys, ego_x: float, ego_y: float, ego_yaw: float):
    c, s = math.cos(ego_yaw), math.sin(ego_yaw)
    dx = np.asarray(xs, dtype=float) - ego_x
    dy = np.asarray(ys, dtype=float) - ego_y
    return c * dx + s * dy, -s * dx + c * dy


@dataclass(frozen=True)
class Footprint:
    """base_link(후륜축) 기준 차체 상자. rear 는 음수."""

    front: float
    rear: float
    half_width: float

    def distance(self, bx, by):
        """상자 바깥까지의 거리. 안이면 0."""
        dx = np.maximum(np.maximum(self.rear - bx, bx - self.front), 0.0)
        dy = np.maximum(np.abs(by) - self.half_width, 0.0)
        return np.hypot(dx, dy)


# ------------------------------------------------------------- 격자 → 장애물

class Obstacle(NamedTuple):
    x: float         # base_link
    y: float
    extent: float    # 발자국 반경
    distance: float  # 차체 바깥까지


def _cell_window(shape, res, origin, cx, cy, half):
    """(cx, cy) 를 중심으로 half 만큼의 행·열 범위. 격자 밖은 잘린다."""
    h, w = shape
    c0 = max(0, int(math.floor((cx - half - origin[0]) / res)))
    c1 = min(w, int(math.ceil((cx + half - origin[0]) / res)))
    r0 = max(0, int(math.floor((cy - half - origin[1]) / res)))
    r1 = min(h, int(math.ceil((cy + half - origin[1]) / res)))
    return r0, r1, c0, c1


def _occupied_centres(data, res, origin, window, threshold):
    r0, r1, c0, c1 = window
    if r1 <= r0 or c1 <= c0:
        return np.empty(0), np.empty(0)
    rows, cols = np.nonzero(data[r0:r1, c0:c1] >= threshold)
    return (origin[0] + (cols + c0 + 0.5) * res,
            origin[1] + (rows + r0 + 0.5) * res)


def grid_obstacles(data: np.ndarray, res: float, origin: Tuple[float, float],
                   ego: Tuple[float, float, float], footprint: Footprint, *,
                   threshold: int = 65, x_range: Tuple[float, float] = (-10.0, 15.0),
                   half_lateral: float = 6.0, bin_m: float = 0.3) -> List[Obstacle]:
    """점유 셀을 base_link 장애물로. 미지(-1)는 그리지 않는다 — 모르면 안 그린다.

    차체 안의 점유는 버린다. 남기면 코리도 표본 0 에 영구 정지선이 선다.
    """
    reach = math.hypot(max(abs(x_range[0]), abs(x_range[1])), half_lateral)
    window = _cell_window(data.shape, res, origin, ego[0], ego[1], reach)
    wx, wy = _occupied_centres(data, res, origin, window, threshold)
    if wx.size == 0:
        return []

    bx, by = to_base(wx, wy, *ego)
    keep = ((bx >= x_range[0]) & (bx <= x_range[1]) & (np.abs(by) <= half_lateral))
    bx, by = bx[keep], by[keep]
    dist = footprint.distance(bx, by)
    outside = dist > 0.0
    bx, by, dist = bx[outside], by[outside], dist[outside]
    if bx.size == 0:
        return []

    keys = np.stack([np.floor(bx / bin_m), np.floor(by / bin_m)], axis=1)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.ravel()
    count = np.bincount(inverse)
    mean_x = np.bincount(inverse, bx) / count
    mean_y = np.bincount(inverse, by) / count
    nearest = np.full(count.size, np.inf)
    np.minimum.at(nearest, inverse, dist)
    extent = 0.5 * bin_m
    return [Obstacle(float(x), float(y), extent, float(d))
            for x, y, d in zip(mean_x, mean_y, nearest)]


def threat_of(distance: float, critical: float = 0.5, warn: float = 1.5,
              caution: float = 3.0) -> int:
    """ObstacleMarker 의 THREAT_* 값."""
    if distance < critical:
        return 3
    if distance < warn:
        return 2
    if distance < caution:
        return 1
    return 0


class OdometryWatch:
    """오도메트리가 끊겼는가. 기준은 시간이 아니라 pose 없이 달린 거리다.

    monoscale 은 프레임이 아니라 누적 시차로 푼다. CARLA 실측(2026-09-17): 1.5 m/s 에서
    8.7 Hz(풀이당 ~0.17 m), 정지 시 ~0.5 Hz. 고정 0.5 s 로 재면 주차해 멈출 때마다,
    속도별 시간으로 재면 오래된 정지 pose 에서 출발하는 순간마다 인지를 철회해 HUD 가
    줄무늬로 깜빡인다. 달린 거리는 CAN 속도로 적분한다 — 멈춰 있다는 독립된 증거다.
    CAN 이 없으면 floor 만 믿는다.
    """

    def __init__(self, floor_sec: float = 0.5, ceiling_sec: float = 5.0,
                 travel_limit_m: float = 0.6, can_timeout_sec: float = 0.5):
        self.floor_sec = floor_sec
        self.ceiling_sec = ceiling_sec
        self.travel_limit_m = travel_limit_m
        self.can_timeout_sec = can_timeout_sec
        self.odometry_at: Optional[float] = None
        self.speed_at: Optional[float] = None
        self.travelled = 0.0

    def on_odometry(self, now: float) -> None:
        self.odometry_at = now
        self.travelled = 0.0

    def on_speed(self, now: float, speed_mps: float) -> None:
        if self.speed_at is not None and self.odometry_at is not None:
            self.travelled += abs(speed_mps) * max(0.0, now - self.speed_at)
        self.speed_at = now

    def age(self, now: float) -> float:
        return -1.0 if self.odometry_at is None else now - self.odometry_at

    def ok(self, now: float) -> bool:
        if self.odometry_at is None:
            return False
        age = now - self.odometry_at
        if age < self.floor_sec:
            return True
        can_alive = self.speed_at is not None and now - self.speed_at < self.can_timeout_sec
        return can_alive and age < self.ceiling_sec and self.travelled <= self.travel_limit_m


# ------------------------------------------------------------------ 계획 창

def planning_costmap(data: np.ndarray, res: float, origin: Tuple[float, float],
                     points: Sequence[Tuple[float, float]], margin_m: float, *,
                     ego: Optional[Tuple[float, float, float]] = None,
                     footprint: Optional[Footprint] = None):
    """points 를 margin 만큼 감싸는 계획용 창. 알려진 셀만 붙이고 미지는 free.

    Autoware setMap 은 cost < 0 을 장애물로 친다. 카메라 격자는 대부분이 미지이고
    monoscale 은 매핑한 창만 발행하므로, 그대로 넘기면 계획이 원천적으로 실패한다.
    창은 원 격자의 격자점에 맞춘다 — 셀이 반 칸씩 밀리면 장애물이 번진다.
    """
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    kx0 = int(math.floor((min(xs) - margin_m - origin[0]) / res))
    ky0 = int(math.floor((min(ys) - margin_m - origin[1]) / res))
    out_origin = (origin[0] + kx0 * res, origin[1] + ky0 * res)
    width = int(math.ceil((max(xs) + margin_m - out_origin[0]) / res))
    height = int(math.ceil((max(ys) + margin_m - out_origin[1]) / res))
    out = np.zeros((height, width), dtype=np.int8)

    h, w = data.shape
    c0, c1 = max(0, kx0), min(w, kx0 + width)
    r0, r1 = max(0, ky0), min(h, ky0 + height)
    if c1 > c0 and r1 > r0:
        block = data[r0:r1, c0:c1]
        out[r0 - ky0:r1 - ky0, c0 - kx0:c1 - kx0] = np.where(block < 0, 0, block)

    if ego is not None and footprint is not None:
        # 자차 자리에 점유가 있으면 A* 는 시작점부터 충돌로 본다.
        half = math.hypot(max(footprint.front, -footprint.rear), footprint.half_width)
        r0, r1, c0, c1 = _cell_window(out.shape, res, out_origin, ego[0], ego[1], half)
        if r1 > r0 and c1 > c0:
            cols, rows = np.meshgrid(np.arange(c0, c1), np.arange(r0, r1))
            bx, by = to_base(out_origin[0] + (cols + 0.5) * res,
                             out_origin[1] + (rows + 0.5) * res, *ego)
            inside = footprint.distance(bx, by) <= 0.0
            out[r0:r1, c0:c1][inside] = 0
    return out, out_origin


def path_blocked(data: np.ndarray, res: float, origin: Tuple[float, float],
                 path: Sequence['Waypoint'], footprint: Footprint,
                 threshold: int = 65) -> bool:
    """남은 경로의 어느 자세에서든 차체 상자 안에 점유가 드는가.

    A* 와 같은 상자로 본다. legacy 는 후륜축 중심 원으로 봤는데, A* 가 0.11 m 여유로
    통과시킨 경로를 원이 1.10 m 에서 막힘으로 읽어 매 쿨다운마다 재계획했다.
    미지는 막지 않는다(계획 창과 같은 규칙).
    """
    if not path:
        return False
    px = np.array([w.x for w in path])
    py = np.array([w.y for w in path])
    reach = math.hypot(max(footprint.front, -footprint.rear), footprint.half_width)
    cx, cy = 0.5 * (px.min() + px.max()), 0.5 * (py.min() + py.max())
    half = 0.5 * max(np.ptp(px), np.ptp(py)) + reach
    ox, oy = _occupied_centres(data, res, origin,
                               _cell_window(data.shape, res, origin, cx, cy, half),
                               threshold)
    if ox.size == 0:
        return False
    for w in path:
        near = np.abs(ox - w.x) + np.abs(oy - w.y) <= 2.0 * reach
        if not near.any():
            continue
        bx, by = to_base(ox[near], oy[near], w.x, w.y, w.yaw)
        if (footprint.distance(bx, by) <= 0.0).any():
            return True
    return False


# ------------------------------------------------------------------ 음성 안내
#
# 저시력 사용자 피드백으로 HUD 에서 글자를 걷어냈다. 기어와 정지는 도형과 **말**이
# 나른다. 말은 화면보다 느리므로 상태가 바뀌는 순간만 말한다 — 계속 말하면 정작
# 급할 때 밀린다.

VOICE_PHRASES = {DRIVE: '전진', REVERSE: '후진', PARK: '정지'}
VOICE_STOP = '정지'


def travel_blocked(obstacles: Sequence[Tuple[float, int]], gear: int,
                   threat: int = 3) -> bool:
    """가는 쪽에 임계 장애물이 있는가. obstacles 는 (base_link x, threat).

    뒤에 있는 것은 전진을 막지 않는다 — 방향을 안 보면 주차장에서 내내 "정지"다.
    """
    if gear == DRIVE:
        forward = True
    elif gear == REVERSE:
        forward = False
    else:
        return False
    return any(t >= threat and ((x > 0.0) if forward else (x < 0.0))
               for x, t in obstacles)


class VoicePolicy:
    """무엇을 언제 말할지. 시각(now)은 부르는 쪽이 준다 — 그래서 테스트된다."""

    def __init__(self, min_gap_sec: float = 1.2, repeat_sec: float = 3.0):
        self.min_gap_sec = min_gap_sec
        self.repeat_sec = repeat_sec
        self.said: Optional[str] = None
        self.said_at = -math.inf

    def decide(self, now: float, advised_gear: int, blocked: bool) -> Optional[str]:
        target = VOICE_STOP if blocked else VOICE_PHRASES.get(advised_gear)
        if target is None:
            self.said = None       # 말할 것이 없어지면 다음 안내는 새 소식이다
            return None
        if now - self.said_at < self.min_gap_sec:
            return None
        if target == self.said:
            # 되풀이는 **지금 막혀 있는 동안**만. 같은 "정지" 라도 도착 권고는 한 번이면
            # 되고, 막힘은 사라지기 전까지 사실이다.
            if not blocked or now - self.said_at < self.repeat_sec:
                return None
        self.said = target
        self.said_at = now
        return target


# -------------------------------------------------------- 궤적 → 구간 · 리본

@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float
    is_back: bool


def densify(waypoints: Sequence[Waypoint], step: float) -> List[Waypoint]:
    """A* 경로점은 확장 거리(~1 m)만큼 성기다. 이탈·막힘 판단과 리본이 step 간격을 본다.

    사이에 넣는 점은 뒤 점의 방향을 따른다 — is_back 은 그 점에 도달한 움직임이다.
    """
    if not waypoints:
        return []
    out = [waypoints[0]]
    for a, b in zip(waypoints, waypoints[1:]):
        n = int(math.ceil(math.hypot(b.x - a.x, b.y - a.y) / step))
        for k in range(1, n):
            u = k / n
            yaw = a.yaw + math.remainder(b.yaw - a.yaw, 2.0 * math.pi) * u
            out.append(Waypoint(a.x + (b.x - a.x) * u, a.y + (b.y - a.y) * u, yaw, b.is_back))
        out.append(b)
    return out


def split_segments(waypoints: Sequence[Waypoint]) -> List[List[Waypoint]]:
    """전후진이 바뀌는 곳에서 자른다. 다음 구간은 전환점(앞 구간의 끝)에서 시작한다."""
    segments: List[List[Waypoint]] = []
    for wp in waypoints:
        if segments and segments[-1][-1].is_back == wp.is_back:
            segments[-1].append(wp)
            continue
        if segments:
            last = segments[-1][-1]
            segments.append([Waypoint(last.x, last.y, last.yaw, wp.is_back), wp])
        else:
            segments.append([wp])
    return segments


def nearest_index(segment: Sequence[Waypoint], x: float, y: float, start: int = 0) -> int:
    """start 이후에서 가장 가까운 점. 뒤로는 안 간다 — 지나온 점으로 되돌아가면
    리본이 자차 뒤에서 시작한다."""
    best, best_d = start, math.inf
    for i in range(start, len(segment)):
        d = math.hypot(segment[i].x - x, segment[i].y - y)
        if d < best_d:
            best, best_d = i, d
    return best


class SegmentTracker:
    """지금 어느 구간의 어디를 달리는가."""

    def __init__(self, segments: List[List[Waypoint]], arrive_m: float = 0.5):
        self.segments = segments
        self.arrive_m = arrive_m
        self.index = 0
        self.progress = 0
        self.finished = not segments

    @property
    def current(self) -> Optional[List[Waypoint]]:
        return None if self.finished else self.segments[self.index]

    def update(self, x: float, y: float) -> None:
        if self.finished:
            return
        last = len(self.segments) - 1
        while True:
            end = self.segments[self.index][-1]
            if math.hypot(end.x - x, end.y - y) >= self.arrive_m:
                break
            if self.index == last:
                self.finished = True
                return
            self.index += 1
            self.progress = 0
        self.progress = nearest_index(self.segments[self.index], x, y, self.progress)


def advised_gear(tracker: SegmentTracker) -> int:
    if not tracker.segments:
        return UNKNOWN
    if tracker.finished:
        return PARK
    return REVERSE if tracker.current[-1].is_back else DRIVE


def ribbon_points(segment: Sequence[Waypoint], start: int,
                  ego: Tuple[float, float, float]) -> List[Tuple[float, float]]:
    rest = segment[start:]
    bx, by = to_base([w.x for w in rest], [w.y for w in rest], *ego)
    return [(float(x), float(y)) for x, y in zip(bx, by)]
