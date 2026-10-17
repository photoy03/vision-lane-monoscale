"""ROS-free filter used to make MonoScale poses safe for lane accumulation."""

import bisect
from collections import deque
from dataclasses import dataclass
import math


def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def resample_span_kind(first, second, span, moving_limit=0.25,
                       stationary_limit=3.0, stationary_translation=0.05,
                       stationary_yaw_deg=2.0, straight_limit=0.0,
                       straight_translation=0.5, straight_yaw_deg=2.0,
                       straight_lateral=0.1):
    """Classify bounded interpolation; never bridge an unobserved turn.

    A straight moving gap is opt-in. Its displacement must agree with the
    starting heading as well as the ending yaw, not merely be small in norm.
    """
    if not math.isfinite(span) or span <= 0.0:
        return "reject"
    if span <= moving_limit:
        return "short"
    dx, dy = second.x - first.x, second.y - first.y
    distance = math.hypot(dx, dy)
    yaw_change = abs(wrap_pi(second.yaw - first.yaw))
    if (span <= stationary_limit and distance <= stationary_translation and
            yaw_change <= math.radians(stationary_yaw_deg)):
        return "stationary"
    if (straight_limit <= 0.0 or span > straight_limit or
            distance > straight_translation or
            yaw_change > math.radians(straight_yaw_deg)):
        return "reject"
    forward = math.cos(first.yaw) * dx + math.sin(first.yaw) * dy
    lateral = -math.sin(first.yaw) * dx + math.cos(first.yaw) * dy
    if forward < -0.02 or abs(lateral) > straight_lateral:
        return "reject"
    return "straight"


def can_resample_span(first, second, span, moving_limit=0.25,
                      stationary_limit=3.0, stationary_translation=0.05,
                      stationary_yaw_deg=2.0, straight_limit=0.0,
                      straight_translation=0.5, straight_yaw_deg=2.0,
                      straight_lateral=0.1):
    return resample_span_kind(
        first, second, span, moving_limit, stationary_limit,
        stationary_translation, stationary_yaw_deg, straight_limit,
        straight_translation, straight_yaw_deg, straight_lateral
    ) != "reject"


def interpolate_imu_yaw(samples, stamp, max_sample_gap_sec=0.12):
    """Interpolate wrapped IMU yaw only when a nearby two-sample bracket exists."""
    if not math.isfinite(stamp) or not math.isfinite(max_sample_gap_sec) or max_sample_gap_sec <= 0.0:
        return None
    previous = None
    for sample_stamp, sample_yaw in samples:
        if not math.isfinite(sample_stamp) or not math.isfinite(sample_yaw):
            continue
        if abs(sample_stamp - stamp) <= 1e-9:
            return wrap_pi(sample_yaw)
        if sample_stamp < stamp:
            previous = (sample_stamp, sample_yaw)
            continue
        if previous is None or sample_stamp - previous[0] > max_sample_gap_sec:
            return None
        ratio = (stamp - previous[0]) / (sample_stamp - previous[0])
        return wrap_pi(previous[1] + ratio * wrap_pi(sample_yaw - previous[1]))
    return None


def imu_fill_poses(first, second, start_stamp, end_stamp, targets, samples,
                   max_sample_gap_sec=0.12, max_closure_m=0.25, max_closure_ratio=0.3):
    """Place poses at ``targets`` strictly inside (start, end) between two accepted poses.

    Heading follows the IMU yaw shape with the endpoint offsets blended linearly,
    exactly as the imu_straight resample already does. Position integrates that
    heading at constant speed, scaled to the observed visual displacement; any
    remaining closure error is spread linearly so both endpoints stay exact.
    Nothing is extrapolated past ``end``. Returns {target: Pose2} or None when
    IMU coverage is missing or the IMU path disagrees with the visual endpoints.
    """
    span = end_stamp - start_stamp
    if not math.isfinite(span) or span <= 0.0:
        return None
    window = [(t, y) for t, y in samples
              if math.isfinite(t) and math.isfinite(y) and
              start_stamp - max_sample_gap_sec <= t <= end_stamp + max_sample_gap_sec]
    stamps = [t for t, _ in window]

    def imu_yaw(stamp):
        index = bisect.bisect_left(stamps, stamp)
        if index < len(stamps) and abs(stamps[index] - stamp) <= 1e-9:
            return wrap_pi(window[index][1])
        if index == 0 or index == len(stamps):
            return None
        (t0, y0), (t1, y1) = window[index - 1], window[index]
        if t1 - t0 > max_sample_gap_sec:
            return None
        return wrap_pi(y0 + (stamp - t0) / (t1 - t0) * wrap_pi(y1 - y0))

    imu_start, imu_end = imu_yaw(start_stamp), imu_yaw(end_stamp)
    if imu_start is None or imu_end is None:
        return None
    offset_start = wrap_pi(first.yaw - imu_start)
    offset_step = wrap_pi(wrap_pi(second.yaw - imu_end) - offset_start)

    def heading(stamp, imu):
        ratio = min(max((stamp - start_stamp) / span, 0.0), 1.0)
        return wrap_pi(imu + offset_start + ratio * offset_step)

    wanted = sorted({t for t in targets if start_stamp + 1e-9 < t < end_stamp - 1e-9})
    grid = sorted(set(wanted) | {t for t in stamps if start_stamp < t < end_stamp} | {end_stamp})
    path, yaws = {}, {}
    px = py = 0.0
    previous_t, previous_h = start_stamp, heading(start_stamp, imu_start)
    for stamp in grid:
        imu = imu_yaw(stamp)
        if imu is None:
            return None
        current_h = heading(stamp, imu)
        middle = wrap_pi(previous_h + 0.5 * wrap_pi(current_h - previous_h))
        dt = stamp - previous_t
        px += math.cos(middle) * dt
        py += math.sin(middle) * dt
        path[stamp], yaws[stamp] = (px, py), current_h
        previous_t, previous_h = stamp, current_h
    dx, dy = second.x - first.x, second.y - first.y
    end_x, end_y = path[end_stamp]
    norm = end_x * end_x + end_y * end_y
    # Signed scale: reversing moves against the heading.
    scale = (dx * end_x + dy * end_y) / norm if norm > 1e-12 else 0.0
    error_x, error_y = dx - scale * end_x, dy - scale * end_y
    distance = math.hypot(dx, dy)
    if math.hypot(error_x, error_y) > max(max_closure_m, max_closure_ratio * distance):
        return None
    poses = {}
    for stamp in wanted:
        ratio = (stamp - start_stamp) / span
        x, y = path[stamp]
        poses[stamp] = Pose2(first.x + scale * x + ratio * error_x,
                             first.y + scale * y + ratio * error_y, yaws[stamp])
    return poses


def imu_straight_resample_safe(
    first, second, start_stamp, end_stamp, samples,
    max_span_sec=0.8, max_translation_m=0.75, max_yaw_deg=8.0,
    max_curve_yaw_deg=3.0, max_lateral_m=0.12,
    max_sample_gap_sec=0.12, max_endpoint_yaw_error_deg=12.0,
):
    """Allow only a short, nearly straight moving gap with continuous IMU yaw.

    Position is still interpolated between observed visual poses. The IMU is
    used only to prove that the hidden interval did not contain a turn and to
    place heading at image timestamps; this never extrapolates translation.
    """
    span = end_stamp - start_stamp
    limits = (max_span_sec, max_translation_m, max_yaw_deg,
              max_curve_yaw_deg, max_lateral_m, max_sample_gap_sec,
              max_endpoint_yaw_error_deg)
    if (not math.isfinite(span) or span <= 0.0 or
            not all(math.isfinite(value) and value > 0.0 for value in limits) or
            span > max_span_sec):
        return False

    rows = sorted((float(t), float(yaw)) for t, yaw in samples
                  if math.isfinite(t) and math.isfinite(yaw) and
                  start_stamp - max_sample_gap_sec <= t <= end_stamp + max_sample_gap_sec)
    start_yaw = interpolate_imu_yaw(rows, start_stamp, max_sample_gap_sec)
    end_yaw = interpolate_imu_yaw(rows, end_stamp, max_sample_gap_sec)
    if start_yaw is None or end_yaw is None:
        return False

    path = [(start_stamp, start_yaw)]
    path.extend((t, wrap_pi(yaw)) for t, yaw in rows if start_stamp < t < end_stamp)
    path.append((end_stamp, end_yaw))
    unwrapped = [path[0][1]]
    for index in range(1, len(path)):
        gap = path[index][0] - path[index - 1][0]
        if gap <= 0.0 or gap > max_sample_gap_sec:
            return False
        unwrapped.append(unwrapped[-1] + wrap_pi(path[index][1] - path[index - 1][1]))

    yaw_delta = unwrapped[-1] - unwrapped[0]
    if abs(yaw_delta) > math.radians(max_yaw_deg):
        return False
    max_curve_error = max(
        abs(value - (unwrapped[0] + (row[0] - start_stamp) / span * yaw_delta))
        for row, value in zip(path, unwrapped)
    )
    if max_curve_error > math.radians(max_curve_yaw_deg):
        return False
    if (abs(wrap_pi(first.yaw - start_yaw)) > math.radians(max_endpoint_yaw_error_deg) or
            abs(wrap_pi(second.yaw - end_yaw)) > math.radians(max_endpoint_yaw_error_deg)):
        return False

    dx, dy = second.x - first.x, second.y - first.y
    if math.hypot(dx, dy) > max_translation_m:
        return False
    forward = math.cos(first.yaw) * dx + math.sin(first.yaw) * dy
    lateral = -math.sin(first.yaw) * dx + math.cos(first.yaw) * dy
    return forward >= -0.02 and abs(lateral) <= max_lateral_m


def imu_stationary_resample_safe(
    first, second, start_stamp, end_stamp, samples,
    max_span_sec=3.0, max_translation_m=0.05, max_yaw_deg=2.0,
    max_curve_yaw_deg=0.5, max_sample_gap_sec=0.12,
    max_endpoint_yaw_error_deg=4.0,
):
    """Require a continuous, nearly constant IMU heading during a stop.

    Unlike the moving-straight guard this tests displacement magnitude only;
    a tiny reverse or lateral settling motion is not a direction violation.
    """
    span = end_stamp - start_stamp
    limits = (max_span_sec, max_translation_m, max_yaw_deg,
              max_curve_yaw_deg, max_sample_gap_sec,
              max_endpoint_yaw_error_deg)
    if (not math.isfinite(span) or span <= 0.0 or
            not all(math.isfinite(value) and value > 0.0 for value in limits) or
            span > max_span_sec or
            math.hypot(second.x - first.x, second.y - first.y) > max_translation_m):
        return False

    rows = sorted((float(t), float(yaw)) for t, yaw in samples
                  if math.isfinite(t) and math.isfinite(yaw) and
                  start_stamp - max_sample_gap_sec <= t <= end_stamp + max_sample_gap_sec)
    start_yaw = interpolate_imu_yaw(rows, start_stamp, max_sample_gap_sec)
    end_yaw = interpolate_imu_yaw(rows, end_stamp, max_sample_gap_sec)
    if start_yaw is None or end_yaw is None:
        return False
    path = [(start_stamp, start_yaw)]
    path.extend((t, wrap_pi(yaw)) for t, yaw in rows if start_stamp < t < end_stamp)
    path.append((end_stamp, end_yaw))
    unwrapped = [path[0][1]]
    for index in range(1, len(path)):
        gap = path[index][0] - path[index - 1][0]
        if gap <= 0.0 or gap > max_sample_gap_sec:
            return False
        unwrapped.append(unwrapped[-1] + wrap_pi(path[index][1] - path[index - 1][1]))
    yaw_delta = unwrapped[-1] - unwrapped[0]
    if abs(yaw_delta) > math.radians(max_yaw_deg):
        return False
    curve_error = max(
        abs(value - (unwrapped[0] + (row[0] - start_stamp) / span * yaw_delta))
        for row, value in zip(path, unwrapped)
    )
    if curve_error > math.radians(max_curve_yaw_deg):
        return False
    return (
        abs(wrap_pi(first.yaw - start_yaw)) <= math.radians(max_endpoint_yaw_error_deg)
        and abs(wrap_pi(second.yaw - end_yaw)) <= math.radians(max_endpoint_yaw_error_deg)
    )


def select_imu_resample_kind(endpoint_kind, stationary_safe=None, straight_safe=None):
    """Apply only the guard belonging to the original endpoint classification."""
    if endpoint_kind == "stationary":
        return "imu_stationary" if stationary_safe else "reject"
    if endpoint_kind == "reject":
        return "imu_straight" if straight_safe else "reject"
    return endpoint_kind


def imu_orientation_is_usable(covariance, quaternion):
    """Validate the ROS IMU orientation estimate before using its yaw."""
    if len(covariance) == 0 or not math.isfinite(float(covariance[0])) or covariance[0] < 0.0:
        return False
    if len(quaternion) != 4 or not all(math.isfinite(float(v)) for v in quaternion):
        return False
    norm_sq = sum(float(v) * float(v) for v in quaternion)
    return math.isfinite(norm_sq) and norm_sq > 1e-18


def use_imu_yaw_for_resample(kind):
    """Only moving-straight gaps may use interpolated IMU heading.

    Stationary IMU yaw is a hidden-turn safety gate; the pose yaw itself must
    remain interpolated from the accepted MonoScale endpoints.
    """
    return kind == "imu_straight"


def resampled_pose_sigmas(span, distance, kind):
    """Conservative 1-sigma process uncertainty for camera-time backfills."""
    if (not math.isfinite(span) or span <= 0.0 or
            not math.isfinite(distance) or distance < 0.0):
        raise ValueError("invalid resampling interval")
    if kind == "imu_stationary":
        return 0.01 + 0.01 * span, math.radians(0.15 + 0.10 * span)
    return 0.02 + 0.04 * span + 0.08 * distance, math.radians(0.25 + 0.50 * span)


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class VisionYawSample:
    stamp: float
    yaw_delta: float
    distance: float
    valid: bool
    yaw_variance: float = math.nan


@dataclass(frozen=True)
class YawCorrectionResult:
    pose: Pose2
    vision_used: bool
    vision_samples: int
    raw_yaw_delta: float
    vision_yaw_delta: float
    corrected_yaw_delta: float
    gyro_bias_rad_s: float
    reason: str


@dataclass(frozen=True)
class FilterResult:
    pose: Pose2
    accepted: bool
    reason: str
    stationary: bool
    raw_translation_m: float
    raw_yaw_rad: float


ESM_MARKER = -8.126e7


def parse_tracker_esm(data):
    """Read one front-tracker ESM increment without depending on ROS types.

    A returned sample can be invalid.  Recording invalid frames is important:
    silently omitting one would make a partial visual sum look like the whole
    rotation between two odometry updates.
    """
    if len(data) < 4:
        return None
    stamp = float(data[0])
    count = int(data[1])
    if not math.isfinite(stamp) or stamp <= 0.0 or count < 0:
        return None
    after = 4 + 5 * count
    valid = len(data) > after + 29 and data[after + 29] == ESM_MARKER
    yaw = float(data[after + 15]) if valid else 0.0
    # Upper triangle of covariance(step, yaw, pitch, roll): yaw/yaw is the
    # fifth entry.  Older/failed fits carry NaN here; they may still supply a
    # deliberately tiny correction, but a finite large variance is rejected.
    yaw_variance = float(data[after + 22]) if valid else math.nan
    distance = float(data[after]) if len(data) > after else 0.0
    valid = valid and math.isfinite(yaw) and math.isfinite(distance)
    return VisionYawSample(
        stamp, yaw if valid else 0.0, distance, valid, yaw_variance
    )


def interpolate_pose(first, second, ratio):
    ratio = min(max(float(ratio), 0.0), 1.0)
    return Pose2(
        first.x + ratio * (second.x - first.x),
        first.y + ratio * (second.y - first.y),
        wrap_pi(first.yaw + ratio * wrap_pi(second.yaw - first.yaw)),
    )


class VisionYawCorrector:
    """Correct slow gyro drift from front-camera ESM while preserving turns.

    The raw MonoScale pose already contains the short-term F9R rotation.  ESM
    is used in two deliberately limited ways: it learns gyro bias only while
    the visual turn rate is nearly straight, and contributes a small bounded
    fraction during a turn.  Raw x/y increments are rotated into the corrected
    heading so every downstream map receives one internally consistent pose.
    """

    def __init__(
        self,
        enabled=True,
        bias_tau_sec=30.0,
        bias_limit_rad_s=math.radians(0.35),
        straight_rate_rad_s=math.radians(3.0),
        straight_hold_sec=0.8,
        turn_blend=0.01,
        max_visual_adjustment_rad=math.radians(0.12),
        max_yaw_variance=math.radians(5.0) ** 2,
        max_innovation_rad=math.radians(12.0),
        max_visual_rate_rad_s=math.radians(140.0),
        min_linear_speed_mps=0.10,
        reset_gap_sec=1.5,
    ):
        positive = (
            bias_tau_sec,
            bias_limit_rad_s,
            straight_rate_rad_s,
            straight_hold_sec,
            max_visual_adjustment_rad,
            max_yaw_variance,
            max_innovation_rad,
            max_visual_rate_rad_s,
            min_linear_speed_mps,
            reset_gap_sec,
        )
        if not all(math.isfinite(v) and v > 0.0 for v in positive):
            raise ValueError("yaw-correction limits must be finite and positive")
        if not math.isfinite(turn_blend) or not 0.0 <= turn_blend <= 1.0:
            raise ValueError("turn_blend must be in [0, 1]")
        self.enabled = bool(enabled)
        self.bias_tau_sec = bias_tau_sec
        self.bias_limit_rad_s = bias_limit_rad_s
        self.straight_rate_rad_s = straight_rate_rad_s
        self.straight_hold_sec = straight_hold_sec
        self.turn_blend = turn_blend
        self.max_visual_adjustment_rad = max_visual_adjustment_rad
        self.max_yaw_variance = max_yaw_variance
        self.max_innovation_rad = max_innovation_rad
        self.max_visual_rate_rad_s = max_visual_rate_rad_s
        self.min_linear_speed_mps = min_linear_speed_mps
        self.reset_gap_sec = reset_gap_sec
        self._samples = deque(maxlen=1024)
        self._stamp = None
        self._raw = None
        self._corrected = None
        self._bias = 0.0
        self._straight_duration = 0.0

    @property
    def bias_rad_s(self):
        return self._bias

    def add_vision_sample(self, sample):
        if sample is None or not math.isfinite(sample.stamp):
            return
        if self._samples and sample.stamp <= self._samples[-1].stamp:
            return
        self._samples.append(sample)

    def update(self, stamp, raw):
        if not math.isfinite(stamp) or not all(
            math.isfinite(v) for v in (raw.x, raw.y, raw.yaw)
        ):
            raise ValueError("yaw corrector requires a finite pose and stamp")
        raw = Pose2(raw.x, raw.y, wrap_pi(raw.yaw))
        if self._stamp is None:
            self._stamp, self._raw, self._corrected = stamp, raw, raw
            self._discard_through(stamp)
            return YawCorrectionResult(raw, False, 0, 0.0, 0.0, 0.0, self._bias, "initialized")

        dt = stamp - self._stamp
        if dt <= 0.0:
            return YawCorrectionResult(
                self._corrected, False, 0, 0.0, 0.0, 0.0, self._bias,
                "nonincreasing_stamp")

        samples = []
        while self._samples and self._samples[0].stamp <= stamp:
            sample = self._samples.popleft()
            if sample.stamp > self._stamp:
                samples.append(sample)
        valid = bool(samples) and all(sample.valid for sample in samples)
        vision_delta = sum(sample.yaw_delta for sample in samples) if valid else 0.0
        finite_variances = [
            sample.yaw_variance for sample in samples
            if math.isfinite(sample.yaw_variance)
        ]
        raw_delta = wrap_pi(raw.yaw - self._raw.yaw)
        distance = math.hypot(raw.x - self._raw.x, raw.y - self._raw.y)
        innovation = wrap_pi(raw_delta - vision_delta)
        reason = "no_vision"

        if dt > self.reset_gap_sec:
            valid = False
            reason = "gap"
        elif valid and (
            abs(innovation) > self.max_innovation_rad
            or abs(vision_delta) / dt > self.max_visual_rate_rad_s
            or (finite_variances and max(finite_variances) > self.max_yaw_variance)
        ):
            valid = False
            reason = "vision_gate"

        if self.enabled and valid:
            reason = "turn_blend"
            speed = distance / dt
            both_straight = (
                abs(vision_delta) / dt <= self.straight_rate_rad_s
                and abs(raw_delta) / dt <= self.straight_rate_rad_s
            )
            if speed >= self.min_linear_speed_mps and both_straight:
                self._straight_duration += dt
            else:
                self._straight_duration = 0.0
            if self._straight_duration >= self.straight_hold_sec:
                target = max(
                    -self.bias_limit_rad_s,
                    min(self.bias_limit_rad_s, innovation / dt),
                )
                alpha = 1.0 - math.exp(-dt / self.bias_tau_sec)
                self._bias += alpha * (target - self._bias)
                reason = "bias_update"
            elif both_straight:
                reason = "straight_hold"
        else:
            self._straight_duration = 0.0

        corrected_delta = raw_delta
        if self.enabled:
            corrected_delta = raw_delta - self._bias * dt
            if valid:
                visual_adjustment = self.turn_blend * wrap_pi(
                    vision_delta - corrected_delta
                )
                visual_adjustment = max(
                    -self.max_visual_adjustment_rad,
                    min(self.max_visual_adjustment_rad, visual_adjustment),
                )
                corrected_delta = wrap_pi(
                    corrected_delta + visual_adjustment
                )

        # MonoScale's x/y increment is already in the world described by its
        # previous raw yaw. Rotate that same displacement into the corrected
        # world before accumulating it; changing yaw alone would leave a bent
        # trajectory underneath a straight-looking quaternion.
        heading_offset = wrap_pi(self._corrected.yaw - self._raw.yaw)
        c, s = math.cos(heading_offset), math.sin(heading_offset)
        dx, dy = raw.x - self._raw.x, raw.y - self._raw.y
        corrected = Pose2(
            self._corrected.x + c * dx - s * dy,
            self._corrected.y + s * dx + c * dy,
            wrap_pi(self._corrected.yaw + corrected_delta),
        )
        self._stamp, self._raw, self._corrected = stamp, raw, corrected
        return YawCorrectionResult(
            corrected, self.enabled and valid, len(samples), raw_delta,
            vision_delta, corrected_delta, self._bias, reason)

    def _discard_through(self, stamp):
        while self._samples and self._samples[0].stamp <= stamp:
            self._samples.popleft()


class LaneOdomFilter:
    """Reject impossible pose jumps and smooth only near-stationary jitter.

    Moving poses pass through without a general low-pass delay.  When the raw
    pose is nearly stationary, an exponential filter removes the vibration
    that otherwise creates false map keyframes.  IMU data is deliberately not
    accepted here: x/y remain whatever MonoScale's visual solve produced, and
    yaw remains MonoScale's selected heading source.
    """

    def __init__(
        self,
        max_linear_speed_mps=5.0,
        max_angular_speed_rad_s=math.radians(120.0),
        max_translation_step_m=0.30,
        max_yaw_step_rad=math.radians(10.0),
        stationary_linear_speed_mps=0.08,
        stationary_angular_speed_rad_s=math.radians(3.0),
        stationary_tau_sec=0.20,
        reset_gap_sec=0.75,
    ):
        values = (
            max_linear_speed_mps,
            max_angular_speed_rad_s,
            max_translation_step_m,
            max_yaw_step_rad,
            stationary_linear_speed_mps,
            stationary_angular_speed_rad_s,
            stationary_tau_sec,
            reset_gap_sec,
        )
        if not all(math.isfinite(v) and v > 0.0 for v in values):
            raise ValueError("all odom-filter limits must be finite and positive")
        self.max_linear_speed_mps = max_linear_speed_mps
        self.max_angular_speed_rad_s = max_angular_speed_rad_s
        self.max_translation_step_m = max_translation_step_m
        self.max_yaw_step_rad = max_yaw_step_rad
        self.stationary_linear_speed_mps = stationary_linear_speed_mps
        self.stationary_angular_speed_rad_s = stationary_angular_speed_rad_s
        self.stationary_tau_sec = stationary_tau_sec
        self.reset_gap_sec = reset_gap_sec
        self._stamp = None
        self._input_stamp = None
        self._raw = None
        self._output = None

    def update(self, stamp_sec, pose):
        if not math.isfinite(stamp_sec) or not all(
            math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)
        ):
            return self._rejected("nonfinite")
        pose = Pose2(pose.x, pose.y, wrap_pi(pose.yaw))
        if self._stamp is None:
            self._stamp = stamp_sec
            self._input_stamp = stamp_sec
            self._raw = pose
            self._output = pose
            return FilterResult(pose, True, "initialized", True, 0.0, 0.0)

        input_dt = stamp_sec - self._input_stamp
        if input_dt <= 0.0:
            return self._rejected("nonincreasing_stamp")
        self._input_stamp = stamp_sec
        dx = pose.x - self._raw.x
        dy = pose.y - self._raw.y
        dyaw = wrap_pi(pose.yaw - self._raw.yaw)
        distance = math.hypot(dx, dy)

        # A reset is allowed only after the INPUT actually disappeared.  The
        # previous code measured time since the last accepted pose instead, so
        # a continuous stream of rejected yaw samples was accepted wholesale
        # after 0.75 s, bypassing every jump limit.
        if input_dt > self.reset_gap_sec:
            self._stamp = stamp_sec
            self._raw = pose
            self._output = pose
            return FilterResult(pose, True, "gap_reinitialized", False, distance, dyaw)

        # Limits describe one received sample.  They must not expand just
        # because preceding samples were rejected.
        translation_limit = max(self.max_translation_step_m, self.max_linear_speed_mps * input_dt)
        yaw_limit = max(self.max_yaw_step_rad, self.max_angular_speed_rad_s * input_dt)
        if distance > translation_limit:
            return self._rejected("translation_jump", distance, dyaw)
        if abs(dyaw) > yaw_limit:
            return self._rejected("yaw_jump", distance, dyaw)

        linear_speed = distance / input_dt
        angular_speed = abs(dyaw) / input_dt
        stationary = (
            linear_speed <= self.stationary_linear_speed_mps
            and angular_speed <= self.stationary_angular_speed_rad_s
        )
        if stationary:
            alpha = 1.0 - math.exp(-input_dt / self.stationary_tau_sec)
            filtered = Pose2(
                self._output.x + alpha * (pose.x - self._output.x),
                self._output.y + alpha * (pose.y - self._output.y),
                wrap_pi(self._output.yaw + alpha * wrap_pi(pose.yaw - self._output.yaw)),
            )
        else:
            # Avoid systematic pose lag while the vehicle is actually moving.
            filtered = pose

        self._stamp = stamp_sec
        self._raw = pose
        self._output = filtered
        return FilterResult(filtered, True, "accepted", stationary, distance, dyaw)

    def _rejected(self, reason, distance=0.0, dyaw=0.0):
        fallback = self._output or Pose2(0.0, 0.0, 0.0)
        return FilterResult(fallback, False, reason, False, distance, dyaw)



def covariance_matrix_is_usable(covariance):
    """Check a finite, symmetric PSD ROS 6x6 pose covariance."""
    if len(covariance) != 36:
        return False
    values = [float(value) for value in covariance]
    if not all(math.isfinite(value) for value in values):
        return False
    scale = max(1.0, max(abs(value) for value in values))
    matrix = [[0.0] * 6 for _ in range(6)]
    for row in range(6):
        for column in range(6):
            a, b = values[row * 6 + column], values[column * 6 + row]
            if abs(a - b) > 1e-9 * scale:
                return False
            matrix[row][column] = 0.5 * (a + b)
    # MonoScale leaves the ROS covariance array zero when it has no estimate.
    if any(matrix[index][index] <= 0.0 for index in (0, 1, 5)):
        return False

    # Dependency-free symmetric Jacobi eigenvalue check (matrix is only 6x6).
    for _ in range(720):
        p, q, largest = 0, 1, 0.0
        for row in range(6):
            for column in range(row + 1, 6):
                magnitude = abs(matrix[row][column])
                if magnitude > largest:
                    p, q, largest = row, column, magnitude
        if largest <= 1e-12 * scale:
            break
        apq = matrix[p][q]
        tau = (matrix[q][q] - matrix[p][p]) / (2.0 * apq)
        tangent = (1.0 if tau >= 0.0 else -1.0) / (
            abs(tau) + math.sqrt(1.0 + tau * tau))
        cosine = 1.0 / math.sqrt(1.0 + tangent * tangent)
        sine = tangent * cosine
        matrix[p][p] -= tangent * apq
        matrix[q][q] += tangent * apq
        matrix[p][q] = matrix[q][p] = 0.0
        for index in range(6):
            if index in (p, q):
                continue
            aip, aiq = matrix[index][p], matrix[index][q]
            matrix[index][p] = matrix[p][index] = cosine*aip - sine*aiq
            matrix[index][q] = matrix[q][index] = sine*aip + cosine*aiq
    return min(matrix[index][index] for index in range(6)) >= -1e-10 * scale


def interpolate_pose_covariance(previous, current, ratio, span, distance, kind):
    """Return a diagonal covariance bounding both endpoint matrices."""
    try:
        ratio = float(ratio)
    except (TypeError, ValueError):
        ratio = float("nan")
    if (not math.isfinite(ratio) or
            not covariance_matrix_is_usable(previous) or
            not covariance_matrix_is_usable(current)):
        result = [0.0] * 36
        result[0] = -1.0
        return result
    ratio = min(max(ratio, 0.0), 1.0)
    sigma_xy, sigma_yaw = resampled_pose_sigmas(span, distance, kind)
    # A convex blend is PSD but can claim less uncertainty than an endpoint.
    # Diagonal row-sum bounds dominate both symmetric endpoint matrices.
    result = [0.0] * 36
    for row in range(6):
        previous_bound = sum(abs(float(previous[row*6 + column]))
                             for column in range(6))
        current_bound = sum(abs(float(current[row*6 + column]))
                            for column in range(6))
        result[row*6 + row] = max(previous_bound, current_bound)
    result[0] += sigma_xy*sigma_xy
    result[7] += sigma_xy*sigma_xy
    result[35] += sigma_yaw*sigma_yaw
    return result


# Pose provenance (vision_lane_interfaces/msg/PoseProvenance.msg kinds).
INTERPOLATED_KINDS = ("short", "moving", "stationary", "straight")
IMU_KINDS = ("imu_fill", "imu_straight", "imu_stationary")


def pose_provenance_kind(resample_kind, is_resampled):
    """'raw' for a passthrough, 'interpolated' for a blend of two raw poses,
    'imu_fill' when IMU yaw shaped it, 'unknown' for anything else."""
    if not is_resampled:
        return "raw"
    if resample_kind in INTERPOLATED_KINDS:
        return "interpolated"
    if resample_kind in IMU_KINDS:
        return "imu_fill"
    return "unknown"


def unknown_pose_covariance():
    """ROS convention for an unknown pose covariance: element 0 = -1, the rest 0."""
    covariance = [0.0] * 36
    covariance[0] = -1.0
    return covariance
