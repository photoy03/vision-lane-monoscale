"""ROS-free filter used to make MonoScale poses safe for lane accumulation."""

from dataclasses import dataclass
import math


def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class FilterResult:
    pose: Pose2
    accepted: bool
    reason: str
    stationary: bool
    raw_translation_m: float
    raw_yaw_rad: float


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
