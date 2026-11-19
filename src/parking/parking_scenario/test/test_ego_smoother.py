import math
import random

from parking_scenario.display_filter import EgoSmoother


def drive(noise, smoother):
    """1.2 m/s along x, poses at 10 Hz with position noise, picture at 30 Hz. -> frame-to-frame jerk rms, max error."""
    rng = random.Random(1)
    shown, t_pose = [], 0.0
    for frame in range(300):
        now = frame / 30.0
        while t_pose <= now:
            smoother.add(t_pose, 1.2 * t_pose + rng.gauss(0, noise), rng.gauss(0, noise), 0.0)
            t_pose += 0.1
        shown.append((now,) + smoother.at(now))
    xs = [p[1] for p in shown[60:]]
    jerk = [c - 2 * b + a for a, b, c in zip(xs, xs[1:], xs[2:])]
    err = max(abs(p[1] - 1.2 * p[0]) for p in shown[60:])
    return math.sqrt(sum(j * j for j in jerk) / len(jerk)), err


def test_noisy_pose_gives_a_steady_picture_without_lag():
    jerk, err = drive(0.03, EgoSmoother(0.15))
    assert jerk < 0.010 and err < 0.08


def test_exact_pose_is_followed_exactly():
    jerk, err = drive(0.0, EgoSmoother(0.15))
    assert jerk < 1e-6 and err < 1e-6
