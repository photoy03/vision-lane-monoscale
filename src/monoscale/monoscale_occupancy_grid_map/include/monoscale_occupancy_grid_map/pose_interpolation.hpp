#pragma once

#include <algorithm>
#include <cmath>

namespace monoscale_occupancy
{

// A published grid contains all evidence committed so far. With multiple
// cameras, an older keyframe can finish after a newer one, but that must not
// move the grid header backwards in time. Invalid candidates leave the
// current evidence stamp unchanged.
inline double monotonic_evidence_stamp(double current, double candidate)
{
  if (!std::isfinite(candidate) || candidate <= 0.0) {return current;}
  if (!std::isfinite(current) || current <= 0.0) {return candidate;}
  return std::max(current, candidate);
}

// Pose-at-image-stamp interpolation (gap/translation/yaw bounded) now lives in
// the shared monoscale_pose/pose_timeline.hpp (PoseTimeline::lookup_bounded).

}  // namespace monoscale_occupancy
