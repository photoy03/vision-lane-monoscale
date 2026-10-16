#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include "monoscale_occupancy_grid_map/sweep.hpp"

namespace monoscale_occupancy
{

enum class PoseInterpolationReject
{
  none,
  invalid,
  gap,
  translation,
  yaw
};

struct PoseInterpolationResult
{
  std::optional<Pose5> pose;
  PoseInterpolationReject rejection{PoseInterpolationReject::invalid};
};

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

// Interpolate a camera-time pose only across a bounded, low-motion interval.
// A relaxed time ceiling is safe here only because translation and yaw change
// are also independently limited; rapid turns/gaps remain rejected.
inline PoseInterpolationResult interpolate_pose_bounded(
  const Pose5 & a, const Pose5 & b, double fraction, double span_sec,
  double max_gap_sec, double max_translation_m, double max_yaw_deg)
{
  const auto finite_pose=[](const Pose5 & p) {
    return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw)&&
      std::isfinite(p.roll)&&std::isfinite(p.pitch);
  };
  if(!finite_pose(a)||!finite_pose(b)||!std::isfinite(fraction)||
     !std::isfinite(span_sec)||!std::isfinite(max_gap_sec)||
     !std::isfinite(max_translation_m)||!std::isfinite(max_yaw_deg)||
     fraction<0.0||fraction>1.0||span_sec<=0.0||max_gap_sec<=0.0||
     max_translation_m<=0.0||max_yaw_deg<=0.0||max_yaw_deg>180.0)
    return {};
  if(span_sec>max_gap_sec) return {{},PoseInterpolationReject::gap};

  const double translation=std::hypot(b.x-a.x,b.y-a.y);
  if(!std::isfinite(translation)) return {};
  if(translation>max_translation_m) return {{},PoseInterpolationReject::translation};
  const double dyaw=std::atan2(std::sin(b.yaw-a.yaw),std::cos(b.yaw-a.yaw));
  if(!std::isfinite(dyaw)) return {};
  constexpr double kPi=3.14159265358979323846;
  if(std::abs(dyaw)>max_yaw_deg*kPi/180.0)
    return {{},PoseInterpolationReject::yaw};

  Pose5 result;
  result.x=a.x+(b.x-a.x)*fraction;
  result.y=a.y+(b.y-a.y)*fraction;
  result.yaw=a.yaw+dyaw*fraction;
  result.roll=a.roll+(b.roll-a.roll)*fraction;
  result.pitch=a.pitch+(b.pitch-a.pitch)*fraction;
  if(!finite_pose(result)) return {};
  return {result,PoseInterpolationReject::none};
}

}  // namespace monoscale_occupancy
