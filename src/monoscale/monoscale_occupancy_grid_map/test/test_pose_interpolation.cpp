#include <gtest/gtest.h>
#include <limits>

#include "monoscale_occupancy_grid_map/pose_interpolation.hpp"

using monoscale_occupancy::Pose5;
using monoscale_occupancy::PoseInterpolationReject;
using monoscale_occupancy::interpolate_pose_bounded;
using monoscale_occupancy::monotonic_evidence_stamp;

TEST(EvidenceStamp, NeverRegressesAcrossCameraCompletionOrder)
{
  EXPECT_DOUBLE_EQ(monotonic_evidence_stamp(0.0, 10.0), 10.0);
  EXPECT_DOUBLE_EQ(monotonic_evidence_stamp(10.0, 10.2), 10.2);
  EXPECT_DOUBLE_EQ(monotonic_evidence_stamp(10.2, 10.17), 10.2);
  EXPECT_DOUBLE_EQ(monotonic_evidence_stamp(10.2, 0.0), 10.2);
  EXPECT_DOUBLE_EQ(monotonic_evidence_stamp(
    10.2, std::numeric_limits<double>::quiet_NaN()), 10.2);
}

TEST(PoseInterpolation, AcceptsSmallMotionAcrossBoundedLongerGap)
{
  Pose5 a{0.0,0.0,0.0,0.0,0.0};
  Pose5 b{0.06,0.0,0.02,0.01,-0.01};
  const auto result=interpolate_pose_bounded(a,b,0.5,0.35,0.40,0.10,3.0);
  ASSERT_TRUE(result.pose.has_value());
  EXPECT_EQ(result.rejection,PoseInterpolationReject::none);
  EXPECT_NEAR(result.pose->x,0.03,1e-12);
  EXPECT_NEAR(result.pose->yaw,0.01,1e-12);
  EXPECT_NEAR(result.pose->roll,0.005,1e-12);
  EXPECT_NEAR(result.pose->pitch,-0.005,1e-12);
}

TEST(PoseInterpolation, RejectsLargeTimeGapTranslationAndYaw)
{
  const Pose5 a{};
  EXPECT_EQ(interpolate_pose_bounded(a,a,0.5,0.41,0.40,0.10,3.0).rejection,
    PoseInterpolationReject::gap);
  EXPECT_EQ(interpolate_pose_bounded(a,Pose5{0.11,0,0,0,0},0.5,0.30,0.40,0.10,3.0).rejection,
    PoseInterpolationReject::translation);
  EXPECT_EQ(interpolate_pose_bounded(a,Pose5{0,0,4.0*3.14159265358979323846/180.0,0,0},
    0.5,0.30,0.40,0.10,3.0).rejection,PoseInterpolationReject::yaw);
}

TEST(PoseInterpolation, InterpolatesYawAcrossPiByShortestArc)
{
  constexpr double kPi=3.14159265358979323846;
  const Pose5 a{0,0,179.0*kPi/180.0,0,0};
  const Pose5 b{0.02,0,-179.0*kPi/180.0,0,0};
  const auto result=interpolate_pose_bounded(a,b,0.5,0.20,0.40,0.10,3.0);
  ASSERT_TRUE(result.pose.has_value());
  EXPECT_NEAR(std::abs(result.pose->yaw),kPi,1e-12);
}

TEST(PoseInterpolation, RejectsInvalidLimitsAndNonfinitePoses)
{
  const Pose5 a{};
  EXPECT_EQ(interpolate_pose_bounded(a,a,1.1,0.1,0.4,0.1,3.0).rejection,
    PoseInterpolationReject::invalid);
  Pose5 bad{};bad.pitch=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(interpolate_pose_bounded(a,bad,0.5,0.1,0.4,0.1,3.0).rejection,
    PoseInterpolationReject::invalid);
}
