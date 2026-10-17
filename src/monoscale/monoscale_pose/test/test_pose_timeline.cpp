#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "monoscale_pose/pose_timeline.hpp"

using monoscale_pose::MotionGates;
using monoscale_pose::PoseMatchKind;
using monoscale_pose::PoseTimeline;
using monoscale_pose::TimedPose;

namespace
{
constexpr int64_t kSecond = 1000000000;
constexpr double kPi = 3.14159265358979323846;
const MotionGates kOccupancyGates{0.40, 0.10, 3.0};

TimedPose pose(int64_t stamp, double x, double y = 0.0, double yaw = 0.0,
  double roll = 0.0, double pitch = 0.0)
{
  TimedPose p{stamp, x, y, yaw};
  p.roll = roll;
  p.pitch = pitch;
  return p;
}
}  // namespace

TEST(Lookup, InterpolatesInsideGapAndAgeLimits)
{
  PoseTimeline poses(14.0);
  ASSERT_TRUE(poses.insert(pose(10 * kSecond, 0.0)));
  ASSERT_TRUE(poses.insert(pose(10 * kSecond + kSecond / 5, 1.0)));
  const auto mid = poses.lookup(10 * kSecond + kSecond / 10, 0.15, 0.25, false);
  ASSERT_TRUE(mid.valid);
  EXPECT_EQ(mid.kind, PoseMatchKind::interpolated);
  EXPECT_NEAR(mid.pose.x, 0.5, 1e-12);
  EXPECT_EQ(poses.lookup(11 * kSecond, 0.15, 0.25, false).kind, PoseMatchKind::stamp_after_latest);
}

TEST(Lookup, CapacityIsConfigurable)
{
  PoseTimeline poses(1e6, 3);
  for (int i = 1; i <= 5; ++i) {ASSERT_TRUE(poses.insert(pose(i * kSecond, i)));}
  EXPECT_EQ(poses.size(), 3u);
  EXPECT_EQ(poses.oldest_stamp_ns(), 3 * kSecond);
}

TEST(LookupBounded, ExactAndOutsideTheHistory)
{
  PoseTimeline poses(1e6, 4000);
  EXPECT_EQ(poses.lookup_bounded(kSecond, kOccupancyGates).kind, PoseMatchKind::buffer_empty);
  poses.insert(pose(10 * kSecond, 1.0));
  poses.insert(pose(10 * kSecond + kSecond / 5, 1.05));
  const auto exact = poses.lookup_bounded(10 * kSecond, kOccupancyGates);
  ASSERT_TRUE(exact.valid);
  EXPECT_EQ(exact.kind, PoseMatchKind::exact);
  EXPECT_DOUBLE_EQ(exact.pose.x, 1.0);
  EXPECT_EQ(poses.lookup_bounded(9 * kSecond, kOccupancyGates).kind, PoseMatchKind::stamp_before_oldest);
  EXPECT_EQ(poses.lookup_bounded(11 * kSecond, kOccupancyGates).kind, PoseMatchKind::stamp_after_latest);
}

// The cases below are the former monoscale_occupancy interpolate_pose_bounded tests.
TEST(LookupBounded, AcceptsSmallMotionAcrossBoundedLongerGap)
{
  PoseTimeline poses(1e6, 4000);
  poses.insert(pose(kSecond, 0.0));
  poses.insert(pose(kSecond + 350000000, 0.06, 0.0, 0.02, 0.01, -0.01));
  const auto result = poses.lookup_bounded(kSecond + 175000000, kOccupancyGates);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.kind, PoseMatchKind::interpolated);
  EXPECT_NEAR(result.pose.x, 0.03, 1e-12);
  EXPECT_NEAR(result.pose.yaw, 0.01, 1e-12);
  EXPECT_NEAR(result.pose.roll, 0.005, 1e-12);
  EXPECT_NEAR(result.pose.pitch, -0.005, 1e-12);
}

TEST(LookupBounded, RejectsLargeTimeGapTranslationAndYaw)
{
  PoseTimeline gap(1e6, 4000);
  gap.insert(pose(kSecond, 0.0));
  gap.insert(pose(kSecond + 410000000, 0.0));
  EXPECT_EQ(gap.lookup_bounded(kSecond + 1000, kOccupancyGates).kind, PoseMatchKind::gap_too_large);
  PoseTimeline moved(1e6, 4000);
  moved.insert(pose(kSecond, 0.0));
  moved.insert(pose(kSecond + 300000000, 0.11));
  EXPECT_EQ(moved.lookup_bounded(kSecond + 1000, kOccupancyGates).kind,
    PoseMatchKind::translation_too_large);
  PoseTimeline turned(1e6, 4000);
  turned.insert(pose(kSecond, 0.0));
  turned.insert(pose(kSecond + 300000000, 0.0, 0.0, 4.0 * kPi / 180.0));
  EXPECT_EQ(turned.lookup_bounded(kSecond + 1000, kOccupancyGates).kind,
    PoseMatchKind::yaw_too_large);
}

TEST(LookupBounded, InterpolatesYawAcrossPiByShortestArc)
{
  PoseTimeline poses(1e6, 4000);
  poses.insert(pose(kSecond, 0.0, 0.0, 179.0 * kPi / 180.0));
  poses.insert(pose(kSecond + 200000000, 0.02, 0.0, -179.0 * kPi / 180.0));
  const auto result = poses.lookup_bounded(kSecond + 100000000, kOccupancyGates);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(std::abs(result.pose.yaw), kPi, 1e-12);
}

TEST(LookupBounded, RejectsInvalidLimitsAndNonfiniteAttitude)
{
  PoseTimeline poses(1e6, 4000);
  poses.insert(pose(kSecond, 0.0));
  poses.insert(pose(kSecond + 100000000, 0.0, 0.0, 0.0, 0.0,
    std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(poses.lookup_bounded(kSecond + 1000, kOccupancyGates).kind, PoseMatchKind::invalid);
  PoseTimeline ok(1e6, 4000);
  ok.insert(pose(kSecond, 0.0));
  ok.insert(pose(kSecond + 100000000, 0.0));
  EXPECT_EQ(ok.lookup_bounded(kSecond + 1000, MotionGates{0.4, 0.1, 181.0}).kind,
    PoseMatchKind::invalid);
  EXPECT_EQ(ok.lookup_bounded(kSecond + 1000, MotionGates{0.0, 0.1, 3.0}).kind,
    PoseMatchKind::invalid);
}
