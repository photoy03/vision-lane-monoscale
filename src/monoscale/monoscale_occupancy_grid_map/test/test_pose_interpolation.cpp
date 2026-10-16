#include <gtest/gtest.h>
#include <limits>

#include "monoscale_occupancy_grid_map/pose_interpolation.hpp"

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

// The bounded pose interpolation tests moved with the code to monoscale_pose
// (test/test_pose_timeline.cpp, LookupBounded.*).
