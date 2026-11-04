#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "monoscale_occupancy_grid_map/camera_liveness.hpp"
#include "vision_lane_interfaces/pipeline_progress.hpp"

using monoscale_occupancy::CameraLiveness;

namespace
{
constexpr int64_t kSec = 1000000000LL;
const std::vector<std::string> kCameras{"front", "rear"};
}  // namespace

TEST(CameraLiveness, AliveUpToDeadSecByArrivalAndDeadPastIt)
{
  CameraLiveness live(true, 7 * kSec);
  live.arrive("front", 100 * kSec, 99 * kSec, 0);
  EXPECT_TRUE(live.alive("front", 107 * kSec));      // gap == dead_sec: alive
  EXPECT_FALSE(live.alive("front", 107 * kSec + 1));  // past it: dead
  // Arrival counts, not capture time: an image captured long ago that arrives
  // now keeps the camera alive.
  live.arrive("front", 110 * kSec, 95 * kSec, 0);
  EXPECT_TRUE(live.alive("front", 116 * kSec));
}

TEST(CameraLiveness, ADeadCameraIsLeftOutOfTheBound)
{
  CameraLiveness live(true, 7 * kSec);
  live.arrive("front", 10 * kSec, 10 * kSec, 0);
  live.arrive("rear", 10 * kSec, 10 * kSec, 0);
  std::vector<std::string> excluded;
  EXPECT_EQ(live.bound(kCameras, 10 * kSec, 0, &excluded), 10 * kSec);
  EXPECT_TRUE(excluded.empty());
  // The front stops; the rear goes on to 30 s.
  for (int64_t t = 11; t <= 30; ++t) {live.arrive("rear", t * kSec, t * kSec, 0);}
  EXPECT_EQ(live.bound(kCameras, 17 * kSec, 10 * kSec, &excluded), 10 * kSec);  // 7 s: still alive
  EXPECT_EQ(live.bound(kCameras, 30 * kSec, 10 * kSec, &excluded), 30 * kSec);
  ASSERT_EQ(excluded.size(), 1u);
  EXPECT_EQ(excluded[0], "front");
  EXPECT_EQ(live.deaths().at("front"), 1u);
}

TEST(CameraLiveness, AllDeadStandsStill)
{
  CameraLiveness live(true, 7 * kSec);
  live.arrive("front", 10 * kSec, 9 * kSec, 0);
  live.arrive("rear", 10 * kSec, 10 * kSec, 0);
  std::vector<std::string> excluded;
  EXPECT_EQ(live.bound(kCameras, 60 * kSec, 0, &excluded), 9 * kSec);  // min over all, as without liveness
  EXPECT_EQ(excluded.size(), 2u);
}

TEST(CameraLiveness, OffNeverExcludesNorDiscards)
{
  CameraLiveness live(false, 7 * kSec);
  live.arrive("front", 10 * kSec, 10 * kSec, 0);
  live.arrive("rear", 60 * kSec, 60 * kSec, 0);
  EXPECT_TRUE(live.alive("front", 60 * kSec));
  EXPECT_TRUE(live.arrive("front", 61 * kSec, 5 * kSec, 50 * kSec));
  EXPECT_EQ(live.late_after_revival(), 0u);
}

namespace
{
struct CutResult
{
  uint64_t regressions = 0;
  int64_t front_dropped = 0, dead_from = -1, alive_again = -1, published = 0;
  uint64_t late = 0, revivals = 0;
};

// Front cut from 20 s to 80 s; images every 0.1 s, arriving 0.05 s after
// capture; the front's stream runs `lag_ns` behind the rear's.
CutResult cut_front(int64_t lag_ns)
{
  CameraLiveness live(true, 7 * kSec);
  pipeline_progress::MonotonicWatermark watermark;
  std::vector<std::string> excluded;
  CutResult out;
  for (int64_t step = 0; step <= 1200; ++step) {  // 0..120 s
    const int64_t now = step * kSec / 10;
    live.arrive("rear", now, now - kSec / 20, out.published);
    if (now < 20 * kSec || now >= 80 * kSec) {
      if (!live.arrive("front", now, now - kSec / 20 - lag_ns, out.published)) {++out.front_dropped;}
    }
    out.published = watermark.update(live.bound(kCameras, now, out.published, &excluded));
    const bool front_out = !excluded.empty();
    if (front_out && out.dead_from < 0) {out.dead_from = now;}
    if (!front_out && out.dead_from >= 0 && out.alive_again < 0) {out.alive_again = now;}
  }
  out.regressions = watermark.regressions();
  out.late = live.late_after_revival();
  out.revivals = live.revivals().count("front") ? live.revivals().at("front") : 0;
  return out;
}
}  // namespace

TEST(CameraLiveness, ARevivedCameraLosesOnlyImagesBehindAndTheBoundNeverGoesBack)
{
  // The front a little behind the rear (0.15 s): its first image(s) back are
  // behind the bound the rear carried on, and are the only loss.
  const auto r = cut_front(15 * kSec / 100);
  EXPECT_EQ(r.regressions, 0u);
  EXPECT_LE(r.front_dropped, 2);
  EXPECT_GE(r.front_dropped, 1);
  EXPECT_EQ(static_cast<int64_t>(r.late), r.front_dropped);
  // Declared dead dead_sec after the last arrival, counted again with its first image.
  EXPECT_GT(r.dead_from, 27 * kSec - kSec / 5);
  EXPECT_LT(r.dead_from, 27 * kSec + kSec / 5);
  EXPECT_EQ(r.alive_again, 80 * kSec);
  EXPECT_EQ(r.revivals, 1u);
  // The bound kept moving through the cut: about the rear's newest at the end.
  EXPECT_GT(r.published, 119 * kSec);
}

TEST(CameraLiveness, TheLossOnRevivalIsTheLagOfTheRevivedStream)
{
  // 0.3 s behind at 10 images/s: three images back are behind the bound, and
  // the bound holds while the camera catches up, so it still never goes back.
  const auto r = cut_front(3 * kSec / 10);
  EXPECT_EQ(r.regressions, 0u);
  EXPECT_EQ(r.front_dropped, 3);
  EXPECT_EQ(r.revivals, 1u);
}
