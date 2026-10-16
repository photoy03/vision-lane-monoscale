#include "monoscale_occupancy_grid_map/sweep.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

namespace
{

monoscale_occupancy::Lens measured_lens()
{
  monoscale_occupancy::Lens lens;
  lens.fx = 1124.8539494230718;
  lens.fy = 1127.7212520984583;
  lens.cx = 1239.9860488395623;
  lens.cy = 653.1232644848533;
  lens.distortion = {
    -0.026746544589302992, 0.025439551135889072,
    -0.020583591860473614, 0.0010514108516609898};
  return lens;
}

TEST(FisheyeGeometry, ProjectionMatchesOpenCvCalibration)
{
  const auto lens = measured_lens();
  const std::vector<cv::Point3d> points{
    {0.0, 0.0, 1.0}, {0.3, -0.2, 1.0}, {0.8, 0.1, 0.7}, {-0.9, 0.4, 0.5}};
  cv::Mat k = (cv::Mat_<double>(3, 3) <<
    lens.fx, 0.0, lens.cx, 0.0, lens.fy, lens.cy, 0.0, 0.0, 1.0);
  cv::Mat d(4, 1, CV_64F);
  for (int i = 0; i < 4; ++i) {d.at<double>(i) = lens.distortion[i];}
  std::vector<cv::Point2d> expected;
  cv::fisheye::projectPoints(points, expected, cv::Vec3d(), cv::Vec3d(), k, d);
  for (size_t i = 0; i < points.size(); ++i) {
    const auto actual = monoscale_occupancy::project_fisheye(
      lens, Eigen::Vector3d(points[i].x, points[i].y, points[i].z));
    EXPECT_NEAR(actual.x(), expected[i].x, 1e-8);
    EXPECT_NEAR(actual.y(), expected[i].y, 1e-8);
  }
}

TEST(FisheyeGeometry, PixelRayRoundTripIncludesDistortion)
{
  const auto lens = measured_lens();
  const std::vector<Eigen::Vector2d> pixels{
    {lens.cx, lens.cy}, {100.0, 100.0}, {2400.0, 1200.0}, {1700.0, 300.0}};
  for (const auto & pixel : pixels) {
    const auto ray = monoscale_occupancy::unproject_fisheye(lens, pixel.x(), pixel.y());
    EXPECT_NEAR(ray.norm(), 1.0, 1e-10);
    const auto reconstructed = monoscale_occupancy::project_fisheye(lens, ray);
    EXPECT_NEAR(reconstructed.x(), pixel.x(), 1e-6);
    EXPECT_NEAR(reconstructed.y(), pixel.y(), 1e-6);
  }
}

}  // namespace
