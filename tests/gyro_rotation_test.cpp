#include <cassert>
#include <cmath>
#include <deque>
#include <string>

#include <monoscale_tracker/gyro_rotation.hpp>

int main()
{
  using monoscale_tracker::GyroSample;
  using monoscale_tracker::camera_ray_rotation;
  using monoscale_tracker::integrate_gyro;
  const std::deque<GyroSample> samples{
    {0.0, {0.0, 0.0, 1.0}},
    {0.1, {0.0, 0.0, 1.0}},
    {0.2, {0.0, 0.0, 1.0}}};
  cv::Matx33d r;
  std::string reason;
  int intervals = -1;
  assert(integrate_gyro(samples, 0.0, 0.2, 0.11, 0.02, r, reason, intervals));
  assert(intervals == 2 && reason == "OK");
  // Positive body yaw moves a fixed scene bearing clockwise in the camera.
  const cv::Vec3d ray = camera_ray_rotation(cv::Matx33d::eye(), r) *
    cv::Vec3d(1.0, 0.0, 0.0);
  assert(std::abs(ray[0] - std::cos(0.2)) < 1e-9);
  assert(std::abs(ray[1] + std::sin(0.2)) < 1e-9);
  assert(!integrate_gyro(samples, -0.01, 0.1, 0.11, 0.02, r, reason, intervals));
  assert(reason == "BEFORE_BUFFER");
  assert(!integrate_gyro(samples, 0.0, 0.3, 0.11, 0.02, r, reason, intervals));
  assert(reason == "AFTER_BUFFER");
  assert(!integrate_gyro(samples, 0.0, 0.2, 0.05, 0.02, r, reason, intervals));
  assert(reason == "INTERNAL_GAP");
}
