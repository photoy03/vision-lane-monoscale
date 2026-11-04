// The estimator driven end to end on a synthetic drive.
//
// The Python equivalent (test_estimator_integration.py) renders images and
// feeds the optical-flow front end. This path takes tracks, so the drive is
// built the same way but stops one step earlier: ground points are placed in
// the world, projected through the shipped calibration into both cameras, and
// handed over as the tracker would hand them over. What is being checked is
// that the extrinsics, the intrinsics, the lens model, the anchor map and the
// motion solve agree with each other -- each of which is tested alone
// elsewhere, and none of which proves the assembly.

#include <cmath>
#include <deque>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "monoscale_core/estimator.hpp"

using monoscale::CameraSettings;
using monoscale::Estimator;
using monoscale::EstimatorSettings;
using monoscale::Identities;
using monoscale::ImuSample;
using monoscale::Lens;
using monoscale::Points2;
using monoscale::Pose2;
using monoscale::TrackFrame;

namespace
{

constexpr int kTrackWidth = 1280;
constexpr int kTrackHeight = 720;
constexpr double kGravity = 9.81;

Eigen::Matrix3d rotation_of(const std::vector<double> & flat)
{
  Eigen::Matrix3d rotation;
  rotation << flat[0], flat[1], flat[2], flat[3], flat[4], flat[5], flat[6], flat[7],
    flat[8];
  return rotation;
}

// The assembled fisheye pair, as vision_fisheye.param.yaml describes it.
CameraSettings front_camera()
{
  CameraSettings camera;
  camera.name = "front";
  camera.rotation_base_from_camera =
    rotation_of({0.0, -0.5, 0.8660254, -1.0, 0.0, 0.0, 0.0, -0.8660254, -0.5});
  camera.translation_base_from_camera = Eigen::Vector3d(3.694, 0.0, 0.89);
  camera.ground_min_distance_m = 0.6;
  camera.k << 1051.81, 0.0, 1279.5, 0.0, 1051.81, 719.5, 0.0, 0.0, 1.0;
  camera.distortion = Eigen::VectorXd::Zero(4);
  camera.lens = Lens::Equidistant;
  camera.calibration_width = 2560;
  camera.calibration_height = 1440;
  return camera;
}

CameraSettings rear_camera()
{
  CameraSettings camera = front_camera();
  camera.name = "rear";
  camera.rotation_base_from_camera =
    rotation_of({0.0, 0.5, -0.8660254, 1.0, 0.0, 0.0, 0.0, -0.8660254, -0.5});
  camera.translation_base_from_camera = Eigen::Vector3d(-0.82, 0.0, 1.26);
  camera.ground_min_distance_m = 0.0;
  return camera;
}

// The intrinsics as they apply to the frame the tracker measured pixels in.
Eigen::Matrix3d track_intrinsics(const CameraSettings & camera)
{
  Eigen::Matrix3d k = camera.k;
  k.row(0) *= static_cast<double>(kTrackWidth) / camera.calibration_width;
  k.row(1) *= static_cast<double>(kTrackHeight) / camera.calibration_height;
  return k;
}

// A ground point in base_link, as this camera would report it. Returns false
// when the point is behind the lens or outside the frame.
bool project(
  const CameraSettings & camera, const Eigen::Matrix3d & k,
  const Eigen::Vector3d & point_base, Eigen::Vector2d & pixel)
{
  const Eigen::Vector3d in_camera = camera.rotation_base_from_camera.transpose() *
    (point_base - camera.translation_base_from_camera);
  if (in_camera.z() <= 1e-6) {
    return false;
  }
  // Pinhole first, then bend it the way an equidistant lens does: the pinhole
  // radius is f*tan(theta), the fisheye radius is f*theta.
  const double nx = in_camera.x() / in_camera.z();
  const double ny = in_camera.y() / in_camera.z();
  const double radius = std::hypot(nx, ny);
  double scale = 1.0;
  if (radius > 1e-12) {
    scale = std::atan(radius) / radius;
  }
  pixel.x() = k(0, 0) * nx * scale + k(0, 2);
  pixel.y() = k(1, 1) * ny * scale + k(1, 2);
  return pixel.x() >= 0.0 && pixel.x() < kTrackWidth && pixel.y() >= 0.0 &&
         pixel.y() < kTrackHeight;
}

Eigen::Vector4d yaw_quaternion(double yaw)
{
  return Eigen::Vector4d(0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw));
}

// Points spread over the road, in the world frame. The third coordinate is a
// height: everything is on the road unless a test asks otherwise.
std::vector<Eigen::Vector3d> road_points()
{
  std::vector<Eigen::Vector3d> points;
  for (int i = -40; i <= 120; ++i) {
    for (int j = -12; j <= 12; ++j) {
      points.emplace_back(0.25 * i, 0.25 * j, 0.0);
    }
  }
  return points;
}

// The same road with something standing on it, off to one side of the path so
// the vehicle drives past rather than through.
std::vector<Eigen::Vector3d> road_with_an_obstacle(double height)
{
  std::vector<Eigen::Vector3d> points = road_points();
  for (int i = 0; i < 24; ++i) {
    for (int j = 0; j < 4; ++j) {
      points.emplace_back(6.0 + 0.1 * i, 1.6 + 0.1 * j, height);
    }
  }
  return points;
}

EstimatorSettings deployment_settings()
{
  EstimatorSettings settings;
  settings.cameras = {front_camera(), rear_camera()};
  settings.ground_max_distance_m = 8.0;
  settings.ground_ransac_threshold_m = 0.12;
  settings.ground_min_inliers = 24;
  settings.solve_trigger_disparity_px = 3.0;
  settings.mapping_min_period_sec = 1e9;
  settings.sync_tolerance_sec = 0.02;
  return settings;
}

struct Drive
{
  Estimator estimator;
  std::vector<Eigen::Vector3d> world;
  double stamp = 1.0;
  double travelled = 0.0;
  Pose2 truth;
  // How many cameras to feed. The estimator fuses a list rather than a pair,
  // so one is a supported rig and not a degenerate case.
  size_t cameras = 2;
  // Which cameras deliver this step (all when empty): a camera hole.
  std::vector<bool> active;
  // When a captured frame reaches the estimator (immediately when unset):
  // release(camera, captured, now). Models tracker output that lags capture.
  std::function<bool(size_t, double, double)> release;
  std::vector<std::deque<TrackFrame>> held{2};

  explicit Drive(const EstimatorSettings & settings)
  : estimator(settings), world(road_points()), cameras(settings.cameras.size()) {}

  // One camera frame at the current truth pose, for every camera. Withholding
  // the inertial samples models the instrument dropping out while the cameras
  // keep delivering, which is a live failure and not a recordable one.
  void step(double forward, double yaw_rate, double dt, bool feed_imu = true)
  {
    // The IMU runs at twice the camera rate, as the release kit does.
    for (int i = 0; feed_imu && i < 2; ++i) {
      ImuSample imu;
      imu.stamp = stamp + 0.5 * dt * i;
      imu.orientation = yaw_quaternion(truth.yaw + yaw_rate * 0.5 * dt * i);
      imu.angular_velocity = Eigen::Vector3d(0.0, 0.0, yaw_rate);
      imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, kGravity);
      estimator.ingest_imu(imu);
    }

    truth = truth.compose(
      monoscale::motion_from_twist(forward, 0.0, yaw_rate, dt));
    travelled += forward * dt;
    stamp += dt;

    for (size_t index = 0; index < cameras; ++index) {
      if (!active.empty() && !active[index]) {
        continue;
      }
      const CameraSettings camera = index == 0 ? front_camera() : rear_camera();
      const Eigen::Matrix3d k = track_intrinsics(camera);
      TrackFrame frame;
      frame.stamp = stamp;
      frame.width = kTrackWidth;
      frame.height = kTrackHeight;

      std::vector<int64_t> ids;
      std::vector<Eigen::Vector2d> pixels;
      const double c = std::cos(truth.yaw);
      const double s = std::sin(truth.yaw);
      for (size_t n = 0; n < world.size(); ++n) {
        // World into base_link at the truth pose.
        const double dx = world[n].x() - truth.x;
        const double dy = world[n].y() - truth.y;
        const Eigen::Vector3d in_base(
          c * dx + s * dy, -s * dx + c * dy, world[n].z());
        Eigen::Vector2d pixel;
        if (!project(camera, k, in_base, pixel)) {
          continue;
        }
        ids.push_back(static_cast<int64_t>(n));
        pixels.push_back(pixel);
      }

      const Eigen::Index count = static_cast<Eigen::Index>(ids.size());
      frame.ids.resize(count);
      frame.pixels.resize(count, 2);
      frame.previous_pixels.resize(count, 2);
      for (Eigen::Index n = 0; n < count; ++n) {
        frame.ids(n) = ids[static_cast<size_t>(n)];
        frame.pixels(n, 0) = pixels[static_cast<size_t>(n)].x();
        frame.pixels(n, 1) = pixels[static_cast<size_t>(n)].y();
        // The tracker reports where each feature was on the hop before. A
        // rough stand-in is enough: only the 90th percentile of the shift is
        // read, and only to decide when to solve.
        frame.previous_pixels(n, 0) = pixels[static_cast<size_t>(n)].x() - 4.0;
        frame.previous_pixels(n, 1) = pixels[static_cast<size_t>(n)].y();
      }
      if (!release) {
        estimator.ingest_tracks(index, frame);
      } else {
        held[index].push_back(std::move(frame));
      }
    }
    for (size_t index = 0; release && index < held.size(); ++index) {
      while (!held[index].empty() && release(index, held[index].front().stamp, stamp)) {
        estimator.ingest_tracks(index, held[index].front());
        held[index].pop_front();
      }
    }
  }
};

}  // namespace

TEST(EstimatorDrive, RecoversAStraightDriveInMetres)
{
  Drive drive(deployment_settings());

  // 2 m/s for four seconds at 30 Hz.
  for (int i = 0; i < 120; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }
  const auto updates = drive.estimator.take_updates();

  ASSERT_FALSE(updates.empty());
  const Pose2 & pose = drive.estimator.pose();
  ASSERT_GT(drive.estimator.diagnostics().map_aligned_frames, 0)
    << "the anchor map never answered, so this measured the fallback instead";

  // The pose is held at the origin until the map can answer, so what it has
  // travelled is short of the truth by that warm-up. What must not happen is a
  // scale error: whatever it did cover, it covered in metres.
  EXPECT_GT(pose.x, 0.5 * drive.travelled);
  EXPECT_LT(pose.x, drive.travelled + 0.5);
  EXPECT_NEAR(pose.y, 0.0, 0.3);
  EXPECT_NEAR(pose.yaw, 0.0, 0.02);
}

TEST(EstimatorDrive, ACurvedDriveDoesNotInventLateralVelocity)
{
  EstimatorSettings settings = deployment_settings();
  settings.twist_lowpass_tau = 0.0;
  Drive drive(settings);

  for (int i = 0; i < 180; ++i) {
    drive.step(2.0, 0.35, 1.0 / 30.0);
  }
  const auto updates = drive.estimator.take_updates();

  ASSERT_FALSE(updates.empty());
  const Eigen::Vector3d & twist = updates.back().twist;
  EXPECT_GT(twist.x(), 1.0);
  EXPECT_NEAR(twist.y(), 0.0, 0.05);
  EXPECT_NEAR(twist.z(), 0.35, 0.05);
}

TEST(EstimatorDrive, ARejectedMapAlignmentDoesNotMakeThePoseValid)
{
  EstimatorSettings settings = deployment_settings();
  settings.max_translation_per_frame_m = 1e-6;
  Drive drive(settings);

  for (int i = 0; i < 180; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }
  const auto updates = drive.estimator.take_updates();

  ASSERT_GT(drive.estimator.diagnostics().map_aligned_frames, 0);
  ASSERT_FALSE(updates.empty());
  for (const auto & update : updates) {
    EXPECT_FALSE(update.pose_valid);
  }
}

TEST(EstimatorDrive, AnInstrumentDropoutDelaysTheMotionRatherThanLosingIt)
{
  // The solve needs a heading prior from the instrument. When it cannot have
  // one the pair cannot be solved -- but the two camera frames bounding the
  // interval are perfectly good, and remembering the current one throws the
  // motion between them away permanently. Holding the solve frame instead lets
  // the next pair measure across the gap.
  //
  // A drive with a dropout in the middle must therefore end up where a drive
  // without one ends up. Discarding the interval does not merely lose the
  // 0.4 m covered during it: measured, a 0.2 s dropout put the old code 4.87 m
  // ahead of the clean run over 16 m of driving, because resynchronising the
  // solve frame leaves the map holding anchors the pose never caught up to.
  // Distance covered, and how many times the heading was missed.
  auto run = [](int gap_at) {
      Drive drive(deployment_settings());
      for (int i = 0; i < 240; ++i) {
        // imu_max_age_sec is 20 ms and the instrument runs at 60 Hz, so
        // withholding a few frames' worth puts every stamp out of reach.
        const bool feed = gap_at < 0 || i < gap_at || i >= gap_at + 6;
        drive.step(2.0, 0.0, 1.0 / 30.0, feed);
      }
      drive.estimator.take_updates();
      return std::pair<double, int64_t>(
        drive.estimator.pose().x, drive.estimator.diagnostics().imu_yaw_misses);
    };

  const auto clean = run(-1);
  const auto gapped = run(120);

  ASSERT_EQ(clean.second, 0);
  ASSERT_GT(gapped.second, 0)
    << "the dropout was not deep enough to miss the heading";
  // Held, the next solve spans the gap and the two drives agree to within a
  // solve's worth of travel.
  EXPECT_NEAR(gapped.first, clean.first, 0.15);
}

TEST(EstimatorDrive, AnExpiredImuReferenceResynchronisesAndSolvesAgain)
{
  EstimatorSettings settings = deployment_settings();
  // Make the live failure cheap to reproduce: a normal deployment retains 30
  // seconds, while this test deliberately ages the previous solve out after
  // half a second.
  settings.imu_history_sec = 0.5;
  Drive drive(settings);

  for (int i = 0; i < 120; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }
  ASSERT_GT(drive.estimator.diagnostics().map_aligned_frames, 0);
  const double before = drive.estimator.pose().x;

  // The vehicle is stationary but the F9R keeps publishing. No camera frames
  // arrive during this interval, so the heading at the held solve frame falls
  // out of the configured history window.
  for (int i = 0; i < 100; ++i) {
    ImuSample imu;
    imu.stamp = drive.stamp + 0.01 * i;
    imu.orientation = yaw_quaternion(drive.truth.yaw);
    imu.angular_velocity = Eigen::Vector3d::Zero();
    imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, kGravity);
    drive.estimator.ingest_imu(imu);
  }
  drive.stamp += 1.0;

  for (int i = 0; i < 60; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }
  drive.estimator.take_updates();

  EXPECT_EQ(drive.estimator.diagnostics().imu_yaw_resyncs, 1);
  EXPECT_GT(drive.estimator.diagnostics().imu_yaw_misses, 0);
  EXPECT_GT(drive.estimator.pose().x, before + 1.0)
    << "odometry remained latched to the pose before the expired IMU reference";
}

TEST(EstimatorDrive, ARejectedVisualSolvePreservesAContinuousTurnAcrossLongGap)
{
  EstimatorSettings settings = deployment_settings();
  settings.max_translation_per_frame_m = 1e-6;
  settings.imu_yaw_only_max_gap_sec = 20.0;
  Drive drive(settings);

  for (int i = 0; i < 180; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }
  ASSERT_GT(drive.estimator.diagnostics().map_aligned_frames, 0);
  drive.estimator.take_updates();

  // No camera frames arrive for sixteen seconds while the vehicle turns. The
  // translation solve that closes the gap is deliberately rejected below;
  // continuous IMU samples still make its yaw observable and safe to apply.
  constexpr double dt = 1.0 / 60.0;
  constexpr double yaw_rate = 0.10;
  for (int i = 0; i < 960; ++i) {
    drive.truth = drive.truth.compose(monoscale::motion_from_twist(0.0, 0.0, yaw_rate, dt));
    drive.stamp += dt;
    ImuSample imu;
    imu.stamp = drive.stamp;
    imu.orientation = yaw_quaternion(drive.truth.yaw);
    imu.angular_velocity = Eigen::Vector3d(0.0, 0.0, yaw_rate);
    imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, kGravity);
    drive.estimator.ingest_imu(imu);
  }

  for (int i = 0; i < 6; ++i) {
    drive.step(0.0, 0.0, 1.0 / 30.0);
  }

  EXPECT_GT(drive.estimator.diagnostics().yaw_only_updates, 0);
  EXPECT_NEAR(drive.estimator.pose().yaw, drive.truth.yaw, 0.05);
}

TEST(EstimatorDrive, CoastingReadsTheFilterVisionActuallyFeeds)
{
  // Drive normally until the map answers and the fusion filter has settled,
  // then thin the road out below the inlier floor so nothing solves. Every pair
  // after that is rejected and the coast is the only thing left to move the
  // pose, which makes it a direct test of which filter the coast asks.
  //
  // Four filters exist and exactly one of them is fed vision. The other three
  // are propagated on the accelerometer alone, and on a constant-speed drive
  // that integrates to zero -- so reading the wrong one does not coast, it
  // stops. The default model is Displacement, so the velocity filter beside it
  // is one of the three.
  Drive drive(deployment_settings());
  for (int i = 0; i < 180; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }
  ASSERT_GT(drive.estimator.diagnostics().map_aligned_frames, 0)
    << "the map never answered, so the filter never settled either";
  const double before = drive.estimator.pose().x;
  const int64_t failures_before = drive.estimator.diagnostics().motion_failures;

  std::vector<Eigen::Vector3d> sparse;
  for (size_t n = 0; n < drive.world.size(); n += 40) {
    sparse.push_back(drive.world[n]);
  }
  drive.world = sparse;
  for (int i = 0; i < 60; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }

  ASSERT_GT(drive.estimator.diagnostics().motion_failures, failures_before)
    << "the solve was supposed to be starved";
  EXPECT_GT(drive.estimator.diagnostics().coasted, 0);
  // Two seconds at 2 m/s, dead reckoned, so nothing here is exact. What is
  // being asserted is that it kept going at roughly road speed rather than
  // parking on the spot.
  const double carried = drive.estimator.pose().x - before;
  EXPECT_GT(carried, 2.0);
  EXPECT_LT(carried, 6.0);
}

TEST(EstimatorDrive, TheScaleComesOutOfTheGroundPlaneNotAFit)
{
  // The claim the whole stack rests on: distance is metric because the camera
  // height is. Halving the mounting height halves every recovered range, so a
  // drive solved with a wrong height comes out proportionally wrong -- which is
  // what makes the height the thing to get right rather than a free parameter.
  EstimatorSettings honest = deployment_settings();
  EstimatorSettings shortened = deployment_settings();
  for (auto & camera : shortened.cameras) {
    camera.range_scale = 2.0;
  }

  Drive truthful(honest);
  Drive shrunk(shortened);
  for (int i = 0; i < 120; ++i) {
    truthful.step(2.0, 0.0, 1.0 / 30.0);
    shrunk.step(2.0, 0.0, 1.0 / 30.0);
  }

  const double honest_x = truthful.estimator.pose().x;
  const double shrunk_x = shrunk.estimator.pose().x;
  ASSERT_GT(honest_x, 1.0);
  // Dividing every range by two halves the measured travel.
  EXPECT_NEAR(shrunk_x / honest_x, 0.5, 0.1);
}

TEST(EstimatorDrive, AFrameWithNoPeerDoesNotStallTheRun)
{
  // Online the two cameras do not always deliver together. A frame with no
  // partner inside the tolerance has to be dropped rather than held, or the
  // queues drift apart and no pair ever forms again.
  EstimatorSettings settings = deployment_settings();
  Drive drive(settings);

  for (int i = 0; i < 60; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }
  const int64_t before = drive.estimator.diagnostics().pairs_seen;

  // A lone front frame, far from anything the rear will produce.
  TrackFrame orphan;
  orphan.stamp = drive.stamp + 5.0;
  orphan.width = kTrackWidth;
  orphan.height = kTrackHeight;
  orphan.ids.resize(0);
  orphan.pixels.resize(0, 2);
  orphan.previous_pixels.resize(0, 2);
  drive.estimator.ingest_tracks(0, orphan);

  for (int i = 0; i < 60; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }

  EXPECT_GT(drive.estimator.diagnostics().pairs_seen, before + 30);
}

TEST(EstimatorDrive, OneImuSampleDrainsEveryReadyPair)
{
  EstimatorSettings settings = deployment_settings();
  settings.adaptive_solve_interval = false;
  settings.frame_decimation = 100;
  Estimator estimator(settings);

  ImuSample imu;
  imu.stamp = 1.0;
  imu.orientation = yaw_quaternion(0.0);
  imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, kGravity);
  estimator.ingest_imu(imu);

  for (double stamp : {1.1, 1.2}) {
    TrackFrame frame;
    frame.stamp = stamp;
    frame.width = kTrackWidth;
    frame.height = kTrackHeight;
    frame.ids.resize(0);
    frame.pixels.resize(0, 2);
    frame.previous_pixels.resize(0, 2);
    estimator.ingest_tracks(0, frame);
    estimator.ingest_tracks(1, frame);
  }
  EXPECT_EQ(estimator.diagnostics().pairs_seen, 0);

  imu.stamp = 1.3;
  estimator.ingest_imu(imu);

  EXPECT_EQ(estimator.diagnostics().pairs_seen, 2);
}

TEST(EstimatorDrive, ObstaclesAreSilentRatherThanFatalWithoutImages)
{
  // The Python reached for keyframe triangulation here and dereferenced a
  // frame that has no picture in it, which is a crash on the deployed path as
  // soon as mapping is enabled. Producing no obstacles is the honest answer.
  EstimatorSettings settings = deployment_settings();
  settings.mapping_min_period_sec = 0.0;
  settings.obstacle_slip_baseline_m = 0.0;
  Drive drive(settings);

  for (int i = 0; i < 90; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }
  const auto updates = drive.estimator.take_updates();

  size_t ground = 0;
  size_t obstacles = 0;
  for (const auto & update : updates) {
    for (const auto & point : update.points) {
      if (point.label == monoscale::kObstacleLabel) {
        ++obstacles;
      } else {
        ++ground;
      }
    }
  }
  EXPECT_GT(ground, 0u);
  EXPECT_EQ(obstacles, 0u);
  EXPECT_GT(drive.estimator.diagnostics().obstacles_unavailable, 0);
}

TEST(EstimatorDrive, OneCameraIsARigNotADegenerateCase)
{
  // With a single camera the disagreement between two of them -- the one
  // signal neither can produce alone -- is gone, and `single_camera_variance`
  // carries that weight instead. It runs, and it is measurably worse; what
  // must not happen is that it silently stops solving.
  EstimatorSettings settings = deployment_settings();
  settings.cameras = {front_camera()};
  Drive drive(settings);

  for (int i = 0; i < 120; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }

  ASSERT_GT(drive.estimator.diagnostics().map_aligned_frames, 0)
    << "a single camera never reached the anchor map";
  const Pose2 & pose = drive.estimator.pose();
  EXPECT_GT(pose.x, 0.5 * drive.travelled);
  EXPECT_LT(pose.x, drive.travelled + 0.5);
  EXPECT_NEAR(pose.y, 0.0, 0.5);
}

TEST(EstimatorDrive, AnObstacleIsReadOffHowItsProjectionSlides)
{
  // The default way of placing an obstacle on this path, and the only one it
  // has: a feature standing above the road projects to a ground point that
  // slides against the travel, and how fast it slides is its height. Nothing
  // is triangulated -- this is the residual the ground registration already
  // computes and throws away as an outlier.
  const double height = 0.5;
  EstimatorSettings settings = deployment_settings();
  settings.mapping_min_period_sec = 0.0;
  Drive drive(settings);
  drive.world = road_with_an_obstacle(height);

  for (int i = 0; i < 150; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }

  std::vector<float> heights;
  for (const auto & update : drive.estimator.take_updates()) {
    for (const auto & point : update.points) {
      if (point.label == monoscale::kObstacleLabel) {
        heights.push_back(point.z);
      }
    }
  }

  ASSERT_FALSE(heights.empty()) << "the obstacle never came out of the slip";
  // The method reads a height, not a point cloud: what has to hold is that the
  // heights land near the real one rather than at the road or at the gate.
  std::sort(heights.begin(), heights.end());
  const float middle = heights[heights.size() / 2];
  EXPECT_NEAR(middle, height, 0.2) << "median of " << heights.size() << " readings";
}

TEST(EstimatorDrive, FlatRoadProducesNoObstacles)
{
  // The other half of the same claim. A slip method that finds obstacles on an
  // empty road is worse than none: the grid ships at a threshold where one
  // reading marks a cell.
  EstimatorSettings settings = deployment_settings();
  settings.mapping_min_period_sec = 0.0;
  Drive drive(settings);

  for (int i = 0; i < 150; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }

  size_t obstacles = 0;
  size_t ground = 0;
  for (const auto & update : drive.estimator.take_updates()) {
    for (const auto & point : update.points) {
      (point.label == monoscale::kObstacleLabel ? obstacles : ground) += 1;
    }
  }
  EXPECT_GT(ground, 0u);
  EXPECT_EQ(obstacles, 0u);
}

TEST(EstimatorDrive, TheCovarianceItPublishesGrowsAndIsNotAFilterArtefact)
{
  // A dead-reckoning source has to say how sure it is, and an all-zero
  // covariance on the wire is read by plenty of stacks as "exact". This one is
  // built from what the solve measured -- the scatter of the inlying votes over
  // their count -- rather than from the fusion filter's own covariance, which
  // is dominated by a process noise chosen to make the filter defer to vision
  // and reads a NIS of 0.001 where an honest one gives 2.
  EstimatorSettings settings = deployment_settings();
  Drive drive(settings);
  for (int i = 0; i < 120; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }
  ASSERT_GT(drive.estimator.diagnostics().map_aligned_frames, 0);

  std::optional<monoscale::Update> last;
  double first_trace = -1.0;
  for (int i = 0; i < 120; ++i) {
    drive.step(2.0, 0.0, 1.0 / 30.0);
    for (const auto & update : drive.estimator.take_updates()) {
      if (!update.covariance_valid) {
        continue;
      }
      if (first_trace < 0.0) {
        first_trace = update.pose_covariance.trace();
      }
      last = update;
    }
  }
  ASSERT_TRUE(last.has_value()) << "no update carried a covariance";
  EXPECT_TRUE(last->pose_covariance.allFinite());
  EXPECT_GT(last->pose_covariance(0, 0), 0.0);
  EXPECT_GT(last->pose_covariance(1, 1), 0.0);
  EXPECT_GT(last->pose_covariance(2, 2), 0.0) << "the heading claims no uncertainty at all";
  // Dead reckoning has no absolute reference, so it only ever grows.
  EXPECT_GT(last->pose_covariance.trace(), first_trace);
  // And it has to be a believable size: a metre of one-sigma over six metres
  // travelled would be useless, and a micrometre would be a lie.
  const double sigma = std::sqrt(last->pose_covariance(0, 0) + last->pose_covariance(1, 1));
  EXPECT_GT(sigma, 1e-4);
  EXPECT_LT(sigma, 1.0);
  EXPECT_TRUE(last->twist_covariance.allFinite());
  EXPECT_GT(last->twist_covariance(0, 0), 0.0);
}

// Lone-camera solve (docs/C2_CRITERIA_KO.md 7.2-3, 7.4-1).
namespace
{
struct HoleResult
{
  std::vector<monoscale::Update> updates;
  Pose2 pose;
  double travelled = 0.0;
  double hole_start = 0.0;
  double hole_end = 0.0;
  monoscale::Diagnostics diagnostics;
};

// Both cameras, then only the rear for `hole_steps`, then both again, at a
// parking speed: the first lone hop spans lone_gap_sec, and the per-hop cap
// (max_translation_per_frame_m, 1 m) only lets it through below 1 m/s.
HoleResult drive_through_front_hole(bool lone, int hole_steps, double speed = 0.5)
{
  EstimatorSettings settings = deployment_settings();
  settings.lone_camera_solve_enabled = lone;
  Drive drive(settings);
  HoleResult out;
  const double dt = 1.0 / 30.0;
  for (int i = 0; i < 60; ++i) {
    drive.step(speed, 0.0, dt);
  }
  out.hole_start = drive.stamp;
  drive.active = {false, true};
  for (int i = 0; i < hole_steps; ++i) {
    drive.step(speed, 0.0, dt);
  }
  out.hole_end = drive.stamp;
  drive.active.clear();
  for (int i = 0; i < 90; ++i) {
    drive.step(speed, 0.0, dt);
  }
  out.updates = drive.estimator.take_updates();
  out.pose = drive.estimator.pose();
  out.travelled = drive.travelled;
  out.diagnostics = drive.estimator.diagnostics();
  return out;
}

size_t solved_inside(const HoleResult & r, double margin)
{
  size_t n = 0;
  for (const auto & u : r.updates) {
    if (u.stamp > r.hole_start + margin && u.stamp < r.hole_end && u.pose_valid && !u.solved_by.empty()) {
      ++n;
    }
  }
  return n;
}
}  // namespace

TEST(LoneCameraSolve, OffWaitsThroughAHoleAsBefore)
{
  const auto r = drive_through_front_hole(false, 90);
  EXPECT_EQ(solved_inside(r, 0.0), 0u);
  EXPECT_TRUE(r.diagnostics.lone_solves.empty());
}

TEST(LoneCameraSolve, OnTheRearAloneFillsAFrontHole)
{
  const auto r = drive_through_front_hole(true, 90);  // 3 s without the front
  EXPECT_GT(solved_inside(r, 1.0), 3u);
  ASSERT_TRUE(r.diagnostics.lone_solves.count("rear"));
  EXPECT_GT(r.diagnostics.lone_solves.at("rear"), 3);
  EXPECT_EQ(r.diagnostics.lone_solves.count("front"), 0u);
  // Lone updates are the rear's own: stamp and solver.
  for (const auto & u : r.updates) {
    if (u.stamp > r.hole_start + 1.0 && u.stamp < r.hole_end && !u.solved_by.empty()) {
      EXPECT_EQ(u.solved_by, "rear");
    }
  }
  // In this noiseless world one long two-camera hop across the hole (off) is
  // exact; chaining single-camera hops (on) costs a few centimetres. What lone
  // solving buys is output inside the hole, not a better total.
  const auto off = drive_through_front_hole(false, 90);
  EXPECT_LE(std::abs(r.pose.x - r.travelled), std::abs(off.pose.x - off.travelled) + 0.1);
  EXPECT_NEAR(r.pose.x, r.travelled, 0.3);
  EXPECT_NEAR(r.pose.y, 0.0, 0.3);
  std::printf("front hole 3 s: lone on x=%.3f, off x=%.3f, truth %.3f\n", r.pose.x, off.pose.x, r.travelled);
}

TEST(LoneCameraSolve, TheReturningCameraIsReseededAloneAndTheOtherKeepsSolving)
{
  const auto r = drive_through_front_hole(true, 90);
  EXPECT_GE(r.diagnostics.lone_reseeds, 1);
  // The first solved update after the hole is not a pose hold: the rear solved
  // across it while the front only re-seeded; then both solve again.
  bool saw_rear_after = false;
  bool saw_both_after = false;
  for (const auto & u : r.updates) {
    if (u.stamp <= r.hole_end || u.solved_by.empty()) {
      continue;
    }
    if (!saw_both_after && u.solved_by == "rear") {
      saw_rear_after = true;
    }
    if (u.solved_by == "front,rear") {
      saw_both_after = true;
    }
  }
  EXPECT_TRUE(saw_rear_after);
  EXPECT_TRUE(saw_both_after);
  EXPECT_EQ(r.diagnostics.imu_yaw_resyncs, 0);
}

TEST(LoneCameraSolve, AShortGapUnderLoneGapIsNotLone)
{
  // A front camera at a third of the rear's rate (a 0.1 s gap, well under
  // lone_gap_sec) stays a pairing matter: no lone solve.
  EstimatorSettings settings = deployment_settings();
  settings.lone_camera_solve_enabled = true;
  Drive drive(settings);
  for (int i = 0; i < 240; ++i) {
    drive.active = {i % 3 == 0, true};
    drive.step(2.0, 0.0, 1.0 / 30.0);
  }
  EXPECT_TRUE(drive.estimator.diagnostics().lone_solves.empty());
  EXPECT_GT(drive.estimator.diagnostics().frames_processed, 10);
}

TEST(LoneCameraSolve, AHoleLongerThanTheImuHistoryOnlyResyncsTheSilentCamera)
{
  // 35 s without the front (IMU history 30 s): the rear kept a fresh reference
  // by solving alone, so nothing expires for it and the front is re-seeded alone.
  const auto r = drive_through_front_hole(true, 35 * 30);
  EXPECT_EQ(r.diagnostics.imu_yaw_resyncs, 0);
  EXPECT_GE(r.diagnostics.lone_reseeds, 1);
  EXPECT_NEAR(r.pose.x, r.travelled, 1.0);
}

namespace
{
EstimatorSettings lone_settings()
{
  EstimatorSettings settings = deployment_settings();
  settings.lone_camera_solve_enabled = true;
  return settings;
}
}  // namespace

TEST(LoneCameraSolve, APeerThatArrivesLateIsNotTakenForAHole)
{
  // Frames reach the estimator seconds after capture (tracker lag; the front
  // 0.2 s behind the rear), with no hole at all: arrival is continuous, so
  // nothing goes alone however far behind the clock the frames are.
  Drive drive(lone_settings());
  drive.release = [](size_t camera, double captured, double now) {
      return now >= captured + (camera == 0 ? 3.0 : 2.8);
    };
  for (int i = 0; i < 300; ++i) {
    drive.step(0.5, 0.0, 1.0 / 30.0);
  }
  const auto & d = drive.estimator.diagnostics();
  EXPECT_TRUE(d.lone_solves.empty());
  EXPECT_TRUE(d.lone_frames.empty());
  EXPECT_TRUE(d.lone_late_peer.empty());
  EXPECT_GT(d.frames_processed, 10);
}

TEST(LoneCameraSolve, ASilentPeerLetsFramesGoAfterLoneSilence)
{
  // The front stops for good: it never moves past the rear's frames, so only
  // arrival silence can release them -- after lone_silence_sec, and only rear
  // frames more than lone_gap_sec past the front's last one (the ones before
  // wait however long the silence lasts).
  Drive drive(lone_settings());
  for (int i = 0; i < 60; ++i) {
    drive.step(0.5, 0.0, 1.0 / 30.0);
  }
  const double last_front = drive.stamp;
  drive.active = {false, true};
  for (int i = 0; i < 45; ++i) {  // 1.5 s: silent, but not for lone_silence_sec
    drive.step(0.5, 0.0, 1.0 / 30.0);
  }
  EXPECT_TRUE(drive.estimator.diagnostics().lone_frames.empty());
  for (int i = 0; i < 75; ++i) {  // to 4 s of silence
    drive.step(0.5, 0.0, 1.0 / 30.0);
  }
  const auto & d = drive.estimator.diagnostics();
  ASSERT_TRUE(d.lone_frames.count("rear"));
  EXPECT_GT(d.lone_frames.at("rear"), 0);
  EXPECT_GT(d.first_lone_stamp, last_front + 1.0);
  EXPECT_EQ(d.lone_by_reason.count("sparse"), 0u);
}

TEST(LoneCameraSolve, AFrontEveryTwoSecondsIsSparseAndTheRearGoesAlone)
{
  // Front frames 2 s apart (each a peer for the rear within lone_gap_sec, so
  // the hole test never fires): rear frames between them have no pair inside
  // sync tolerance and are solved alone once the front frame after them is in.
  // Off, the same drive yields nothing (two pairs, no motion).
  Drive drive(lone_settings());
  for (int i = 0; i < 360; ++i) {
    drive.active = {i % 60 == 0, true};
    drive.step(0.5, 0.0, 1.0 / 30.0);
  }
  const auto & d = drive.estimator.diagnostics();
  ASSERT_TRUE(d.lone_by_reason.count("sparse"));
  EXPECT_GT(d.lone_by_reason.at("sparse"), 100);
  EXPECT_TRUE(d.lone_late_peer.empty());
  // Travel between the first and last valid update against the truth over the
  // same stamps (the rear frames after the last front frame are still waiting
  // for the next one, as they should).
  const auto updates = drive.estimator.take_updates();
  const monoscale::Update * first = nullptr;
  const monoscale::Update * last = nullptr;
  for (const auto & u : updates) {
    if (u.pose_valid) {
      if (!first) {
        first = &u;
      }
      last = &u;
    }
  }
  ASSERT_TRUE(first && last);
  EXPECT_GT(last->stamp - first->stamp, 8.0);
  EXPECT_NEAR(last->pose.x - first->pose.x, 0.5 * (last->stamp - first->stamp), 0.15);
}

TEST(LoneCameraSolve, AFrontEveryOnePointTwoSecondsIsNotSparse)
{
  // Inside lone_sparse_sec (the recordings' normal widest gap is 1.33 s).
  Drive drive(lone_settings());
  for (int i = 0; i < 360; ++i) {
    drive.active = {i % 36 == 0, true};
    drive.step(0.5, 0.0, 1.0 / 30.0);
  }
  EXPECT_TRUE(drive.estimator.diagnostics().lone_frames.empty());
}

TEST(LoneCameraSolve, APeerArrivingInBurstsAfterSilenceIsCountedAsAMisfire)
{
  // The residual risk of the arrival test: a front that goes quiet past
  // lone_silence_sec and then delivers its backlog at once. Rear frames released
  // during the quiet are wrong, and lone_late_peer says so.
  Drive drive(lone_settings());
  drive.release = [](size_t camera, double, double now) {
      // The front delivers only in a 0.2 s window every 3 s.
      return camera == 1 || std::fmod(now, 3.0) < 0.2;
    };
  for (int i = 0; i < 450; ++i) {
    drive.step(0.5, 0.0, 1.0 / 30.0);
  }
  const auto & d = drive.estimator.diagnostics();
  ASSERT_TRUE(d.lone_late_peer.count("rear"));
  EXPECT_GT(d.lone_late_peer.at("rear"), 0);
}


TEST(LoneCameraSolve, RearFramesAboutToBeAbsorbedIntoTheNextPairGoAloneFirst)
{
  // Front frames 2 s apart that reach the estimator 0.5 s late: by the time the
  // next front frame arrives, the rear frame paired with it is already queued,
  // so the rear frames before it would be absorbed into that pair without ever
  // reaching the drop path. They go alone instead (observer_eval_C2_5a_fardrop).
  Drive drive(lone_settings());
  drive.release = [](size_t camera, double captured, double now) {
      return camera == 1 || now >= captured + 0.5;
    };
  // Rear at 5 Hz as on the vehicle (a 30 Hz rear would fill the retention).
  for (int i = 0; i < 360; ++i) {
    drive.active = {i % 60 == 0, i % 6 == 0};
    drive.step(0.5, 0.0, 1.0 / 30.0);
  }
  const auto & d = drive.estimator.diagnostics();
  const int64_t absorbed_alone =
    (d.lone_by_reason.count("sparse@absorb") ? d.lone_by_reason.at("sparse@absorb") : 0) +
    (d.lone_by_reason.count("gap@absorb") ? d.lone_by_reason.at("gap@absorb") : 0);
  EXPECT_GT(absorbed_alone, 20);
  EXPECT_TRUE(d.lone_late_peer.empty());
}

TEST(LoneCameraSolve, NormallySpacedLatePeersAreAbsorbedAsBefore)
{
  // Front every 0.4 s, 0.5 s late: absorption is ordinary pairing, nothing alone.
  Drive drive(lone_settings());
  drive.release = [](size_t camera, double captured, double now) {
      return camera == 1 || now >= captured + 0.5;
    };
  for (int i = 0; i < 300; ++i) {
    drive.active = {i % 12 == 0, i % 6 == 0};
    drive.step(0.5, 0.0, 1.0 / 30.0);
  }
  const auto & d = drive.estimator.diagnostics();
  EXPECT_EQ(d.lone_by_reason.count("sparse@absorb"), 0u);
  EXPECT_EQ(d.lone_by_reason.count("gap@absorb"), 0u);
  EXPECT_TRUE(d.lone_frames.empty());
}

TEST(FusionModelName, OnlyImplementedModelsAreAccepted)
{
  EXPECT_EQ(monoscale::fusion_model_from_name("velocity"), monoscale::FusionModel::Velocity);
  EXPECT_EQ(monoscale::fusion_model_from_name("displacement"), monoscale::FusionModel::Displacement);
  EXPECT_THROW(monoscale::fusion_model_from_name("msckf"), std::invalid_argument);
  EXPECT_THROW(monoscale::fusion_model_from_name("msckf6"), std::invalid_argument);
  EXPECT_THROW(monoscale::fusion_model_from_name("velocty"), std::invalid_argument);
}

