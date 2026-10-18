// D-17 stage 1: the shared sharpness measure gives each node the number it computed
// before. The references below are the two inline measures as they stood.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "vision_lane_interfaces/image_quality.hpp"

namespace {

// global_mapping_node.cpp measure_image_quality() before D-17.
double lane_reference(const cv::Mat & input, int sample_width, cv::Mat * gray_out)
{
  cv::Mat sample;
  if (input.cols > sample_width) {
    const double scale = sample_width / static_cast<double>(input.cols);
    cv::resize(input, sample, {}, scale, scale, cv::INTER_AREA);
  } else {
    sample = input;
  }
  cv::Mat gray;
  if (sample.channels() == 3) {
    cv::cvtColor(sample, gray, cv::COLOR_BGR2GRAY);
  } else if (sample.channels() == 4) {
    cv::cvtColor(sample, gray, cv::COLOR_BGRA2GRAY);
  } else {
    gray = sample;
  }
  cv::Mat laplacian;
  cv::Laplacian(gray, laplacian, CV_32F, 3);
  cv::Scalar mean, stddev;
  cv::meanStdDev(laplacian, mean, stddev);
  gray.copyTo(*gray_out);
  return stddev[0] * stddev[0];
}

// sweep_node.cpp image callback before D-17 (on the already resized gray image).
double occupancy_reference(const cv::Mat & resized)
{
  cv::Mat laplacian;
  cv::Laplacian(resized, laplacian, CV_32F);
  cv::Scalar mean;
  cv::Scalar deviation;
  cv::meanStdDev(laplacian, mean, deviation);
  return deviation[0] * deviation[0];
}

cv::Mat noise(int rows, int cols, int type, int seed)
{
  cv::Mat image(rows, cols, type);
  cv::RNG rng(seed);
  rng.fill(image, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(image, image, {5, 5}, 1.2);
  return image;
}

bool same_bits(double a, double b)
{
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

bool same_image(const cv::Mat & a, const cv::Mat & b)
{
  return a.size() == b.size() && a.type() == b.type() && cv::norm(a, b, cv::NORM_INF) == 0.0;
}

}  // namespace

TEST(ImageQuality, LaneCallMatchesTheOldLaneMeasure)
{
  int seed = 1;
  for (const int type : {CV_8UC1, CV_8UC3, CV_8UC4}) {
    for (const cv::Size size : {cv::Size(2560, 1440), cv::Size(1280, 720), cv::Size(640, 360),
        cv::Size(320, 240)})
    {
      const cv::Mat input = noise(size.height, size.width, type, seed++);
      cv::Mat old_gray;
      cv::Mat new_gray;
      const double old_value = lane_reference(input, 640, &old_gray);
      const double new_value = image_quality::sharpness(input, 640, 3, &new_gray);
      EXPECT_TRUE(same_bits(old_value, new_value)) << size << " type " << type;
      EXPECT_TRUE(same_image(old_gray, new_gray)) << size << " type " << type;
    }
  }
}

TEST(ImageQuality, OccupancyCallMatchesTheOldOccupancyMeasure)
{
  int seed = 100;
  for (const int processing_width : {1280, 640}) {
    for (const cv::Size size : {cv::Size(2560, 1440), cv::Size(1280, 720), cv::Size(640, 480)}) {
      const cv::Mat gray = noise(size.height, size.width, CV_8UC1, seed++);
      const double scale = processing_width / static_cast<double>(gray.cols);
      cv::Mat resized;
      cv::resize(gray, resized, cv::Size(), scale, scale, cv::INTER_AREA);
      EXPECT_TRUE(same_bits(occupancy_reference(resized), image_quality::sharpness(resized, 0, 1)))
        << size << " width " << processing_width;
    }
  }
}

TEST(ImageQuality, TheTwoScalesDiffer)
{
  // Why one threshold cannot serve both nodes until they share the call values.
  cv::Mat resized;
  cv::resize(noise(1440, 2560, CV_8UC1, 7), resized, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);
  const double occupancy = image_quality::sharpness(resized, 0, 1);
  const double lane = image_quality::sharpness(resized, 640, 3);
  EXPECT_GT(std::abs(lane - occupancy), 0.1 * std::max(lane, occupancy));
}
