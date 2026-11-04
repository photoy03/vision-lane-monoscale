#pragma once

// Shared image sharpness measure for the lane and occupancy quality gates (D-17).
// Header-only; the including package links OpenCV (this package does not).
//
// The two nodes still call it with their own values (lane: 640 px sample, ksize 3;
// occupancy: its processing image as is, ksize 1), so the numbers are the ones each
// node computed before. The two scales differ: the same threshold does not mean the
// same thing in both nodes until they are called with the same values.
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace image_quality {

// Variance of the Laplacian of `image` (gray, BGR or BGRA). The image is first shrunk
// to `sample_width` px (INTER_AREA) when it is wider; sample_width <= 0 measures it as
// is. ksize 1 is OpenCV's fixed 3x3 kernel, larger values the Sobel-based one. When
// `gray_out` is given it receives the gray sample that was measured.
inline double sharpness(const cv::Mat & image, int sample_width, int ksize, cv::Mat * gray_out = nullptr)
{
  cv::Mat sample;
  if (sample_width > 0 && image.cols > sample_width) {
    const double scale = sample_width / static_cast<double>(image.cols);
    cv::resize(image, sample, {}, scale, scale, cv::INTER_AREA);
  } else {
    sample = image;
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
  cv::Laplacian(gray, laplacian, CV_32F, ksize);
  cv::Scalar mean;
  cv::Scalar deviation;
  cv::meanStdDev(laplacian, mean, deviation);
  if (gray_out != nullptr) {
    *gray_out = gray;
  }
  return deviation[0] * deviation[0];
}

}  // namespace image_quality
