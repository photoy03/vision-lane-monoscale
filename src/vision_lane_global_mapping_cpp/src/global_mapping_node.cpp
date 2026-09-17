#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <thread>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#ifndef VISION_LANE_HAS_OPENCV_CUDA
#define VISION_LANE_HAS_OPENCV_CUDA 0
#endif
#if VISION_LANE_HAS_OPENCV_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_lane_interfaces/msg/marking_segment.hpp>
#include <vision_lane_interfaces/msg/marking_segment_array.hpp>
#include "pose_timeline.hpp"

namespace {
using Clock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;

double ms_since(const Clock::time_point &t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
double rad(double deg) { return deg * kPi / 180.0; }
double deg(double r) { return r * 180.0 / kPi; }
double wrap_pi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}
double angle180(double a) {
  a = std::fmod(a, 180.0);
  return a < 0.0 ? a + 180.0 : a;
}
double angle_diff180(double a, double b) {
  const double d = std::abs(angle180(a) - angle180(b));
  return std::min(d, 180.0 - d);
}
double orthogonal_error(double a, double base) {
  return std::min(angle_diff180(a, base), angle_diff180(a, base + 90.0));
}
double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }

struct Pose2 {
  double x{0.0}, y{0.0}, yaw{0.0};
};

cv::Point2f metric_to_base(const cv::Point2f &p) {
  // BEV: x=right, y=forward. base_link: x=forward, y=left.
  return {p.y, -p.x};
}
cv::Point2f base_to_metric(const cv::Point2f &p) { return {-p.y, p.x}; }
cv::Point2f base_to_global(const cv::Point2f &p, const Pose2 &pose) {
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  return {static_cast<float>(pose.x + c * p.x - s * p.y),
          static_cast<float>(pose.y + s * p.x + c * p.y)};
}
cv::Point2f global_to_base(const cv::Point2f &p, const Pose2 &pose) {
  const double dx = p.x - pose.x, dy = p.y - pose.y;
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  return {static_cast<float>(c * dx + s * dy),
          static_cast<float>(-s * dx + c * dy)};
}

struct Params {
  double camera_height{0.8}, camera_forward{0.0}, camera_left{0.0};
  double pitch_deg{30.0}, roll_deg{0.0}, yaw_deg{0.0};
  double x_min{-4.0}, x_max{4.0}, y_min{0.0}, y_max{5.0}, mpp{0.02};
  int white_s_max{60}, white_v_min{140};
  int yellow_h_min{15}, yellow_h_max{40}, yellow_s_min{70}, yellow_v_min{70};
  int glare_v_min{248}, glare_s_max{95}, glare_dilate{9};
  double glare_min_area_ratio{0.006};
  double illumination_sigma_m{0.60}, illumination_scale{0.35}, normalization_scale{128.0};
  double tophat_size_m{0.30};
  int threshold_bands{6}, threshold_update_interval{3}, threshold_sample_stride{4};
  double mad_k{5.5}, threshold_margin{3.0}, threshold_min{8.0}, threshold_max{80.0};
  double threshold_alpha{0.20}, strong_response_margin{36.0}, glare_recovery_margin{20.0};
  int hough_threshold{28}, max_raw{120};
  double hough_min_len{0.65}, hough_max_gap{0.18};
  double candidate_min_len{0.70}, min_binary{0.72}, min_color{0.08};
  double candidate_min_score{0.46}, max_glare{0.85};
  double line_min_width{0.05}, line_max_width{0.22}, line_target_width{0.12};
  double dominant_tol{12.0}, dominant_min_conf{0.35};
  double merge_distance{0.20}, merge_gap{0.35};
  double final_min_span{0.85}, final_min_score{0.52};
  double parallel_min{1.50}, parallel_max{3.50}, orthogonal_near{0.45};
  double inference_parallel_w{0.18}, inference_orthogonal_w{0.14};
  double inference_appearance_w{0.68}, inference_min_score{0.50};
  double strong_single_score{0.72};
  int max_each{8}, vectorize_every_n{2};
  int history{5}, confirm_hits{3}, max_missed{3};
  double temporal_angle{7.0}, temporal_offset{0.32}, temporal_gap{0.60}, ema{0.35};
  double map_x_min{-2.0}, map_x_max{20.0}, map_y_min{-10.0}, map_y_max{10.0};
  double map_res{0.03}, map_inc{1.0}, map_max{20.0}, map_confirm{3.0};
  double map_min_translation{0.05}, map_min_yaw_deg{1.0};
};

struct Prep {
  cv::Mat binary, color, glare, response, threshold_map;
};

struct Candidate {
  cv::Point2f p0_px, p1_px, p0_m, p1_m;
  double angle{0}, length{0}, width{0}, binary{0}, color{0}, response{0}, glare{0}, score{0};
};

struct Observation {
  cv::Point2f p0_m, p1_m;
  std::string family;
  double angle{0}, span{0}, width{0}, confidence{0};
  uint32_t track_id{0}, hits_total{0}, hits_window{0};
  bool confirmed{false}, reprojected{false};
};

struct Primitive {
  uint32_t id{0};
  cv::Point2f g0, g1;
  std::string family;
  double width{0}, confidence{0};
  std::deque<int> hits;
  int hits_total{1}, missed{0};
  bool confirmed{false};
};

struct Profile {
  uint64_t frame{0};
  double bev{0}, preprocess{0}, hough{0}, temporal{0}, mapping{0}, total{0};
  double input_age{0}, pose_age{0};
  int raw{0}, observations{0}, active{0}, confirmed{0};
  bool pose_valid{false}, motion_detected{false}, map_accumulated{false};
  double sharpness{0.0}, frame_delta{0.0};
  bool quality_rejected{false};
  int map_updates{0};
};

struct PoseLookup {
  Pose2 pose;
  bool valid{false};
  bool interpolated{false};
  double age_sec{std::numeric_limits<double>::infinity()};
};

struct MapUpdateResult {
  bool initial_seed{false};
  bool motion_detected{false};
  bool accumulated{false};
  double translation_m{0.0};
  double yaw_deg{0.0};
};

double percentile(std::vector<float> values, double q) {
  if (values.empty()) return 0.0;
  const size_t k = std::min(values.size() - 1,
      static_cast<size_t>(std::llround((values.size() - 1) * q)));
  std::nth_element(values.begin(), values.begin() + k, values.end());
  return values[k];
}
double median(std::vector<float> values) { return percentile(std::move(values), 0.5); }

cv::Matx33d rot_x(double d) {
  const double c = std::cos(rad(d)), s = std::sin(rad(d));
  return {1,0,0, 0,c,-s, 0,s,c};
}
cv::Matx33d rot_z(double d) {
  const double c = std::cos(rad(d)), s = std::sin(rad(d));
  return {c,-s,0, s,c,0, 0,0,1};
}

class Pipeline {
 public:
  explicit Pipeline(Params p) : p_(std::move(p)) {}

  bool initialize_maps(const std::string &path, int source_w, int source_h) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    cv::Mat K, D;
    int calib_w = 0, calib_h = 0;
    fs["K"] >> K; fs["D"] >> D; fs["image_width"] >> calib_w; fs["image_height"] >> calib_h;
    if (K.empty() || D.empty() || calib_w <= 0 || calib_h <= 0) return false;
    K.convertTo(K, CV_64F); D.convertTo(D, CV_64F);
    const double sx = source_w / static_cast<double>(calib_w);
    const double sy = source_h / static_cast<double>(calib_h);
    K.at<double>(0,0) *= sx; K.at<double>(0,2) *= sx;
    K.at<double>(1,1) *= sy; K.at<double>(1,2) *= sy;

    const int bw = static_cast<int>(std::llround((p_.x_max - p_.x_min) / p_.mpp));
    const int bh = static_cast<int>(std::llround((p_.y_max - p_.y_min) / p_.mpp));
    std::vector<cv::Point3d> ground; ground.reserve(static_cast<size_t>(bw * bh));
    for (int v = 0; v < bh; ++v) {
      const double y = p_.y_max - v * p_.mpp;
      for (int u = 0; u < bw; ++u) ground.emplace_back(p_.x_min + u * p_.mpp, y, 0.0);
    }
    const cv::Vec3d camera_pos(-p_.camera_left, p_.camera_forward, p_.camera_height);
    const cv::Matx33d base(1,0,0, 0,0,-1, 0,1,0);
    const cv::Matx33d R = rot_z(p_.roll_deg) * rot_x(p_.pitch_deg) * base * rot_z(-p_.yaw_deg);
    std::vector<cv::Point3d> camera; camera.reserve(ground.size());
    std::vector<uint8_t> depth_ok; depth_ok.reserve(ground.size());
    for (const auto &g : ground) {
      const cv::Vec3d q = R * (cv::Vec3d(g.x,g.y,g.z) - camera_pos);
      camera.emplace_back(q[0],q[1],q[2]); depth_ok.push_back(q[2] > 1e-6);
    }
    std::vector<cv::Point2d> image;
    cv::fisheye::projectPoints(camera, image, cv::Vec3d(0,0,0), cv::Vec3d(0,0,0), K, D);
    map_x_.create(bh, bw, CV_32F); map_y_.create(bh, bw, CV_32F);
    valid_ = cv::Mat::zeros(bh, bw, CV_8U);
    for (int i = 0; i < bw * bh; ++i) {
      const int v = i / bw, u = i % bw;
      map_x_.at<float>(v,u) = static_cast<float>(image[i].x);
      map_y_.at<float>(v,u) = static_cast<float>(image[i].y);
      if (depth_ok[i] && image[i].x >= 0 && image[i].x < source_w-1 &&
          image[i].y >= 0 && image[i].y < source_h-1) valid_.at<uint8_t>(v,u)=255;
    }
    source_size_ = {source_w, source_h};
    thresholds_.assign(std::max(1,p_.threshold_bands), p_.threshold_min);
    return true;
  }

  cv::Mat remap_bev(const cv::Mat &frame) const {
    cv::Mat bev; cv::remap(frame, bev, map_x_, map_y_, cv::INTER_LINEAR,
                           cv::BORDER_CONSTANT, cv::Scalar());
    return bev;
  }

  const cv::Mat &valid_mask() const { return valid_; }
  const cv::Mat &map_x() const { return map_x_; }
  const cv::Mat &map_y() const { return map_y_; }

  Prep preprocess(const cv::Mat &bev) {
    cv::Mat hsv; cv::cvtColor(bev, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> ch; cv::split(hsv, ch);
    cv::Mat white, yellow;
    cv::inRange(hsv, cv::Scalar(0,0,p_.white_v_min), cv::Scalar(179,p_.white_s_max,255), white);
    cv::inRange(hsv, cv::Scalar(p_.yellow_h_min,p_.yellow_s_min,p_.yellow_v_min),
                cv::Scalar(p_.yellow_h_max,255,255), yellow);
    cv::Mat color;
    cv::bitwise_or(white, yellow, color);
    cv::Mat glare_seed;
    cv::inRange(hsv, cv::Scalar(0,0,p_.glare_v_min), cv::Scalar(179,p_.glare_s_max,255), glare_seed);

    cv::Mat response;
    make_response_cpu(ch[2], response);
    return finalize_preprocess(color, glare_seed, response);
  }

  Prep finalize_preprocess(const cv::Mat &color, const cv::Mat &glare_seed_input,
                           const cv::Mat &response_input) {
    Prep out;
    cv::bitwise_and(color, valid_, out.color);
    cv::Mat white_support;
    cv::dilate(out.color, white_support, cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
    cv::Mat glare_seed;
    cv::bitwise_and(glare_seed_input, valid_, glare_seed);
    cv::Mat labels, stats, centers;
    const int count=cv::connectedComponentsWithStats(glare_seed,labels,stats,centers,8);
    out.glare=cv::Mat::zeros(glare_seed.size(),CV_8U);
    const int min_area=std::max(1,static_cast<int>(std::llround(valid_.total()*p_.glare_min_area_ratio)));
    for(int label=1;label<count;++label)if(stats.at<int>(label,cv::CC_STAT_AREA)>=min_area)
      out.glare.setTo(255,labels==label);
    int gk=std::max(1,p_.glare_dilate)|1;
    if(gk>1)cv::dilate(out.glare,out.glare,cv::getStructuringElement(cv::MORPH_ELLIPSE,{gk,gk}));
    cv::bitwise_and(out.glare, valid_, out.glare);
    response_input.copyTo(out.response);
    cv::bitwise_and(out.response, valid_, out.response);

    out.threshold_map = cv::Mat(out.response.size(), CV_32F);
    cv::Mat raw = cv::Mat::zeros(out.response.size(), CV_8U);
    const int bands = std::max(1, std::min(p_.threshold_bands, out.response.rows));
    if (static_cast<int>(thresholds_.size()) != bands) thresholds_.assign(bands,p_.threshold_min);
    const bool recompute=threshold_frame_==0 || threshold_frame_%std::max(1,p_.threshold_update_interval)==0;
    ++threshold_frame_;
    for (int b=0;b<bands;++b) {
      const int y0=b*out.response.rows/bands, y1=(b+1)*out.response.rows/bands;
      if(recompute){std::vector<float> values;
        const int stride=std::max(1,p_.threshold_sample_stride);
        for (int y=y0;y<y1;y+=stride) for (int x=0;x<out.response.cols;x+=stride)
          if (valid_.at<uint8_t>(y,x) && !out.glare.at<uint8_t>(y,x)) values.push_back(out.response.at<uint8_t>(y,x));
        double now=p_.threshold_min;
        if(values.size()>=32){std::vector<float> bg=values;const double trim=percentile(values,0.90);
          bg.erase(std::remove_if(bg.begin(),bg.end(),[&](float v){return v>trim;}),bg.end());if(bg.empty())bg=values;
          const double med=median(bg);std::vector<float> dev;dev.reserve(bg.size());for(float v:bg)dev.push_back(std::abs(v-static_cast<float>(med)));
          const double mad=median(dev);const double tmad=med+p_.mad_k*1.4826*mad;const double thigh=percentile(bg,0.99)+p_.threshold_margin;
          now=std::clamp(std::max({tmad,thigh,p_.threshold_min}),p_.threshold_min,p_.threshold_max);}
        thresholds_[b]=threshold_frame_==1?now:(1.0-p_.threshold_alpha)*thresholds_[b]+p_.threshold_alpha*now;}
      out.threshold_map.rowRange(y0,y1).setTo(thresholds_[b]);
      cv::compare(out.response.rowRange(y0,y1), cv::Scalar(thresholds_[b]), raw.rowRange(y0,y1), cv::CMP_GT);
    }
    cv::Mat strong, rf; out.response.convertTo(rf,CV_32F);
    cv::compare(rf, out.threshold_map + p_.strong_response_margin, strong, cv::CMP_GE);
    cv::Mat support; cv::bitwise_or(white_support,strong,support);
    cv::bitwise_and(raw,support,out.binary); cv::bitwise_and(out.binary,valid_,out.binary);
    cv::Mat recovery, outside, not_glare;
    cv::compare(rf, out.threshold_map + p_.glare_recovery_margin, recovery, cv::CMP_GE);
    cv::bitwise_and(recovery,out.glare,recovery);cv::bitwise_not(out.glare,not_glare);
    cv::bitwise_and(out.binary,not_glare,outside);cv::bitwise_or(outside,recovery,out.binary);
    cv::morphologyEx(out.binary,out.binary,cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
    cv::morphologyEx(out.binary,out.binary,cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
    return out;
  }

  std::vector<Observation> vectorize(const Prep &prep, int *raw_count, double *dom, double *conf) const {
    cv::Mat edges; cv::Canny(prep.binary,edges,50,150,3,false);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges,lines,1.0,kPi/360.0,p_.hough_threshold,
                    std::max(10.0,p_.hough_min_len/p_.mpp),
                    std::max(1.0,p_.hough_max_gap/p_.mpp));
    std::sort(lines.begin(),lines.end(),[](const auto&a,const auto&b){
      return cv::norm(cv::Point(a[0],a[1])-cv::Point(a[2],a[3])) >
             cv::norm(cv::Point(b[0],b[1])-cv::Point(b[2],b[3]));});
    if (static_cast<int>(lines.size())>p_.max_raw) lines.resize(p_.max_raw);
    cv::Mat distance; cv::distanceTransform(prep.binary,distance,cv::DIST_L2,3);
    std::vector<Candidate> cand;
    for(const auto &l:lines) {
      Candidate c; c.p0_px={static_cast<float>(l[0]),static_cast<float>(l[1])};
      c.p1_px={static_cast<float>(l[2]),static_cast<float>(l[3])};
      c.p0_m=pixel_to_metric(c.p0_px); c.p1_m=pixel_to_metric(c.p1_px);
      c.length=cv::norm(c.p1_m-c.p0_m); if(c.length<p_.candidate_min_len) continue;
      c.angle=angle180(deg(std::atan2(c.p1_m.y-c.p0_m.y,c.p1_m.x-c.p0_m.x)));
      quality_samples(c,prep,distance);
      if(c.binary<p_.min_binary || (c.color<p_.min_color && c.response<0.70) || c.glare>p_.max_glare) continue;
      const double half=std::max({p_.line_target_width-p_.line_min_width,
                                  p_.line_max_width-p_.line_target_width,0.02});
      const double width_score=clamp01(1.0-std::abs(c.width-p_.line_target_width)/half);
      const double length_score=clamp01((c.length-p_.candidate_min_len)/2.0);
      c.score=clamp01((0.28*c.binary+0.20*c.color+0.20*width_score+0.14*c.response+
                       0.18*length_score)*(1.0-0.45*clamp01(c.glare)));
      if(c.width<p_.line_min_width || c.width>p_.line_max_width || c.score<p_.candidate_min_score) continue;
      cand.push_back(c);
    }
    *raw_count=static_cast<int>(cand.size());
    if(cand.empty()){*dom=0;*conf=0;return{};}
    std::tie(*dom,*conf)=dominant(cand);
    if(*conf<p_.dominant_min_conf) return{};
    auto a=merge(cand,"A",*dom); auto b=merge(cand,"B",angle180(*dom+90.0));
    geometric_filter(a,b);
    a.insert(a.end(),b.begin(),b.end()); return a;
  }

 private:
  void make_response_cpu(const cv::Mat &value, cv::Mat &response) const {
    cv::Mat small, illum;
    cv::Mat source_f; value.convertTo(source_f,CV_32F); source_f+=1.0f;
    cv::resize(source_f, small, {}, p_.illumination_scale, p_.illumination_scale, cv::INTER_AREA);
    const double sigma=std::max(1.0,p_.illumination_sigma_m/p_.mpp*p_.illumination_scale);
    cv::GaussianBlur(small, small, {0,0}, sigma, sigma, cv::BORDER_REFLECT);
    cv::resize(small, illum, value.size(), 0,0, cv::INTER_LINEAR);
    cv::Mat normf, normalized;
    cv::divide(source_f, illum + 1e-6, normf, p_.normalization_scale);
    normf.convertTo(normalized,CV_8U); cv::bitwise_and(normalized, valid_, normalized);
    cv::GaussianBlur(normalized, normalized, {3,3}, 0);
    const int tk=std::max(3,static_cast<int>(std::llround(p_.tophat_size_m/p_.mpp))|1);
    cv::morphologyEx(normalized,response,cv::MORPH_TOPHAT,
                     cv::getStructuringElement(cv::MORPH_RECT,{tk,tk}));
  }

  cv::Point2f pixel_to_metric(const cv::Point2f&p) const {
    return {static_cast<float>(p_.x_min+(p.x+0.5)*p_.mpp),
            static_cast<float>(p_.y_max-(p.y+0.5)*p_.mpp)};
  }
  void quality_samples(Candidate &c,const Prep&prep,const cv::Mat&distance) const {
    cv::Point2f d=c.p1_px-c.p0_px; const double len=cv::norm(d); d*=1.0f/static_cast<float>(len);
    const cv::Point2f n(-d.y,d.x); const int samples=std::clamp(static_cast<int>(std::llround(len/8.0)),12,64);
    const int radius=std::clamp(static_cast<int>(std::ceil((p_.line_max_width/p_.mpp)*0.75)),3,24);
    int bh=0,ch=0,gh=0; double response_sum=0; std::vector<float> widths;
    for(int i=0;i<samples;++i){
      const float t=samples==1?0.0f:i/static_cast<float>(samples-1);
      const cv::Point2f center=c.p0_px+t*(c.p1_px-c.p0_px);
      bool bok=false,cok=false,gok=false; float maxdist=0,maxexcess=0;
      for(int o=-radius;o<=radius;++o){
        const cv::Point2f q=center+static_cast<float>(o)*n;
        const int x=static_cast<int>(std::llround(q.x)),y=static_cast<int>(std::llround(q.y));
        if(x<0||x>=prep.binary.cols||y<0||y>=prep.binary.rows) continue;
        bok|=prep.binary.at<uint8_t>(y,x)>0; cok|=prep.color.at<uint8_t>(y,x)>0;
        gok|=prep.glare.at<uint8_t>(y,x)>0; maxdist=std::max(maxdist,distance.at<float>(y,x));
        maxexcess=std::max(maxexcess,std::max(0.0f,prep.response.at<uint8_t>(y,x)-prep.threshold_map.at<float>(y,x)));
      }
      bh+=bok;ch+=cok;gh+=gok;if(maxdist>0.25f)widths.push_back(maxdist);
      response_sum+=maxexcess;
    }
    c.binary=bh/static_cast<double>(samples);c.color=ch/static_cast<double>(samples);
    c.glare=gh/static_cast<double>(samples);c.width=widths.empty()?0.0:2.0*median(widths)*p_.mpp;
    c.response=clamp01((response_sum/samples)/std::max(p_.strong_response_margin,1.0));
  }
  std::pair<double,double> dominant(const std::vector<Candidate>&c) const {
    std::vector<double>w; double total=0; for(const auto&x:c){w.push_back(std::min(x.length,3.0)*std::pow(std::max(x.score,0.05),2));total+=w.back();}
    double best=c.front().angle,support=-1; std::vector<bool> best_in;
    for(const auto&p:c){double s=0;std::vector<bool>in;for(size_t i=0;i<c.size();++i){const bool ok=orthogonal_error(c[i].angle,p.angle)<=p_.dominant_tol;in.push_back(ok);if(ok)s+=w[i];}if(s>support){support=s;best=p.angle;best_in=in;}}
    double zr=0,zi=0;for(size_t i=0;i<c.size();++i)if(best_in[i]){zr+=w[i]*std::cos(4*rad(c[i].angle));zi+=w[i]*std::sin(4*rad(c[i].angle));}
    if(std::abs(zr)+std::abs(zi)>1e-9)best=std::fmod(deg(0.25*std::atan2(zi,zr))+90.0,90.0);
    return {best,support/std::max(total,1e-9)};
  }
  std::vector<Observation> merge(const std::vector<Candidate>&all,const std::string&family,double target) const {
    const cv::Point2d dir(std::cos(rad(target)),std::sin(rad(target))), normal(-dir.y,dir.x);
    struct Item{const Candidate*c;double lo,hi,across,err;};std::vector<Item>items;
    for(const auto&c:all){const double err=angle_diff180(c.angle,target);if(err>p_.dominant_tol)continue;
      const auto dot=[](const cv::Point2f&p,const cv::Point2d&q){return p.x*q.x+p.y*q.y;};
      double a0=dot(c.p0_m,dir),a1=dot(c.p1_m,dir);items.push_back({&c,std::min(a0,a1),std::max(a0,a1),0.5*(dot(c.p0_m,normal)+dot(c.p1_m,normal)),err});}
    std::sort(items.begin(),items.end(),[](auto&a,auto&b){return a.across<b.across;});
    struct Cluster{std::vector<Item>it;double lo,hi,across,wsum;};std::vector<Cluster>cs;
    for(const auto&i:items){int best=-1;double cost=1e9;for(size_t j=0;j<cs.size();++j){double da=std::abs(i.across-cs[j].across);double gap=std::max({cs[j].lo-i.hi,i.lo-cs[j].hi,0.0});if(da<=p_.merge_distance&&gap<=p_.merge_gap&&da+0.25*gap<cost){best=j;cost=da+0.25*gap;}}
      const double w=std::max(i.c->score*i.c->length,1e-3);if(best<0)cs.push_back({{i},i.lo,i.hi,i.across,w});else{auto&c=cs[best];c.across=(c.across*c.wsum+i.across*w)/(c.wsum+w);c.wsum+=w;c.lo=std::min(c.lo,i.lo);c.hi=std::max(c.hi,i.hi);c.it.push_back(i);}}
    std::vector<Observation>out;for(const auto&c:cs){const double span=c.hi-c.lo;if(span<p_.final_min_span)continue;double q=0,width=0,err=0,color=0,response=0,ws=0;for(const auto&i:c.it){double w=std::max(i.c->score*i.c->length,1e-3);ws+=w;q+=w*i.c->score;width+=w*i.c->width;err+=w*i.err;color+=w*i.c->color;response+=w*i.c->response;}q/=ws;width/=ws;err/=ws;color/=ws;response/=ws;
      const double final=clamp01(0.82*q+0.18*clamp01(1.0-err/std::max(p_.dominant_tol,1.0))+std::min(0.10,0.025*std::max(0,static_cast<int>(c.it.size())-1)));
      if(final<p_.final_min_score || (color<0.12 && !(span>=1.5&&response>=0.8&&final>=0.62)))continue;
      cv::Point2f p0(static_cast<float>(dir.x*c.lo+normal.x*c.across),static_cast<float>(dir.y*c.lo+normal.y*c.across));
      cv::Point2f p1(static_cast<float>(dir.x*c.hi+normal.x*c.across),static_cast<float>(dir.y*c.hi+normal.y*c.across));out.push_back({p0,p1,family,target,span,width,final});}
    std::sort(out.begin(),out.end(),[](auto&a,auto&b){return a.confidence>b.confidence;});if(static_cast<int>(out.size())>p_.max_each)out.resize(p_.max_each);return out;
  }
  static double point_segment(const cv::Point2f&p,const cv::Point2f&a,const cv::Point2f&b){cv::Point2f ab=b-a;double den=ab.dot(ab);if(den<1e-12)return cv::norm(p-a);double t=std::clamp((p-a).dot(ab)/den,0.0,1.0);return cv::norm(p-(a+static_cast<float>(t)*ab));}
  void geometric_filter(std::vector<Observation>&a,std::vector<Observation>&b) const {
    auto score=[&](Observation&v,const std::vector<Observation>&same,const std::vector<Observation>&other){double par=0,orth=0;cv::Point2f d=v.p1_m-v.p0_m;double n=cv::norm(d);if(n>1e-9){d*=1.0f/n;cv::Point2f norm(-d.y,d.x);cv::Point2f mid=0.5f*(v.p0_m+v.p1_m);for(const auto&s:same)if(&s!=&v){double spacing=std::abs((0.5f*(s.p0_m+s.p1_m)-mid).dot(norm));if(spacing>=p_.parallel_min&&spacing<=p_.parallel_max)par=1;}for(const auto&o:other){double near=std::min({point_segment(v.p0_m,o.p0_m,o.p1_m),point_segment(v.p1_m,o.p0_m,o.p1_m),point_segment(o.p0_m,v.p0_m,v.p1_m),point_segment(o.p1_m,v.p0_m,v.p1_m)});if(near<=p_.orthogonal_near)orth=1;}}
      v.confidence=p_.inference_appearance_w*v.confidence+p_.inference_parallel_w*par+p_.inference_orthogonal_w*orth;};
    for(auto&v:a) score(v,a,b);
    for(auto&v:b) score(v,b,a);
    auto keep=[&](const Observation&v){return v.confidence>=p_.inference_min_score||v.confidence>=p_.strong_single_score;};
    a.erase(std::remove_if(a.begin(),a.end(),[&](auto&v){return !keep(v);}),a.end());b.erase(std::remove_if(b.begin(),b.end(),[&](auto&v){return !keep(v);}),b.end());
  }
  Params p_; cv::Mat map_x_,map_y_,valid_; cv::Size source_size_; std::vector<double>thresholds_; int threshold_frame_{0};
};

#if VISION_LANE_HAS_OPENCV_CUDA
class CudaPreprocessor {
 public:
  explicit CudaPreprocessor(const Params &p) : p_(p) {}

  Prep preprocess(const cv::Mat &frame, Pipeline &pipeline,
                  double *bev_ms, double *preprocess_ms) {
    ensure_initialized(pipeline.valid_mask().size(), pipeline.valid_mask(),
                       pipeline.map_x(), pipeline.map_y());
    auto stage_start = Clock::now();
    d_input_.upload(frame);
    cv::cuda::remap(d_input_, d_bev_, d_map_x_, d_map_y_, cv::INTER_LINEAR,
                    cv::BORDER_CONSTANT, cv::Scalar());
    cv::cuda::Stream::Null().waitForCompletion();
    *bev_ms = ms_since(stage_start);
    stage_start = Clock::now();
    cv::cuda::cvtColor(d_bev_, d_hsv_, cv::COLOR_BGR2HSV);
    cv::cuda::split(d_hsv_, hsv_channels_);

    // Threshold single HSV channels instead of cv::cuda::inRange so this also
    // builds on the OpenCV 4.x CUDA versions commonly shipped with JetPack.
    ge(hsv_channels_[2], p_.white_v_min, d_tmp0_);
    le(hsv_channels_[1], p_.white_s_max, d_tmp1_);
    cv::cuda::bitwise_and(d_tmp0_, d_tmp1_, d_white_);

    ge(hsv_channels_[0], p_.yellow_h_min, d_tmp0_);
    le(hsv_channels_[0], p_.yellow_h_max, d_tmp1_);
    cv::cuda::bitwise_and(d_tmp0_, d_tmp1_, d_yellow_h_);
    ge(hsv_channels_[1], p_.yellow_s_min, d_tmp0_);
    cv::cuda::bitwise_and(d_yellow_h_, d_tmp0_, d_yellow_hs_);
    ge(hsv_channels_[2], p_.yellow_v_min, d_tmp0_);
    cv::cuda::bitwise_and(d_yellow_hs_, d_tmp0_, d_yellow_);
    cv::cuda::bitwise_or(d_white_, d_yellow_, d_color_);
    ge(hsv_channels_[2], p_.glare_v_min, d_tmp0_);
    le(hsv_channels_[1], p_.glare_s_max, d_tmp1_);
    cv::cuda::bitwise_and(d_tmp0_, d_tmp1_, d_glare_seed_);

    // The wide illumination Gaussian runs on the downscaled CPU image. Its
    // kernel can exceed CUDA's practical filter width, while this transfer is
    // only illumination_scale^2 of the full BEV. All full-resolution filters
    // stay on the GPU.
    hsv_channels_[2].convertTo(d_source_f_, CV_32F, 1.0, 1.0);
    cv::cuda::resize(d_source_f_, d_small_, small_size_, 0.0, 0.0, cv::INTER_AREA);
    d_small_.download(small_cpu_);
    cv::GaussianBlur(small_cpu_, small_cpu_, {0,0}, illumination_sigma_,
                     illumination_sigma_, cv::BORDER_REFLECT);
    d_small_blurred_.upload(small_cpu_);
    cv::cuda::resize(d_small_blurred_, d_illum_, bev_size_, 0.0, 0.0,
                     cv::INTER_LINEAR);
    cv::cuda::add(d_illum_, cv::Scalar::all(1e-6), d_denom_);
    cv::cuda::divide(d_source_f_, d_denom_, d_norm_f_, p_.normalization_scale);
    d_norm_f_.convertTo(d_normalized_, CV_8U);
    cv::cuda::bitwise_and(d_normalized_, d_valid_, d_masked_normalized_);
    blur3_->apply(d_masked_normalized_, d_blurred_);
    top_hat_->apply(d_blurred_, d_response_);

    // One full-resolution download: B=color, G=glare seed, R=Top-hat response.
    // connected-components and band MAD remain on CPU to preserve v5.2 logic.
    std::vector<cv::cuda::GpuMat> output_channels{d_color_,d_glare_seed_,d_response_};
    cv::cuda::merge(output_channels,d_packed_);
    cv::Mat packed;
    d_packed_.download(packed);
    std::vector<cv::Mat> host_channels;
    cv::split(packed,host_channels);
    Prep out=pipeline.finalize_preprocess(host_channels[0],host_channels[1],host_channels[2]);
    *preprocess_ms = ms_since(stage_start);
    return out;
  }

 private:
  static void ge(const cv::cuda::GpuMat &src, int minimum,
                 cv::cuda::GpuMat &dst) {
    cv::cuda::threshold(src,dst,static_cast<double>(minimum-1),255.0,
                        cv::THRESH_BINARY);
  }

  static void le(const cv::cuda::GpuMat &src, int maximum,
                 cv::cuda::GpuMat &dst) {
    cv::cuda::threshold(src,dst,static_cast<double>(maximum),255.0,
                        cv::THRESH_BINARY_INV);
  }

  void ensure_initialized(const cv::Size &bev_size, const cv::Mat &valid,
                          const cv::Mat &map_x, const cv::Mat &map_y) {
    if (bev_size == bev_size_ && !d_valid_.empty()) return;
    bev_size_ = bev_size;
    small_size_ = {
      std::max(1, static_cast<int>(std::llround(bev_size.width * p_.illumination_scale))),
      std::max(1, static_cast<int>(std::llround(bev_size.height * p_.illumination_scale)))
    };
    illumination_sigma_ = std::max(
      1.0, p_.illumination_sigma_m / p_.mpp * p_.illumination_scale);
    d_valid_.upload(valid);
    d_map_x_.upload(map_x);
    d_map_y_.upload(map_y);
    blur3_ = cv::cuda::createGaussianFilter(
      CV_8UC1, CV_8UC1, {3,3}, 0.0, 0.0, cv::BORDER_REFLECT);
    const int tk = std::max(
      3, static_cast<int>(std::llround(p_.tophat_size_m / p_.mpp)) | 1);
    top_hat_ = cv::cuda::createMorphologyFilter(
      cv::MORPH_TOPHAT, CV_8UC1,
      cv::getStructuringElement(cv::MORPH_RECT, {tk,tk}));
  }

  Params p_;
  cv::Size bev_size_, small_size_;
  double illumination_sigma_{1.0};
  cv::Mat small_cpu_;
  std::vector<cv::cuda::GpuMat> hsv_channels_;
  cv::Ptr<cv::cuda::Filter> blur3_, top_hat_;
  cv::cuda::GpuMat d_input_, d_bev_, d_hsv_, d_valid_, d_map_x_, d_map_y_;
  cv::cuda::GpuMat d_tmp0_, d_tmp1_, d_white_, d_yellow_h_, d_yellow_hs_;
  cv::cuda::GpuMat d_yellow_, d_color_, d_glare_seed_;
  cv::cuda::GpuMat d_source_f_, d_small_, d_small_blurred_, d_illum_, d_denom_;
  cv::cuda::GpuMat d_norm_f_, d_normalized_, d_masked_normalized_, d_blurred_;
  cv::cuda::GpuMat d_response_, d_packed_;
};
#endif

class Tracker {
 public:
  explicit Tracker(const Params&p):p_(p){}
  void reset(){tracks_.clear();next_id_=1;}
  void update(const std::vector<Observation>&obs,const Pose2&pose){
    for(auto&x:tracks_){x.hits.push_back(0);while(static_cast<int>(x.hits.size())>p_.history)x.hits.pop_front();++x.missed;}
    struct Pair{double cost;size_t ti,oi;};std::vector<Pair>pairs;
    std::vector<std::pair<cv::Point2f,cv::Point2f>> global;
    for(const auto&o:obs)global.push_back({base_to_global(metric_to_base(o.p0_m),pose),base_to_global(metric_to_base(o.p1_m),pose)});
    for(size_t ti=0;ti<tracks_.size();++ti)for(size_t oi=0;oi<obs.size();++oi){auto cost=association(tracks_[ti],global[oi]);if(cost)pairs.push_back({*cost,ti,oi});}
    std::sort(pairs.begin(),pairs.end(),[](auto&a,auto&b){return a.cost<b.cost;});std::vector<bool>ut(tracks_.size()),uo(obs.size());
    for(const auto&m:pairs){if(ut[m.ti]||uo[m.oi])continue;auto&t=tracks_[m.ti];auto g=global[m.oi];if((t.g1-t.g0).dot(g.second-g.first)<0)std::swap(g.first,g.second);const float alpha=static_cast<float>(p_.ema);t.g0=(1.0f-alpha)*t.g0+alpha*g.first;t.g1=(1.0f-alpha)*t.g1+alpha*g.second;t.family=obs[m.oi].family;t.width=obs[m.oi].width;t.confidence=obs[m.oi].confidence;t.hits.back()=1;++t.hits_total;t.missed=0;if(std::accumulate(t.hits.begin(),t.hits.end(),0)>=p_.confirm_hits)t.confirmed=true;ut[m.ti]=uo[m.oi]=true;}
    for(size_t i=0;i<obs.size();++i)if(!uo[i]){Primitive t;t.id=next_id_++;t.g0=global[i].first;t.g1=global[i].second;t.family=obs[i].family;t.width=obs[i].width;t.confidence=obs[i].confidence;t.hits.push_back(1);t.confirmed=p_.confirm_hits<=1;tracks_.push_back(t);}
    tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](auto&t){return t.missed>p_.max_missed;}),tracks_.end());
  }
  std::vector<Observation> project(const Pose2&pose,bool confirmed_only=false,bool force_reprojected=false)const{std::vector<Observation>out;for(const auto&t:tracks_){if(confirmed_only&&!t.confirmed)continue;cv::Point2f b0=global_to_base(t.g0,pose),b1=global_to_base(t.g1,pose);Observation o;o.p0_m=base_to_metric(b0);o.p1_m=base_to_metric(b1);o.family=t.family;o.span=cv::norm(o.p1_m-o.p0_m);o.width=t.width;o.confidence=t.confidence;o.track_id=t.id;o.hits_total=t.hits_total;o.hits_window=std::accumulate(t.hits.begin(),t.hits.end(),0);o.confirmed=t.confirmed;o.reprojected=force_reprojected||t.missed>0;out.push_back(o);}return out;}
  const std::vector<Primitive>&primitives()const{return tracks_;}
 private:
  std::optional<double>association(const Primitive&t,const std::pair<cv::Point2f,cv::Point2f>&g)const{cv::Point2f od=t.g1-t.g0,nd=g.second-g.first;double oa=angle180(deg(std::atan2(od.y,od.x))),na=angle180(deg(std::atan2(nd.y,nd.x)));double ae=angle_diff180(oa,na);if(ae>p_.temporal_angle)return{};double n=cv::norm(od);if(n<1e-9)return{};od*=1.0f/n;cv::Point2f normal(-od.y,od.x);double offset=std::abs((0.5f*(g.first+g.second)-0.5f*(t.g0+t.g1)).dot(normal));if(offset>p_.temporal_offset)return{};double o0=t.g0.dot(od),o1=t.g1.dot(od),n0=g.first.dot(od),n1=g.second.dot(od);double gap=std::max({std::min(o0,o1)-std::max(n0,n1),std::min(n0,n1)-std::max(o0,o1),0.0});if(gap>p_.temporal_gap)return{};return ae/p_.temporal_angle+offset/p_.temporal_offset+0.25*gap/p_.temporal_gap;}
  Params p_;uint32_t next_id_{1};std::vector<Primitive>tracks_;
};

class MarkingMap {
 public:
  explicit MarkingMap(const Params&p):p_(p){h_=static_cast<int>(std::ceil((p_.map_x_max-p_.map_x_min)/p_.map_res));w_=static_cast<int>(std::ceil((p_.map_y_max-p_.map_y_min)/p_.map_res));evidence_=cv::Mat::zeros(h_,w_,CV_32F);}
  MapUpdateResult update(const std::vector<Primitive>&tracks,const Pose2&pose){MapUpdateResult result;if(last_){result.translation_m=std::hypot(pose.x-last_->x,pose.y-last_->y);result.yaw_deg=deg(std::abs(wrap_pi(pose.yaw-last_->yaw)));result.motion_detected=result.translation_m>=p_.map_min_translation||result.yaw_deg>=p_.map_min_yaw_deg;if(!result.motion_detected)return result;}else{result.initial_seed=true;}cv::Mat frame=cv::Mat::zeros(h_,w_,CV_8U);for(const auto&t:tracks)if(t.confirmed){auto a=pixel(t.g0),b=pixel(t.g1);if(!cv::clipLine(cv::Size(w_,h_),a,b))continue;const int thickness=std::max(1,static_cast<int>(std::llround(std::max(t.width,p_.map_res)/p_.map_res)));cv::line(frame,a,b,cv::Scalar(255),thickness,cv::LINE_8);}if(cv::countNonZero(frame)==0)return result;for(int y=0;y<h_;++y)for(int x=0;x<w_;++x)if(frame.at<uint8_t>(y,x)){float&v=evidence_.at<float>(y,x);v=std::min(static_cast<float>(p_.map_max),v+static_cast<float>(p_.map_inc));}last_=pose;++updates_;result.accumulated=true;return result;}
  cv::Mat render()const{cv::Mat n,u8,color;cv::min(evidence_/std::max(p_.map_confirm,1e-6),cv::Scalar(1.0),n);n.convertTo(u8,CV_8U,255);cv::applyColorMap(u8,color,cv::COLORMAP_TURBO);color.setTo(cv::Scalar(),u8==0);return color;}
  int updates()const{return updates_;}
 private:cv::Point pixel(const cv::Point2f&p)const{return {static_cast<int>(std::llround((p_.map_y_max-p.y)/p_.map_res)),static_cast<int>(std::llround((p_.map_x_max-p.x)/p_.map_res))};}Params p_;int h_,w_,updates_{0};cv::Mat evidence_;std::optional<Pose2>last_;
};

class GlobalMappingNode : public rclcpp::Node {
 public:
  GlobalMappingNode():Node("global_mapping_cpp"),p_(load_params()),pipeline_(p_),tracker_(p_),map_(p_){
    input_mode_=declare_parameter<std::string>("input_mode","topic");
    std::transform(input_mode_.begin(),input_mode_.end(),input_mode_.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
    image_topic_=declare_parameter<std::string>("image_topic","/camera/image_raw");
    raw_image_topic_=declare_parameter<std::string>("raw_image_topic","/parking/cpp/image_raw");
    odom_topic_=declare_parameter<std::string>("odom_topic","/parking/odom");
    output_topic_=declare_parameter<std::string>("output_topic","/parking/cpp/marking_segments");
    map_topic_=declare_parameter<std::string>("map_image_topic","/parking/cpp/global_marking_map");
    status_topic_=declare_parameter<std::string>("status_topic","/parking/cpp/status");
    calibration_=declare_parameter<std::string>("calibration_yaml","");
    camera_id_=declare_parameter<std::string>("camera_id","front");
    camera_frame_=declare_parameter<std::string>("camera_frame","front_camera_optical_frame");
    map_frame_=declare_parameter<std::string>("map_frame","map");
    base_frame_=declare_parameter<std::string>("base_frame","base_link");

    use_static_=declare_parameter<bool>("use_static_pose",true);
    static_pose_.x=declare_parameter<double>("static_pose_x",0);
    static_pose_.y=declare_parameter<double>("static_pose_y",0);
    static_pose_.yaw=rad(declare_parameter<double>("static_pose_yaw_deg",0));
    odom_buffer_duration_sec_=declare_parameter<double>("odom_buffer_duration_sec",2.0);
    max_odom_age_sec_=declare_parameter<double>("max_odom_age_sec",0.20);
    max_odom_interpolation_gap_sec_=declare_parameter<double>("max_odom_interpolation_gap_sec",0.25);
    require_odom_for_mapping_=declare_parameter<bool>("require_odom_for_mapping",true);
    allow_local_without_odom_=declare_parameter<bool>("allow_local_without_odom",true);
    sync_images_to_odom_=declare_parameter<bool>("sync_images_to_odom",false);
    sync_max_wait_sec_=declare_parameter<double>("sync_max_wait_sec",0.25);
    sync_queue_size_=declare_parameter<int>("sync_queue_size",8);
    startup_warmup_frames_=declare_parameter<int>("startup_warmup_frames",0);
    image_quality_gate_mode_=declare_parameter<std::string>("image_quality_gate_mode","monitor");
    std::transform(image_quality_gate_mode_.begin(),image_quality_gate_mode_.end(),
      image_quality_gate_mode_.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
    image_quality_min_sharpness_=declare_parameter<double>("image_quality_min_sharpness",25.0);
    image_quality_sample_width_=declare_parameter<int>("image_quality_sample_width",640);
    if(sync_images_to_odom_&&(input_mode_!="topic"||use_static_))
      throw std::runtime_error("sync_images_to_odom requires topic mode and use_static_pose=false");
    if(!std::isfinite(sync_max_wait_sec_)||sync_max_wait_sec_<=0||sync_queue_size_<1||
       startup_warmup_frames_<0||!std::isfinite(max_odom_age_sec_)||max_odom_age_sec_<=0||
       !std::isfinite(max_odom_interpolation_gap_sec_)||max_odom_interpolation_gap_sec_<=0)
      throw std::runtime_error("invalid synchronization limits");
    if((image_quality_gate_mode_!="off"&&image_quality_gate_mode_!="monitor"&&
        image_quality_gate_mode_!="reject")||!std::isfinite(image_quality_min_sharpness_)||
       image_quality_min_sharpness_<0.0||image_quality_sample_width_<64)
      throw std::runtime_error("invalid image quality gate configuration");
    pose_timeline_=vision_lane::PoseTimeline(odom_buffer_duration_sec_);

    publish_map_=declare_parameter<bool>("publish_debug_map",false);
    publish_vectors_debug_=declare_parameter<bool>("publish_debug_vectors",false);
    map_publish_every_n_=std::max(
      1,static_cast<int>(declare_parameter<int>("map_publish_every_n",5)));
    publish_raw_=declare_parameter<bool>("publish_raw_image",false);
    profile_path_=declare_parameter<std::string>("profile_csv_path","/tmp/v52_cpp_profile.csv");
    report_every_=std::max(
      1,static_cast<int>(declare_parameter<int>("profile_report_every",120)));
    preprocess_backend_requested_=declare_parameter<std::string>("preprocess_backend","cpu");
    std::transform(preprocess_backend_requested_.begin(),preprocess_backend_requested_.end(),
      preprocess_backend_requested_.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
    cuda_device_id_=declare_parameter<int>("cuda_device_id",0);
    cuda_fallback_to_cpu_=declare_parameter<bool>("cuda_fallback_to_cpu",false);
    configure_preprocessor();

    v4l2_device_=declare_parameter<std::string>("v4l2_device","/dev/video2");
    capture_width_=declare_parameter<int>("capture_width",2560);
    capture_height_=declare_parameter<int>("capture_height",1440);
    capture_fps_=declare_parameter<double>("capture_fps",30.0);
    capture_fourcc_=declare_parameter<std::string>("capture_fourcc","MJPG");
    capture_buffer_size_=declare_parameter<int>("capture_buffer_size",1);
    capture_poll_ms_=declare_parameter<int>("capture_poll_ms",1);

    segment_pub_=create_publisher<vision_lane_interfaces::msg::MarkingSegmentArray>(output_topic_,10);
    map_pub_=create_publisher<sensor_msgs::msg::Image>(map_topic_,2);
    profile_pub_=create_publisher<std_msgs::msg::String>("/parking/cpp/profile",10);
    status_pub_=create_publisher<std_msgs::msg::String>(status_topic_,10);
    sync_status_pub_=create_publisher<std_msgs::msg::String>(status_topic_+"/sync",10);
    if(publish_vectors_debug_){
      binary_pub_=create_publisher<sensor_msgs::msg::Image>("/parking/cpp/binary",rclcpp::SensorDataQoS());
      vector_debug_pub_=create_publisher<sensor_msgs::msg::Image>("/parking/cpp/vector_bev",rclcpp::SensorDataQoS());
    }
    if(publish_raw_)raw_pub_=create_publisher<sensor_msgs::msg::Image>(raw_image_topic_,rclcpp::SensorDataQoS());
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(odom_topic_,rclcpp::SensorDataQoS(),std::bind(&GlobalMappingNode::on_odom,this,std::placeholders::_1));

    if(!profile_path_.empty()){
      csv_.open(profile_path_);
      if(!csv_)throw std::runtime_error("cannot open profile CSV: "+profile_path_);
      csv_<<"frame,bev_ms,preprocess_ms,hough_ms,temporal_ms,mapping_ms,core_total_ms,raw,observations,active,confirmed,input_age_ms,pose_age_ms,pose_valid,motion_detected,map_accumulated,map_updates,preprocess_backend,image_stamp_ns,sync_wait_ms,warming_up,sync_matched,sync_timeout,sync_dropped,sharpness_laplacian_variance,frame_delta_mean,quality_rejected\n";
    }

    if(input_mode_=="topic"){
      image_sub_=create_subscription<sensor_msgs::msg::Image>(image_topic_,rclcpp::SensorDataQoS().keep_last(2),std::bind(&GlobalMappingNode::on_image,this,std::placeholders::_1));
    }else if(input_mode_=="v4l2"){
      start_v4l2();
    }else{
      throw std::runtime_error("input_mode must be 'topic' or 'v4l2'");
    }
    if(sync_images_to_odom_){
      sync_timer_=create_wall_timer(std::chrono::milliseconds(2),std::bind(&GlobalMappingNode::drain_images,this));
      sync_report_timer_=create_wall_timer(std::chrono::seconds(1),std::bind(&GlobalMappingNode::report_sync,this));
    }
    RCLCPP_INFO(get_logger(),"v5.2 C++ node: input=%s source=%s BEV=%.3fm/px vectorize=1/%d preprocess=%s odom=%s quality=%s(min %.1f)",
      input_mode_.c_str(),input_mode_=="v4l2"?v4l2_device_.c_str():image_topic_.c_str(),p_.mpp,p_.vectorize_every_n,
      preprocess_backend_active_.c_str(),odom_topic_.c_str(),image_quality_gate_mode_.c_str(),image_quality_min_sharpness_);
  }

  ~GlobalMappingNode() override {
    capture_running_.store(false);
    if(capture_thread_.joinable())capture_thread_.join();
    capture_.release();
    if(csv_)csv_.flush();
  }

 private:
  void configure_preprocessor(){
    if(preprocess_backend_requested_!="cpu"&&preprocess_backend_requested_!="cuda")
      throw std::runtime_error("preprocess_backend must be 'cpu' or 'cuda'");
    preprocess_backend_active_="cpu";
    if(preprocess_backend_requested_=="cpu")return;
#if VISION_LANE_HAS_OPENCV_CUDA
    try{
      const int count=cv::cuda::getCudaEnabledDeviceCount();
      if(count<=0)throw std::runtime_error("OpenCV found no CUDA-enabled device");
      if(cuda_device_id_<0||cuda_device_id_>=count)
        throw std::runtime_error("cuda_device_id is outside the detected device range");
      cv::cuda::setDevice(cuda_device_id_);
      cuda_preprocessor_=std::make_unique<CudaPreprocessor>(p_);
      preprocess_backend_active_="cuda";
      RCLCPP_INFO(get_logger(),"CUDA preprocessing enabled on device %d (%d CUDA device(s) detected)",cuda_device_id_,count);
    }catch(const std::exception&e){
      if(!cuda_fallback_to_cpu_)throw;
      RCLCPP_WARN(get_logger(),"CUDA preprocessing unavailable (%s); using CPU fallback",e.what());
    }
#else
    const std::string reason="this OpenCV build does not provide cudaarithm/cudaimgproc/cudawarping/cudafilters";
    if(!cuda_fallback_to_cpu_)throw std::runtime_error("CUDA preprocessing requested but "+reason);
    RCLCPP_WARN(get_logger(),"CUDA preprocessing requested but %s; using CPU fallback",reason.c_str());
#endif
  }

  Prep preprocess_frame(const cv::Mat&input,double*bev_ms,double*preprocess_ms){
#if VISION_LANE_HAS_OPENCV_CUDA
    if(preprocess_backend_active_=="cuda"){
      try{return cuda_preprocessor_->preprocess(input,pipeline_,bev_ms,preprocess_ms);}
      catch(const cv::Exception&e){
        if(!cuda_fallback_to_cpu_)throw;
        preprocess_backend_active_="cpu";
        cuda_preprocessor_.reset();
        RCLCPP_ERROR(get_logger(),"CUDA preprocessing failed at runtime (%s); permanently switching to CPU",e.what());
      }
    }
#endif
    auto t=Clock::now();
    cv::Mat bev=pipeline_.remap_bev(input);
    *bev_ms=ms_since(t);
    t=Clock::now();
    Prep out=pipeline_.preprocess(bev);
    *preprocess_ms=ms_since(t);
    return out;
  }

  std::pair<double,double> measure_image_quality(const cv::Mat&input){
    if(image_quality_gate_mode_=="off")return {0.0,0.0};
    cv::Mat sample;
    if(input.cols>image_quality_sample_width_){
      const double scale=image_quality_sample_width_/static_cast<double>(input.cols);
      cv::resize(input,sample,{},scale,scale,cv::INTER_AREA);
    }else sample=input;
    cv::Mat gray;
    if(sample.channels()==3)cv::cvtColor(sample,gray,cv::COLOR_BGR2GRAY);
    else if(sample.channels()==4)cv::cvtColor(sample,gray,cv::COLOR_BGRA2GRAY);
    else gray=sample;
    cv::Mat laplacian;
    cv::Laplacian(gray,laplacian,CV_32F,3);
    cv::Scalar mean,stddev;
    cv::meanStdDev(laplacian,mean,stddev);
    const double sharpness=stddev[0]*stddev[0];
    double delta=0.0;
    if(!last_quality_gray_.empty()&&last_quality_gray_.size()==gray.size()){
      cv::Mat difference;
      cv::absdiff(gray,last_quality_gray_,difference);
      delta=cv::mean(difference)[0];
    }
    gray.copyTo(last_quality_gray_);
    return {sharpness,delta};
  }

  Params load_params(){Params p;
#define D(name,field,val) p.field=declare_parameter<double>(name,val)
#define I(name,field,val) p.field=declare_parameter<int>(name,val)
    D("camera_height_m",camera_height,0.8);D("camera_forward_m",camera_forward,0);D("camera_left_m",camera_left,0);D("pitch_deg",pitch_deg,30);D("roll_deg",roll_deg,0);D("yaw_deg",yaw_deg,0);D("x_min_m",x_min,-4);D("x_max_m",x_max,4);D("y_min_m",y_min,0);D("y_max_m",y_max,5);D("meters_per_pixel",mpp,0.02);
    I("white_s_max",white_s_max,60);I("white_v_min",white_v_min,140);I("yellow_h_min",yellow_h_min,15);I("yellow_h_max",yellow_h_max,40);I("yellow_s_min",yellow_s_min,70);I("yellow_v_min",yellow_v_min,70);I("glare_v_min",glare_v_min,248);I("glare_s_max",glare_s_max,95);D("glare_min_area_ratio",glare_min_area_ratio,0.006);I("glare_dilate",glare_dilate,9);
    D("illumination_sigma_m",illumination_sigma_m,0.60);D("illumination_scale",illumination_scale,0.35);D("normalization_scale",normalization_scale,128);D("tophat_size_m",tophat_size_m,0.30);I("threshold_bands",threshold_bands,6);I("threshold_update_interval",threshold_update_interval,3);I("threshold_sample_stride",threshold_sample_stride,4);D("mad_k",mad_k,5.5);D("threshold_margin",threshold_margin,3);D("threshold_min",threshold_min,8);D("threshold_max",threshold_max,80);D("strong_response_margin",strong_response_margin,36);D("glare_recovery_margin",glare_recovery_margin,20);
    I("hough_threshold",hough_threshold,28);D("hough_min_line_length_m",hough_min_len,0.65);D("hough_max_line_gap_m",hough_max_gap,0.18);I("hough_max_raw_segments",max_raw,120);D("candidate_min_length_m",candidate_min_len,0.70);D("candidate_min_binary_support",min_binary,0.72);D("candidate_min_color_support",min_color,0.08);D("candidate_min_score",candidate_min_score,0.46);D("candidate_max_glare_ratio",max_glare,0.85);D("line_min_width_m",line_min_width,0.05);D("line_max_width_m",line_max_width,0.22);D("line_target_width_m",line_target_width,0.12);D("dominant_angle_tolerance_deg",dominant_tol,12);D("dominant_min_confidence",dominant_min_conf,0.35);D("merge_distance_m",merge_distance,0.20);D("merge_gap_m",merge_gap,0.35);D("final_min_span_m",final_min_span,0.85);D("final_min_score",final_min_score,0.52);I("max_vectors_each",max_each,8);I("vectorize_every_n",vectorize_every_n,2);
    I("temporal_history_size",history,5);I("temporal_confirm_hits",confirm_hits,3);D("temporal_angle_gate_deg",temporal_angle,7);D("temporal_offset_gate_m",temporal_offset,0.32);D("temporal_gap_gate_m",temporal_gap,0.60);I("temporal_max_missed_updates",max_missed,3);D("temporal_ema_alpha",ema,0.35);
    D("map_x_min_m",map_x_min,-2);D("map_x_max_m",map_x_max,20);D("map_y_min_m",map_y_min,-10);D("map_y_max_m",map_y_max,10);D("map_resolution_m",map_res,0.03);D("map_evidence_increment",map_inc,1);D("map_evidence_max",map_max,20);D("map_confirm_evidence",map_confirm,3);D("map_min_translation_m",map_min_translation,0.05);D("map_min_yaw_deg",map_min_yaw_deg,1);
#undef D
#undef I
    return p;}

  void start_v4l2(){
    if(!capture_.open(v4l2_device_,cv::CAP_V4L2)){
      throw std::runtime_error("failed to open V4L2 device: "+v4l2_device_);
    }
    if(capture_fourcc_.size()>=4){
      capture_.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc(capture_fourcc_[0],capture_fourcc_[1],capture_fourcc_[2],capture_fourcc_[3]));
    }
    capture_.set(cv::CAP_PROP_FRAME_WIDTH,capture_width_);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT,capture_height_);
    capture_.set(cv::CAP_PROP_FPS,capture_fps_);
    capture_.set(cv::CAP_PROP_BUFFERSIZE,std::max(1,capture_buffer_size_));
    RCLCPP_INFO(get_logger(),"V4L2 requested %dx%d %.1f FPS %s; negotiated %.0fx%.0f %.1f FPS",
      capture_width_,capture_height_,capture_fps_,capture_fourcc_.c_str(),capture_.get(cv::CAP_PROP_FRAME_WIDTH),capture_.get(cv::CAP_PROP_FRAME_HEIGHT),capture_.get(cv::CAP_PROP_FPS));
    capture_running_.store(true);
    capture_thread_=std::thread(&GlobalMappingNode::capture_loop,this);
    capture_timer_=create_wall_timer(std::chrono::milliseconds(std::max(1,capture_poll_ms_)),std::bind(&GlobalMappingNode::on_capture_timer,this));
  }

  void capture_loop(){
    int failures=0;
    while(capture_running_.load()&&rclcpp::ok()){
      cv::Mat frame;
      if(!capture_.read(frame)||frame.empty()){
        if(++failures%30==1)RCLCPP_WARN(get_logger(),"V4L2 frame read failed (%d)",failures);
        continue;
      }
      failures=0;
      const auto stamp=now();
      std::lock_guard<std::mutex>lock(capture_mutex_);
      latest_frame_=std::move(frame);
      latest_capture_stamp_=stamp;
      ++latest_capture_sequence_;
    }
  }

  void on_capture_timer(){
    cv::Mat frame;
    rclcpp::Time stamp(0,0,RCL_ROS_TIME);
    {
      std::lock_guard<std::mutex>lock(capture_mutex_);
      if(latest_capture_sequence_==processed_capture_sequence_||latest_frame_.empty())return;
      frame=latest_frame_;
      stamp=latest_capture_stamp_;
      processed_capture_sequence_=latest_capture_sequence_;
    }
    std_msgs::msg::Header header;
    header.stamp=stamp;
    header.frame_id=camera_frame_;
    if(raw_pub_)raw_pub_->publish(*cv_bridge::CvImage(header,"bgr8",frame).toImageMsg());
    process_frame(frame,header);
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg){
    ++received_images_;
    if(sync_images_to_odom_){
      const int64_t stamp=rclcpp::Time(msg->header.stamp).nanoseconds();
      if(stamp<=0||stamp<=last_received_image_ns_||msg->header.frame_id!=camera_frame_){
        ++sync_dropped_;
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
          "image rejected: expected frame=%s, positive increasing stamp (restart all nodes after clock/pose reset)",camera_frame_.c_str());
        return;
      }
      last_received_image_ns_=stamp;
      if(pending_images_.size()>=static_cast<size_t>(sync_queue_size_)){
        pending_images_.pop_front();++sync_dropped_;
      }
      pending_images_.push_back({msg,Clock::now()});
      return;
    }
    process_image_message(msg);
  }

  void process_image_message(const sensor_msgs::msg::Image::SharedPtr&msg){
    try{
      const auto cv_ptr=cv_bridge::toCvShare(msg,"bgr8");
      process_frame(cv_ptr->image,msg->header);
    }catch(const cv_bridge::Exception&e){
      RCLCPP_ERROR(get_logger(),"cv_bridge: %s",e.what());
    }
  }

  void drain_images(){
    if(pending_images_.empty())return;
    const auto &front=pending_images_.front();
    const auto lookup=lookup_pose(rclcpp::Time(front.message->header.stamp));
    const double waited=ms_since(front.arrived);
    if(!lookup.valid&&waited<sync_max_wait_sec_*1000.0)return;
    const auto message=front.message;
    pending_images_.pop_front();
    last_sync_wait_ms_=waited;
    if(lookup.valid)++sync_matched_;else ++sync_timeout_;
    // All callbacks use the default mutually-exclusive group. This never
    // blocks the odom callback while waiting for a future/exact sample.
    process_image_message(message);
  }

  void report_sync(){
    std::ostringstream ss;
    ss<<"{\"images_received\":"<<received_images_<<",\"pending\":"<<pending_images_.size()
      <<",\"matched\":"<<sync_matched_<<",\"timeout\":"<<sync_timeout_
      <<",\"dropped\":"<<sync_dropped_<<",\"rejected_odom\":"<<rejected_odom_
      <<",\"last_sync_wait_ms\":"<<last_sync_wait_ms_
      <<",\"warming_up\":"<<(frame_<=static_cast<uint64_t>(startup_warmup_frames_)?"true":"false")<<"}";
    std_msgs::msg::String msg;msg.data=ss.str();sync_status_pub_->publish(msg);
    if(received_images_==0||sync_matched_==0)
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "startup: images=%lu, matched=%lu, odom_rejected=%lu; waiting for shared images and aligned odom",
        received_images_,sync_matched_,rejected_odom_);
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
    const auto&q=msg->pose.pose.orientation;
    const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if(!std::isfinite(norm)||norm<1e-9||!std::isfinite(msg->pose.pose.position.x)||
       !std::isfinite(msg->pose.pose.position.y)||
       (sync_images_to_odom_&&(msg->header.frame_id!=map_frame_||msg->child_frame_id!=base_frame_))){
      ++rejected_odom_;
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "odom rejected: need finite pose/quaternion and frames %s -> %s",map_frame_.c_str(),base_frame_.c_str());
      return;
    }
    const double x=q.x/norm,y=q.y/norm,z=q.z/norm,w=q.w/norm;
    Pose2 pose{msg->pose.pose.position.x,msg->pose.pose.position.y,std::atan2(2*(w*z+x*y),1-2*(y*y+z*z))};
    rclcpp::Time stamp(msg->header.stamp);
    if(stamp.nanoseconds()==0){
      if(sync_images_to_odom_){++rejected_odom_;return;}
      stamp=now();
    }
    std::lock_guard<std::mutex>lock(odom_mutex_);
    if(!pose_timeline_.insert({stamp.nanoseconds(),pose.x,pose.y,pose.yaw}))++rejected_odom_;
  }

  PoseLookup lookup_pose(const rclcpp::Time&stamp){
    if(use_static_)return {static_pose_,true,false,0.0};
    std::lock_guard<std::mutex>lock(odom_mutex_);
    const auto result=pose_timeline_.lookup(stamp.nanoseconds(),max_odom_age_sec_,
      max_odom_interpolation_gap_sec_,!sync_images_to_odom_);
    return {{result.pose.x,result.pose.y,result.pose.yaw},result.valid,result.interpolated,result.age_sec};
  }

  void process_frame(const cv::Mat&input,const std_msgs::msg::Header&header){
    if(input.empty())return;
    Profile pr;
    pr.frame=++frame_;
    const auto all=Clock::now();
    const rclcpp::Time stamp(header.stamp);
    last_processed_stamp_ns_=stamp.nanoseconds();
    warming_up_=pr.frame<=static_cast<uint64_t>(startup_warmup_frames_);
    if(startup_warmup_frames_>0&&pr.frame==static_cast<uint64_t>(startup_warmup_frames_)+1){
      tracker_.reset();
      RCLCPP_INFO(get_logger(),"camera warm-up complete: tracker reset; mapping now requires confirmed lines and aligned odom");
    }
    pr.input_age=std::max(0.0,(now()-stamp).seconds()*1000.0);
    if(!maps_ready_){
      maps_ready_=pipeline_.initialize_maps(calibration_,input.cols,input.rows);
      if(!maps_ready_){RCLCPP_ERROR(get_logger(),"calibration load failed: %s",calibration_.c_str());return;}
      RCLCPP_INFO(get_logger(),"source=%dx%d -> metric BEV=%dx%d",input.cols,input.rows,static_cast<int>((p_.x_max-p_.x_min)/p_.mpp),static_cast<int>((p_.y_max-p_.y_min)/p_.mpp));
    }

    const PoseLookup lookup=lookup_pose(stamp);
    pr.pose_valid=lookup.valid;
    pr.pose_age=lookup.valid?lookup.age_sec*1000.0:-1.0;
    if(!lookup.valid&&!allow_local_without_odom_){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"waiting for timestamp-aligned odom; frame skipped");
      return;
    }
    const Pose2 pose=lookup.valid?lookup.pose:Pose2{};
    if(!tracking_pose_state_initialized_||tracking_pose_valid_!=lookup.valid){
      if(tracking_pose_state_initialized_)tracker_.reset();
      tracking_pose_valid_=lookup.valid;
      tracking_pose_state_initialized_=true;
    }

    std::tie(pr.sharpness,pr.frame_delta)=measure_image_quality(input);
    pr.quality_rejected=image_quality_gate_mode_=="reject"&&
      pr.sharpness<image_quality_min_sharpness_;
    if(pr.quality_rejected){
      ++quality_rejected_frames_;
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "blurred/vibrating frame excluded from vectorization and map: sharpness=%.1f < %.1f; frame_delta=%.1f",
        pr.sharpness,image_quality_min_sharpness_,pr.frame_delta);
    }

    auto t=Clock::now();
    std::vector<Observation>obs;
    cv::Mat debug_binary;
    const bool run=!pr.quality_rejected&&
      (frame_-1)%std::max(1,p_.vectorize_every_n)==0;
    if(run){
      Prep prep=preprocess_frame(input,&pr.bev,&pr.preprocess);
      if(publish_vectors_debug_)debug_binary=prep.binary;
      double dominant_angle=0,dominant_confidence=0;
      t=Clock::now();
      obs=pipeline_.vectorize(prep,&pr.raw,&dominant_angle,&dominant_confidence);
      pr.hough=ms_since(t);
      t=Clock::now();
      tracker_.update(obs,pose);
    }else{
      t=Clock::now();
    }
    const auto projected=tracker_.project(pose,false,!run);
    pr.temporal=ms_since(t);
    pr.observations=static_cast<int>(projected.size());
    pr.active=static_cast<int>(tracker_.primitives().size());
    pr.confirmed=static_cast<int>(std::count_if(tracker_.primitives().begin(),tracker_.primitives().end(),[](const auto&t){return t.confirmed;}));

    MapUpdateResult map_result;
    t=Clock::now();
    const bool mapping_pose_ok=lookup.valid||!require_odom_for_mapping_;
    if(run&&pr.confirmed>0&&mapping_pose_ok&&!warming_up_)map_result=map_.update(tracker_.primitives(),pose);
    pr.mapping=ms_since(t);
    pr.motion_detected=map_result.motion_detected;
    pr.map_accumulated=map_result.accumulated;
    pr.map_updates=map_.updates();
    pr.total=ms_since(all);

    write_profile(pr);
    publish_segments(header,pose,lookup.valid,projected);
    publish_status(pr,lookup,map_result);
    if(!debug_binary.empty()){
      auto debug_header=header;debug_header.frame_id="bev";
      binary_pub_->publish(*cv_bridge::CvImage(debug_header,"mono8",debug_binary).toImageMsg());
      cv::Mat overlay;cv::cvtColor(debug_binary,overlay,cv::COLOR_GRAY2BGR);
      for(const auto&o:projected){
        const auto pixel=[this](const cv::Point2f&p){return cv::Point(
          static_cast<int>(std::lround((p.x-p_.x_min)/p_.mpp)),
          static_cast<int>(std::lround((p_.y_max-p.y)/p_.mpp)));};
        cv::line(overlay,pixel(o.p0_m),pixel(o.p1_m),
          o.confirmed?cv::Scalar(0,255,0):cv::Scalar(0,140,255),2,cv::LINE_AA);
      }
      vector_debug_pub_->publish(*cv_bridge::CvImage(debug_header,"bgr8",overlay).toImageMsg());
    }
    if(publish_map_&&(map_result.accumulated||pr.frame%map_publish_every_n_==0)){
      std_msgs::msg::Header map_header;
      map_header.stamp=header.stamp;
      map_header.frame_id=map_frame_;
      map_pub_->publish(*cv_bridge::CvImage(map_header,"bgr8",map_.render()).toImageMsg());
    }
    if(!lookup.valid&&require_odom_for_mapping_){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"local vectors active, global map blocked: no fresh odom for image stamp");
    }
  }

  void publish_segments(const std_msgs::msg::Header&header,const Pose2&pose,bool pose_valid,const std::vector<Observation>&obs){
    vision_lane_interfaces::msg::MarkingSegmentArray out;
    out.header=header;
    out.header.frame_id=base_frame_;
    out.camera_id=camera_id_;
    out.pose_valid=pose_valid;
    out.observer_pose.x=pose.x;
    out.observer_pose.y=pose.y;
    out.observer_pose.theta=pose.yaw;
    for(const auto&o:obs){
      vision_lane_interfaces::msg::MarkingSegment s;
      s.track_id=o.track_id;s.camera_id=camera_id_;s.family=o.family;
      const auto a=metric_to_base(o.p0_m),b=metric_to_base(o.p1_m);
      s.start.x=a.x;s.start.y=a.y;s.end.x=b.x;s.end.y=b.y;
      s.angle_rad=std::atan2(b.y-a.y,b.x-a.x);s.span_m=o.span;s.mean_width_m=o.width;s.confidence=o.confidence;
      s.hits_total=o.hits_total;s.hits_in_window=o.hits_window;s.confirmed=o.confirmed;s.reprojected=o.reprojected;s.source_stamp=header.stamp;
      out.segments.push_back(s);
    }
    segment_pub_->publish(out);
  }

  void publish_status(const Profile&p,const PoseLookup&lookup,const MapUpdateResult&map_result){
    std::ostringstream ss;
    ss<<std::boolalpha<<"{\"frame\":"<<p.frame<<",\"input_mode\":\""<<input_mode_<<"\",\"preprocess_backend\":\""<<preprocess_backend_active_<<"\",\"pose_valid\":"<<lookup.valid
      <<",\"pose_interpolated\":"<<lookup.interpolated<<",\"pose_age_ms\":"<<(lookup.valid?lookup.age_sec*1000.0:-1.0)
      <<",\"initial_map_seed\":"<<map_result.initial_seed<<",\"motion_detected\":"<<map_result.motion_detected<<",\"motion_translation_m\":"<<map_result.translation_m
      <<",\"motion_yaw_deg\":"<<map_result.yaw_deg<<",\"map_accumulated\":"<<map_result.accumulated
      <<",\"map_updates\":"<<map_.updates()<<",\"active\":"<<p.active<<",\"confirmed\":"<<p.confirmed
      <<",\"warming_up\":"<<warming_up_<<",\"sync_wait_ms\":"<<last_sync_wait_ms_
      <<",\"sharpness\":"<<p.sharpness<<",\"frame_delta_mean\":"<<p.frame_delta
      <<",\"quality_gate_mode\":\""<<image_quality_gate_mode_<<"\",\"quality_rejected\":"<<p.quality_rejected
      <<",\"quality_rejected_total\":"<<quality_rejected_frames_<<"}";
    std_msgs::msg::String msg;msg.data=ss.str();status_pub_->publish(msg);
  }

  void write_profile(const Profile&p){
    if(csv_){
      csv_<<p.frame<<','<<p.bev<<','<<p.preprocess<<','<<p.hough<<','<<p.temporal<<','<<p.mapping<<','<<p.total<<','<<p.raw<<','<<p.observations<<','<<p.active<<','<<p.confirmed<<','<<p.input_age<<','<<p.pose_age<<','<<p.pose_valid<<','<<p.motion_detected<<','<<p.map_accumulated<<','<<p.map_updates<<','<<preprocess_backend_active_
        <<','<<last_processed_stamp_ns_<<','<<last_sync_wait_ms_<<','<<warming_up_
        <<','<<sync_matched_<<','<<sync_timeout_<<','<<sync_dropped_
        <<','<<p.sharpness<<','<<p.frame_delta<<','<<p.quality_rejected<<'\n';
    }
    samples_.push_back(p.total);
    if(samples_.size()>static_cast<size_t>(report_every_))samples_.erase(samples_.begin());
    if(p.frame%std::max(1,report_every_)==0){
      const double avg=std::accumulate(samples_.begin(),samples_.end(),0.0)/samples_.size();
      auto sorted=samples_;std::sort(sorted.begin(),sorted.end());
      const double p95=sorted[static_cast<size_t>(0.95*(sorted.size()-1))];
      std::ostringstream ss;ss<<"{\"frame\":"<<p.frame<<",\"core_avg_ms\":"<<avg<<",\"core_p95_ms\":"<<p95<<",\"active\":"<<p.active<<",\"confirmed\":"<<p.confirmed<<",\"map_updates\":"<<p.map_updates<<"}";
      std_msgs::msg::String msg;msg.data=ss.str();profile_pub_->publish(msg);
      if(csv_)csv_.flush();
      RCLCPP_INFO(get_logger(),"[PROFILE %lu] core avg=%.2fms P95=%.2fms last: BEV=%.2f pre=%.2f Hough=%.2f temporal=%.2f map=%.2f pose=%s accumulated=%s",
        p.frame,avg,p95,p.bev,p.preprocess,p.hough,p.temporal,p.mapping,p.pose_valid?"Y":"N",p.map_accumulated?"Y":"N");
    }
  }

  Params p_;
  Pipeline pipeline_;
  Tracker tracker_;
  MarkingMap map_;
  uint64_t frame_{0};
  bool maps_ready_{false},use_static_{true},publish_map_{false},publish_raw_{false};
  bool require_odom_for_mapping_{true},allow_local_without_odom_{true};
  bool tracking_pose_state_initialized_{false},tracking_pose_valid_{false};
  bool sync_images_to_odom_{false},warming_up_{false},publish_vectors_debug_{false};
  std::string image_quality_gate_mode_{"monitor"};
  int sync_queue_size_{8},startup_warmup_frames_{0};
  int image_quality_sample_width_{640};
  double image_quality_min_sharpness_{25.0};
  uint64_t quality_rejected_frames_{0};
  double sync_max_wait_sec_{0.25},last_sync_wait_ms_{0};
  uint64_t received_images_{0},sync_matched_{0},sync_timeout_{0},sync_dropped_{0},rejected_odom_{0};
  int64_t last_received_image_ns_{0},last_processed_stamp_ns_{0};
  struct PendingImage {
    sensor_msgs::msg::Image::SharedPtr message;
    Clock::time_point arrived;
  };
  std::deque<PendingImage> pending_images_;
  bool cuda_fallback_to_cpu_{false};
  Pose2 static_pose_;
  std::string input_mode_,image_topic_,raw_image_topic_,odom_topic_,output_topic_,map_topic_,status_topic_,calibration_,camera_id_,camera_frame_,map_frame_,base_frame_,profile_path_;
  std::string preprocess_backend_requested_,preprocess_backend_active_{"cpu"};
  std::string v4l2_device_,capture_fourcc_;
  int report_every_{120},map_publish_every_n_{5},capture_width_{2560},capture_height_{1440},capture_buffer_size_{1},capture_poll_ms_{1},cuda_device_id_{0};
  double capture_fps_{30.0},odom_buffer_duration_sec_{2.0},max_odom_age_sec_{0.20},max_odom_interpolation_gap_sec_{0.25};
  std::ofstream csv_;
  std::vector<double>samples_;

#if VISION_LANE_HAS_OPENCV_CUDA
  std::unique_ptr<CudaPreprocessor>cuda_preprocessor_;
#endif

  std::mutex odom_mutex_;
  vision_lane::PoseTimeline pose_timeline_;
  std::mutex capture_mutex_;
  cv::VideoCapture capture_;
  cv::Mat latest_frame_;
  cv::Mat last_quality_gray_;
  rclcpp::Time latest_capture_stamp_{0,0,RCL_ROS_TIME};
  uint64_t latest_capture_sequence_{0},processed_capture_sequence_{0};
  std::atomic<bool>capture_running_{false};
  std::thread capture_thread_;

  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr sync_timer_,sync_report_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  rclcpp::Publisher<vision_lane_interfaces::msg::MarkingSegmentArray>::SharedPtr segment_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_pub_,vector_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr profile_pub_,status_pub_,sync_status_pub_;
};
}  // namespace

int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<GlobalMappingNode>());rclcpp::shutdown();return 0;}
