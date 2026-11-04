#pragma once

// Lane detection on one image: BEV remap, preprocess, vectorize (E-8). Moved out of
// global_mapping_node.cpp unchanged so the node and an offline replay run the same
// code; test/lane_pipeline_golden checks the move is byte-identical on recorded frames.
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "lane_map/lane_tracker.hpp"

namespace lane_detection {
constexpr double kPi = 3.14159265358979323846;

inline double rad(double deg) { return deg * kPi / 180.0; }
inline double deg(double r) { return r * 180.0 / kPi; }
inline double wrap_pi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}
inline double angle180(double a) {
  a = std::fmod(a, 180.0);
  return a < 0.0 ? a + 180.0 : a;
}
inline double angle_diff180(double a, double b) {
  const double d = std::abs(angle180(a) - angle180(b));
  return std::min(d, 180.0 - d);
}
inline double orthogonal_error(double a, double base) {
  return std::min(angle_diff180(a, base), angle_diff180(a, base + 90.0));
}
inline double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }


using lane_tracking::Pose2;
using lane_tracking::metric_to_base;
using lane_tracking::base_to_metric;
using lane_tracking::base_to_global;
using lane_tracking::global_to_base;

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
  double temporal_angle{7.0}, temporal_offset{0.32}, temporal_anchor_offset{0.40};
  double temporal_gap{0.60}, ema{0.35};
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


using lane_tracking::Observation;
using lane_tracking::Primitive;




inline double percentile(std::vector<float> values, double q) {
  if (values.empty()) return 0.0;
  const size_t k = std::min(values.size() - 1,
      static_cast<size_t>(std::llround((values.size() - 1) * q)));
  std::nth_element(values.begin(), values.begin() + k, values.end());
  return values[k];
}
inline double median(std::vector<float> values) { return percentile(std::move(values), 0.5); }

inline cv::Matx33d rot_x(double d) {
  const double c = std::cos(rad(d)), s = std::sin(rad(d));
  return {1,0,0, 0,c,-s, 0,s,c};
}
inline cv::Matx33d rot_z(double d) {
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

// Detection parameters by name (E-8): the node passes declare_parameter, the offline replay
// a reader of a run's parameter dump, so both fill Params from the same table.
template <class GetDouble, class GetInt>
Params load_params(GetDouble get_double, GetInt get_int) {
  Params p;
#define D(name,field,val) p.field=get_double(name,val)
#define I(name,field,val) p.field=get_int(name,val)
    D("camera_height_m",camera_height,0.8);D("camera_forward_m",camera_forward,0);D("camera_left_m",camera_left,0);D("pitch_deg",pitch_deg,30);D("roll_deg",roll_deg,0);D("yaw_deg",yaw_deg,0);D("x_min_m",x_min,-4);D("x_max_m",x_max,4);D("y_min_m",y_min,0);D("y_max_m",y_max,5);D("meters_per_pixel",mpp,0.02);
    I("white_s_max",white_s_max,60);I("white_v_min",white_v_min,140);I("yellow_h_min",yellow_h_min,15);I("yellow_h_max",yellow_h_max,40);I("yellow_s_min",yellow_s_min,70);I("yellow_v_min",yellow_v_min,70);I("glare_v_min",glare_v_min,248);I("glare_s_max",glare_s_max,95);D("glare_min_area_ratio",glare_min_area_ratio,0.006);I("glare_dilate",glare_dilate,9);
    D("illumination_sigma_m",illumination_sigma_m,0.60);D("illumination_scale",illumination_scale,0.35);D("normalization_scale",normalization_scale,128);D("tophat_size_m",tophat_size_m,0.30);I("threshold_bands",threshold_bands,6);I("threshold_update_interval",threshold_update_interval,3);I("threshold_sample_stride",threshold_sample_stride,4);D("mad_k",mad_k,5.5);D("threshold_margin",threshold_margin,3);D("threshold_min",threshold_min,8);D("threshold_max",threshold_max,80);D("strong_response_margin",strong_response_margin,36);D("glare_recovery_margin",glare_recovery_margin,20);
    I("hough_threshold",hough_threshold,28);D("hough_min_line_length_m",hough_min_len,0.65);D("hough_max_line_gap_m",hough_max_gap,0.18);I("hough_max_raw_segments",max_raw,120);D("candidate_min_length_m",candidate_min_len,0.70);D("candidate_min_binary_support",min_binary,0.72);D("candidate_min_color_support",min_color,0.08);D("candidate_min_score",candidate_min_score,0.46);D("candidate_max_glare_ratio",max_glare,0.85);D("line_min_width_m",line_min_width,0.05);D("line_max_width_m",line_max_width,0.22);D("line_target_width_m",line_target_width,0.12);D("dominant_angle_tolerance_deg",dominant_tol,12);D("dominant_min_confidence",dominant_min_conf,0.35);D("merge_distance_m",merge_distance,0.20);D("merge_gap_m",merge_gap,0.35);D("final_min_span_m",final_min_span,0.85);D("final_min_score",final_min_score,0.52);I("max_vectors_each",max_each,8);I("vectorize_every_n",vectorize_every_n,2);
    I("temporal_history_size",history,5);I("temporal_confirm_hits",confirm_hits,3);D("temporal_angle_gate_deg",temporal_angle,7);D("temporal_offset_gate_m",temporal_offset,0.32);D("temporal_anchor_offset_m",temporal_anchor_offset,0.40);D("temporal_gap_gate_m",temporal_gap,0.60);I("temporal_max_missed_updates",max_missed,3);D("temporal_ema_alpha",ema,0.35);
    D("map_x_min_m",map_x_min,-2);D("map_x_max_m",map_x_max,20);D("map_y_min_m",map_y_min,-10);D("map_y_max_m",map_y_max,10);D("map_resolution_m",map_res,0.03);D("map_evidence_increment",map_inc,1);D("map_evidence_max",map_max,20);D("map_confirm_evidence",map_confirm,3);D("map_min_translation_m",map_min_translation,0.05);D("map_min_yaw_deg",map_min_yaw_deg,1);
#undef D
#undef I
    if(!std::isfinite(p.temporal_anchor_offset)||p.temporal_anchor_offset<=0.0)
      throw std::runtime_error("temporal_anchor_offset_m must be finite and positive");
    return p;
}


}  // namespace lane_detection
