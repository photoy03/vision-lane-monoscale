#pragma once

// Lane primitives and the world-frame temporal lane tracker, shared by
// lane_detection (legacy mapping owner) and lane_map (mapping owner). ROS-free.
// The code below was moved verbatim from lane_detection/global_mapping_node.cpp.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace lane_tracking {
namespace detail {
constexpr double kPi = 3.14159265358979323846;
inline double deg(double r) { return r * 180.0 / kPi; }
inline double angle180(double a) {
  a = std::fmod(a, 180.0);
  return a < 0.0 ? a + 180.0 : a;
}
inline double angle_diff180(double a, double b) {
  const double d = std::abs(angle180(a) - angle180(b));
  return std::min(d, 180.0 - d);
}
}  // namespace detail

struct Pose2 {
  double x{0.0}, y{0.0}, yaw{0.0};
};

inline cv::Point2f metric_to_base(const cv::Point2f &p) {
  // BEV: x=right, y=forward. base_link: x=forward, y=left.
  return {p.y, -p.x};
}
inline cv::Point2f base_to_metric(const cv::Point2f &p) { return {-p.y, p.x}; }
inline cv::Point2f base_to_global(const cv::Point2f &p, const Pose2 &pose) {
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  return {static_cast<float>(pose.x + c * p.x - s * p.y),
          static_cast<float>(pose.y + s * p.x + c * p.y)};
}
inline cv::Point2f global_to_base(const cv::Point2f &p, const Pose2 &pose) {
  const double dx = p.x - pose.x, dy = p.y - pose.y;
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  return {static_cast<float>(c * dx + s * dy),
          static_cast<float>(-s * dx + c * dy)};
}

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
  cv::Point2f anchor_g0, anchor_g1;
  std::string family;
  double width{0}, confidence{0};
  std::deque<int> hits;
  int hits_total{1}, missed{0};
  bool confirmed{false}, anchor_valid{false};
};

// The tracker's own settings (a subset of lane_detection's Params).
struct TrackerParams {
  int history{5}, confirm_hits{3}, max_missed{3};
  double temporal_angle{7.0}, temporal_offset{0.32}, temporal_anchor_offset{0.40};
  double temporal_gap{0.60}, ema{0.35};
};

class Tracker {
 public:
  explicit Tracker(const TrackerParams&p):p_(p){}
  void reset(){tracks_.clear();next_id_=1;}
  void update(const std::vector<Observation>&obs,const Pose2&pose){
    for(auto&x:tracks_){x.hits.push_back(0);while(static_cast<int>(x.hits.size())>p_.history)x.hits.pop_front();++x.missed;}
    struct Pair{double cost;size_t ti,oi;};std::vector<Pair>pairs;
    std::vector<std::pair<cv::Point2f,cv::Point2f>> global;
    for(const auto&o:obs)global.push_back({base_to_global(metric_to_base(o.p0_m),pose),base_to_global(metric_to_base(o.p1_m),pose)});
    for(size_t ti=0;ti<tracks_.size();++ti)for(size_t oi=0;oi<obs.size();++oi){auto cost=association(tracks_[ti],global[oi]);if(cost)pairs.push_back({*cost,ti,oi});}
    std::sort(pairs.begin(),pairs.end(),[](auto&a,auto&b){return a.cost<b.cost;});std::vector<bool>ut(tracks_.size()),uo(obs.size());
    for(const auto&m:pairs){if(ut[m.ti]||uo[m.oi])continue;auto&t=tracks_[m.ti];const bool was_confirmed=t.confirmed;auto g=global[m.oi];if((t.g1-t.g0).dot(g.second-g.first)<0)std::swap(g.first,g.second);const float alpha=static_cast<float>(p_.ema);t.g0=(1.0f-alpha)*t.g0+alpha*g.first;t.g1=(1.0f-alpha)*t.g1+alpha*g.second;t.family=obs[m.oi].family;t.width=obs[m.oi].width;t.confidence=obs[m.oi].confidence;t.hits.back()=1;++t.hits_total;t.missed=0;if(std::accumulate(t.hits.begin(),t.hits.end(),0)>=p_.confirm_hits)t.confirmed=true;if(!was_confirmed&&t.confirmed){t.anchor_g0=t.g0;t.anchor_g1=t.g1;t.anchor_valid=true;}ut[m.ti]=uo[m.oi]=true;}
    for(size_t i=0;i<obs.size();++i)if(!uo[i]){Primitive t;t.id=next_id_++;t.g0=global[i].first;t.g1=global[i].second;t.family=obs[i].family;t.width=obs[i].width;t.confidence=obs[i].confidence;t.hits.push_back(1);t.confirmed=p_.confirm_hits<=1;if(t.confirmed){t.anchor_g0=t.g0;t.anchor_g1=t.g1;t.anchor_valid=true;}tracks_.push_back(t);}
    tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](auto&t){return t.missed>p_.max_missed;}),tracks_.end());
  }
  std::vector<Observation> project(const Pose2&pose,bool confirmed_only=false,bool force_reprojected=false)const{std::vector<Observation>out;for(const auto&t:tracks_){if(confirmed_only&&!t.confirmed)continue;cv::Point2f b0=global_to_base(t.g0,pose),b1=global_to_base(t.g1,pose);Observation o;o.p0_m=base_to_metric(b0);o.p1_m=base_to_metric(b1);o.family=t.family;o.span=cv::norm(o.p1_m-o.p0_m);o.width=t.width;o.confidence=t.confidence;o.track_id=t.id;o.hits_total=t.hits_total;o.hits_window=std::accumulate(t.hits.begin(),t.hits.end(),0);o.confirmed=t.confirmed;o.reprojected=force_reprojected||t.missed>0;out.push_back(o);}return out;}
  const std::vector<Primitive>&primitives()const{return tracks_;}
 private:
  std::optional<double>association(const Primitive&t,const std::pair<cv::Point2f,cv::Point2f>&g)const{cv::Point2f od=t.g1-t.g0,nd=g.second-g.first;double oa=detail::angle180(detail::deg(std::atan2(od.y,od.x))),na=detail::angle180(detail::deg(std::atan2(nd.y,nd.x)));double ae=detail::angle_diff180(oa,na);if(ae>p_.temporal_angle)return{};double n=cv::norm(od);if(n<1e-9)return{};od*=1.0f/n;cv::Point2f normal(-od.y,od.x);double offset=std::abs((0.5f*(g.first+g.second)-0.5f*(t.g0+t.g1)).dot(normal));if(offset>p_.temporal_offset)return{};if(t.anchor_valid){cv::Point2f ad=t.anchor_g1-t.anchor_g0;double al=cv::norm(ad);if(al<1e-9)return{};ad*=1.0f/al;double anchor_angle=detail::angle_diff180(detail::angle180(detail::deg(std::atan2(ad.y,ad.x))),na);if(anchor_angle>p_.temporal_angle)return{};cv::Point2f anormal(-ad.y,ad.x);double anchor_offset=std::abs((0.5f*(g.first+g.second)-0.5f*(t.anchor_g0+t.anchor_g1)).dot(anormal));if(anchor_offset>p_.temporal_anchor_offset)return{};}double o0=t.g0.dot(od),o1=t.g1.dot(od),n0=g.first.dot(od),n1=g.second.dot(od);double gap=std::max({std::min(o0,o1)-std::max(n0,n1),std::min(n0,n1)-std::max(o0,o1),0.0});if(gap>p_.temporal_gap)return{};return ae/p_.temporal_angle+offset/p_.temporal_offset+0.25*gap/p_.temporal_gap;}
  TrackerParams p_;uint32_t next_id_{1};std::vector<Primitive>tracks_;
};

}  // namespace lane_tracking
