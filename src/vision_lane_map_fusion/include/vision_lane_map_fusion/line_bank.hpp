#pragma once
// Independent of ROS/OpenCV. This fuses line observations in an EXISTING map
// frame. It never estimates or changes a vehicle pose / TF.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lane_fusion {
struct Point { double x{0}, y{0}; };
inline Point operator+(Point a, Point b) {return {a.x+b.x, a.y+b.y};}
inline Point operator-(Point a, Point b) {return {a.x-b.x, a.y-b.y};}
inline Point operator*(Point a, double s) {return {a.x*s, a.y*s};}
inline double dot(Point a, Point b) {return a.x*b.x+a.y*b.y;}
inline double norm(Point a) {return std::hypot(a.x,a.y);}
inline Point normal(Point u) {return {-u.y,u.x};}
inline bool finite(Point p) {return std::isfinite(p.x)&&std::isfinite(p.y);}
struct Observation {Point a,b; double width{.12}, confidence{1.};};
struct Settings {
  double angle_deg{4.}, offset_m{.08}, overlap_m{.20}, overlap_ratio{.25};
  double bin_m{.10}, min_length_m{.65}, max_length_m{12.}, min_confidence{.55};
  double min_width_m{.05}, max_width_m{.22}, max_weight{30.}, ambiguity_margin{.15};
  double tentative_ttl_sec{15.};
  int confirm_views{3}, max_bin_votes{20}, max_landmarks{1000}, max_bins{600};
};
struct Landmark {
  uint32_t id{0}, views{0};
  Point reference, reference_u, center, u;
  double width{.12}, confidence{1}, weight{0};
  int64_t last_stamp_ns{0};
  std::map<int,int> support; // Fixed reference-axis bins; one vote per keyframe.
};
struct RenderSegment {Point a,b; double width,confidence; uint32_t id,views; int64_t stamp_ns;};
struct Result {int merged{0}, created{0}, duplicate{0}, ambiguous{0}, invalid{0}, capacity{0}; bool accepted{false};};

class LineBank {
 public:
  explicit LineBank(Settings s = {}):s_(s) {
    const double numbers[] = {s.angle_deg,s.offset_m,s.overlap_m,s.overlap_ratio,s.bin_m,
      s.min_length_m,s.max_length_m,s.min_confidence,s.min_width_m,s.max_width_m,
      s.max_weight,s.ambiguity_margin,s.tentative_ttl_sec};
    for (double v:numbers) if(!std::isfinite(v)) throw std::invalid_argument("nonfinite line settings");
    if(s.angle_deg<=0||s.angle_deg>=20||s.offset_m<=0||s.overlap_m<=0||
       s.overlap_ratio<=0||s.overlap_ratio>1||s.bin_m<=0||s.min_length_m<=0||
       s.max_length_m<s.min_length_m||s.min_confidence<0||s.min_confidence>1||
       s.min_width_m<=0||s.max_width_m<s.min_width_m||s.max_weight<1||s.ambiguity_margin<0||
       s.confirm_views<1||s.max_bin_votes<s.confirm_views||s.max_landmarks<1||
       s.max_bins<2||s.max_bins>10000||s.tentative_ttl_sec<=0)
      throw std::invalid_argument("invalid line fusion settings");
  }
  const Settings& settings() const {return s_;}
  const std::vector<Landmark>& landmarks() const {return lines_;}
  int64_t last_stamp_ns() const {return last_stamp_ns_;}

  Result update(int64_t stamp_ns, std::vector<Observation> observations) {
    Result r;
    if(stamp_ns<=0||stamp_ns<=last_stamp_ns_) return r;
    last_stamp_ns_=stamp_ns; r.accepted=true;
    // Not seeing a confirmed line is not negative evidence (occlusion exists).
    lines_.erase(std::remove_if(lines_.begin(),lines_.end(),[&](const Landmark& l){
      return !confirmed(l) && static_cast<double>(stamp_ns-l.last_stamp_ns)*1e-9>s_.tentative_ttl_sec;
    }),lines_.end());
    observations.erase(std::remove_if(observations.begin(),observations.end(),[&](const Observation& o){
      if(valid(o)) return false;
      ++r.invalid; return true;
    }),observations.end());
    // Longest/highest quality gets the vote if this frame has duplicate Hough lines.
    std::stable_sort(observations.begin(),observations.end(),[](const Observation& a,const Observation& b){
      return norm(a.b-a.a)*a.confidence>norm(b.b-b.a)*b.confidence;
    });
    std::vector<bool> used(lines_.size(),false);
    for(const auto& o:observations) {
      double best=std::numeric_limits<double>::infinity(), second=best;
      size_t at=lines_.size();
      for(size_t i=0;i<lines_.size();++i) {
        const double c=cost(lines_[i],o);
        if(c<best) {second=best;best=c;at=i;} else if(c<second) second=c;
      }
      if(at<lines_.size()) {
        if(second-best<s_.ambiguity_margin) {++r.ambiguous;continue;}
        if(used[at]) {++r.duplicate;continue;}
        if(!add(lines_[at],o,stamp_ns)) {++r.capacity;continue;}
        used[at]=true;++r.merged;
      } else {
        if(lines_.size()>=static_cast<size_t>(s_.max_landmarks)) {++r.capacity;continue;}
        Landmark l;
        l.id=next_id_++;l.reference=o.a+(o.b-o.a)*.5;
        l.reference_u=(o.b-o.a)*(1./norm(o.b-o.a));l.center=l.reference;l.u=l.reference_u;
        if(!add(l,o,stamp_ns)) {++r.capacity;continue;}
        lines_.push_back(std::move(l));used.push_back(true);++r.created;
      }
    }
    return r;
  }

  std::vector<RenderSegment> segments() const {
    std::vector<RenderSegment> out;
    for(const auto& l:lines_) {
      bool active=false;int first=0,last=0;
      const auto emit=[&] {
        const double t0=first*s_.bin_m, t1=(last+1)*s_.bin_m;
        if(t1-t0+1e-9>=s_.min_length_m)
          out.push_back({point_at(l,t0),point_at(l,t1),l.width,l.confidence,l.id,l.views,l.last_stamp_ns});
      };
      for(const auto& b:l.support) {
        if(b.second<s_.confirm_views) {if(active) emit(); active=false;continue;}
        if(active&&b.first!=last+1) {emit();active=false;}
        if(!active) {first=b.first;active=true;}
        last=b.first;
      }
      if(active) emit();
    }
    return out;
  }

 private:
  bool valid(const Observation& o) const {
    const double length=norm(o.b-o.a);
    return finite(o.a)&&finite(o.b)&&std::isfinite(length)&&length>=s_.min_length_m&&
      length<=s_.max_length_m&&std::isfinite(o.confidence)&&o.confidence>0&&o.confidence>=s_.min_confidence&&o.confidence<=1&&
      std::isfinite(o.width)&&o.width>=s_.min_width_m&&o.width<=s_.max_width_m;
  }
  bool confirmed(const Landmark& l) const {
    return std::any_of(l.support.begin(),l.support.end(),[&](const auto& b){return b.second>=s_.confirm_views;});
  }
  Point point_at(const Landmark& l,double t) const {
    const Point p=l.reference+l.reference_u*t, n=normal(l.u), n0=normal(l.reference_u);
    return p+n0*(dot(n,l.center-p)/dot(n,n0));
  }
  double cost(const Landmark& l,const Observation& o) const {
    constexpr double inf=std::numeric_limits<double>::infinity();
    Point v=(o.b-o.a)*(1./norm(o.b-o.a));
    const double angle=std::acos(std::clamp(std::abs(dot(l.u,v)),0.,1.))*180./3.14159265358979323846;
    if(angle>s_.angle_deg||l.support.empty()) return inf;
    // A fixed reference-angle gate prevents a landmark gradually turning into a different road line.
    const double anchor_angle=std::acos(std::clamp(std::abs(dot(l.reference_u,v)),0.,1.))*180./3.14159265358979323846;
    if(anchor_angle>s_.angle_deg) return inf;
    double a=dot(o.a-l.reference,l.reference_u),b=dot(o.b-l.reference,l.reference_u);
    if(a>b) std::swap(a,b);
    const double lo=std::max(a,l.support.begin()->first*s_.bin_m);
    const double hi=std::min(b,(l.support.rbegin()->first+1)*s_.bin_m);
    const double shorter=std::min(b-a,(l.support.rbegin()->first+1-l.support.begin()->first)*s_.bin_m);
    if(hi-lo<s_.overlap_m||hi-lo<s_.overlap_ratio*shorter) return inf;
    const Point n=normal(v);
    const double distance=std::max(std::abs(dot(n,point_at(l,lo)-o.a)),std::abs(dot(n,point_at(l,hi)-o.a)));
    if(distance>s_.offset_m) return inf;
    return distance/s_.offset_m+angle/s_.angle_deg;
  }
  bool add(Landmark& l,const Observation& o,int64_t stamp_ns) {
    double a=dot(o.a-l.reference,l.reference_u),b=dot(o.b-l.reference,l.reference_u);
    if(a>b) std::swap(a,b);
    // Only fully covered bins get votes: a single long outlier cannot extend a confirmed tail.
    if(std::max(std::abs(a),std::abs(b))/s_.bin_m>1e7) return false;
    const int first=static_cast<int>(std::ceil(a/s_.bin_m-1e-8));
    const int last=static_cast<int>(std::floor(b/s_.bin_m+1e-8))-1;
    if(last<first||last-first+1>s_.max_bins) return false;
    int missing=0;
    for(int i=first;i<=last;++i) if(!l.support.count(i)) ++missing;
    if(l.support.size()+missing>static_cast<size_t>(s_.max_bins)) return false;
    Point v=(o.b-o.a)*(1./norm(o.b-o.a));
    if(dot(v,l.reference_u)<0) v=v*-1;
    const double weight=o.confidence*std::min(2.,norm(o.b-o.a));
    const double alpha=weight/(l.weight+weight);
    Point u=l.u*(1-alpha)+v*alpha;
    l.u=u*(1./norm(u));l.center=l.center*(1-alpha)+(o.a+o.b)*(.5*alpha);
    l.width=l.width*(1-alpha)+o.width*alpha;
    l.confidence=l.confidence*(1-alpha)+o.confidence*alpha;
    l.weight=std::min(s_.max_weight,l.weight+weight);
    if(l.views<std::numeric_limits<uint32_t>::max()) ++l.views;
    l.last_stamp_ns=stamp_ns;
    for(int i=first;i<=last;++i) l.support[i]=std::min(s_.max_bin_votes,l.support[i]+1);
    return true;
  }
  Settings s_;std::vector<Landmark> lines_;uint32_t next_id_{1};int64_t last_stamp_ns_{0};
};
}  // namespace lane_fusion
