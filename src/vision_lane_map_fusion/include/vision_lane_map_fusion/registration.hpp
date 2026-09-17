#pragma once
#include "line_bank.hpp"
namespace lane_fusion {
struct Transform {
  double x=0,y=0,yaw=0;
  Point apply(Point p) const {double c=cos(yaw),s=sin(yaw);return {x+c*p.x-s*p.y,y+s*p.x+c*p.y};}
};
struct RegistrationResult {bool accepted=false,near=false; int pairs=0,rank=0; double rms=0; Transform delta;};
// Bounded local line registration. Only normals constrain translation: no
// endpoint residuals, so a parallel scene cannot invent longitudinal motion.
inline RegistrationResult register_lines(const std::vector<Observation>& observations,
 const std::vector<RenderSegment>& map, Point pivot) {
  RegistrationResult out;
  struct Pair {Observation o; RenderSegment m;};
  std::vector<Pair> pairs;
  for(const auto&o:observations) {
    double len=norm(o.b-o.a);if(len<.65||!finite(o.a)||!finite(o.b)) continue;
    Point u=(o.b-o.a)*(1/len); double best=1e9,second=1e9;const RenderSegment* target=nullptr;
    for(const auto&m:map) {
      Point v=(m.b-m.a)*(1/norm(m.b-m.a));
      double angle=acos(std::clamp(std::abs(dot(u,v)),0.,1.));if(angle>6.*M_PI/180.)continue;
      double lo=dot(o.a-m.a,v),hi=dot(o.b-m.a,v);if(lo>hi)std::swap(lo,hi);
      if(std::min(hi,norm(m.b-m.a))-std::max(lo,0.)<.25)continue;
      double d=std::abs(dot(normal(v),(o.a+o.b)*.5-m.a));
      if(d<.35)out.near=true;
      if(d>.18)continue;
      double cost=d/.18+angle/(6*M_PI/180.);
      if(cost<best){second=best;best=cost;target=&m;}else second=std::min(second,cost);
    }
    if(target&&second-best>.20)pairs.push_back({o,*target});
  }
  out.pairs=pairs.size();if(pairs.empty())return out;
  double angle=0,weight=0;
  for(auto&p:pairs){Point a=p.o.b-p.o.a,b=p.m.b-p.m.a;
    double d=std::remainder(atan2(b.y,b.x)-atan2(a.y,a.x),M_PI);
    double w=std::min(2.,norm(a))*p.o.confidence;angle+=w*d;weight+=w;}
  if(weight<=0)return out;
  angle/=weight;
  if(std::abs(angle)>2*M_PI/180.)return out;
  double A=0,B=0,D=0,gx=0,gy=0;
  Transform rotation{pivot.x-cos(angle)*pivot.x+sin(angle)*pivot.y,
    pivot.y-sin(angle)*pivot.x-cos(angle)*pivot.y,angle};
  for(auto&p:pairs){Point n=normal((p.m.b-p.m.a)*(1/norm(p.m.b-p.m.a)));
    Point c=rotation.apply((p.o.a+p.o.b)*.5);
    double r=dot(n,c-p.m.a),w=p.o.confidence*std::min(1.,.06/std::max(1e-9,std::abs(r)));
    A+=w*n.x*n.x;B+=w*n.x*n.y;D+=w*n.y*n.y;gx-=w*n.x*r;gy-=w*n.y*r;}
  double phi=.5*atan2(2*B,A-D),c=cos(phi),s=sin(phi);
  double l1=c*c*A+2*c*s*B+s*s*D,l2=s*s*A-2*c*s*B+c*c*D;
  Point t{};
  if(l1>1e-8){double v=(c*gx+s*gy)/l1;t=t+Point{c,s}*v;++out.rank;}
  if(l2>std::max(1e-8,.08*l1)){double v=(-s*gx+c*gy)/l2;t=t+Point{-s,c}*v;++out.rank;}
  if(norm(t)>.12)return out;
  rotation.x+=t.x;rotation.y+=t.y;
  double squared=0;int n=0;
  for(auto&p:pairs){Point v=normal((p.m.b-p.m.a)*(1/norm(p.m.b-p.m.a)));
    for(Point q:{p.o.a,p.o.b}){double r=dot(v,rotation.apply(q)-p.m.a);squared+=r*r;++n;}}
  out.rms=sqrt(squared/std::max(1,n));
  if(out.rms>.06)return out;
  out.accepted=true;out.delta=rotation;return out;
}
class RegisteredBank {
 public:
  explicit RegisteredBank(Settings s={}):bank_(s){}
  const Settings& settings()const{return bank_.settings();}
  const std::vector<Landmark>& landmarks()const{return bank_.landmarks();}
  std::vector<RenderSegment> segments()const{return bank_.segments();}
  Point corrected(Point p)const{return correction_.apply(p);}
  double yaw_correction()const{return correction_.yaw;}
  RegistrationResult last; size_t held=0,aligned=0;
  Result update(int64_t stamp,std::vector<Observation> obs,Point raw_pivot) {
    if(stamp<=last_stamp_)return {};
    last_stamp_=stamp;
    const auto& cfg=bank_.settings();
    obs.erase(std::remove_if(obs.begin(),obs.end(),[&](const Observation&o){
      double length=norm(o.b-o.a);
      return !finite(o.a)||!finite(o.b)||!std::isfinite(o.confidence)||
        o.confidence<cfg.min_confidence||o.confidence>1||!std::isfinite(o.width)||
        o.width<cfg.min_width_m||o.width>cfg.max_width_m||
        length<cfg.min_length_m||length>cfg.max_length_m;
    }),obs.end());
    if(obs.empty())return {};
    for(auto&o:obs){o.a=corrected(o.a);o.b=corrected(o.b);}
    // Confirmed evidence only. Bootstrap until enough repeated views exist.
    last=register_lines(obs,bank_.segments(),corrected(raw_pivot));
    if(last.accepted){
      auto p=last.delta.apply({correction_.x,correction_.y});
      correction_={p.x,p.y,std::remainder(correction_.yaw+last.delta.yaw,2*M_PI)};
      for(auto&o:obs){o.a=last.delta.apply(o.a);o.b=last.delta.apply(o.b);}++aligned;
    }else if(last.near){++held;return {};}
    auto result=bank_.update(stamp,obs);
    return result;
  }
 private:
  LineBank bank_;Transform correction_;int64_t last_stamp_=0;
};
}
