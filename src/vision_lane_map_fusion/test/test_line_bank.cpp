#include <cassert>
#include <chrono>
#include <iostream>
#include <random>
#include "vision_lane_map_fusion/line_bank.hpp"
using namespace lane_fusion;
static Observation line(double y=0,double a=0,double b=2) {return {{a,y},{b,y},.12,.9};}
static constexpr int64_t sec=1000000000LL;
int main() {
  {
    LineBank bank;
    for(int i=1;i<=3;++i) bank.update(i*sec,{line((i-2)*.02)});
    assert(bank.landmarks().size()==1 && bank.segments().size()==1);
    const auto s=bank.segments()[0];
    assert(std::abs(s.a.y)<1e-10 && std::abs(s.width-.12)<1e-10 && s.views==3);
    bank.update(4*sec,{line(0,2,0)});
    assert(bank.landmarks().size()==1 && bank.segments()[0].views==4);
  }
  {
    LineBank bank;
    auto r=bank.update(sec,{line(),line(.01),line(.02)});
    assert(r.created==1 && r.duplicate==2 && bank.segments().empty());
    assert(!bank.update(sec,{line()}).accepted);
    assert(!bank.update(sec-1,{line()}).accepted);
    assert(!bank.update(0,{line()}).accepted);
    assert(bank.landmarks()[0].views==1);
  }
  {
    LineBank bank;
    const Observation vertical{{1,-1},{1,1},.12,.9};
    for(int i=1;i<=3;++i) bank.update(i*sec,{line(),line(.3),line(0,3,5),vertical});
    assert(bank.landmarks().size()==4 && bank.segments().size()==4);
  }
  {
    LineBank bank;
    bank.update(sec,{line(-.07),line(.07)});
    auto r=bank.update(2*sec,{line()});
    assert(r.ambiguous==1 && r.merged==0 && r.created==0);
  }
  {
    LineBank bank;
    for(int i=1;i<=3;++i) bank.update(i*sec,{line()});
    bank.update(4*sec,{line(0,0,4)});
    assert(bank.segments().size()==1 && bank.segments()[0].b.x<=2.000001);
    bank.update(5*sec,{line(0,0,4)});
    assert(bank.segments()[0].b.x<=2.000001);
    bank.update(6*sec,{line(0,0,4)});
    assert(bank.segments()[0].b.x>=3.99999);
  }
  {
    LineBank bank;
    auto low=line();low.confidence=.1;
    auto nan=line();nan.a.x=std::numeric_limits<double>::quiet_NaN();
    auto inf=line();inf.b.x=std::numeric_limits<double>::infinity();
    auto wide=line();wide.width=.5;
    auto high=line();high.confidence=2;
    auto r=bank.update(sec,{low,nan,inf,wide,high,line(0,0,0),line(0,0,13)});
    assert(r.invalid==7 && bank.landmarks().empty());
  }
  {
    LineBank bank;
    bank.update(sec,{line(),line(.5)});
    bank.update(2*sec,{line()});bank.update(3*sec,{line()});
    bank.update(20*sec,{});
    assert(bank.landmarks().size()==1 && bank.segments().size()==1);
  }
  {
    Settings settings;settings.max_landmarks=2;
    LineBank bank(settings);
    auto r=bank.update(sec,{line(),line(.3),line(.6)});
    assert(r.created==2 && r.capacity==1);
    settings.max_bins=5;
    LineBank tiny(settings);
    assert(tiny.update(sec,{line()}).capacity==1 && tiny.landmarks().empty());
    settings.bin_m=std::numeric_limits<double>::quiet_NaN();
    bool threw=false;
    try {LineBank bad(settings);} catch(const std::invalid_argument&) {threw=true;}
    assert(threw);
  }
  {
    LineBank bank;
    bank.update(sec,{line(0,0,4)});
    for(int i=2;i<=4;++i) bank.update(i*sec,{line(0,0,1)});
    for(int i=5;i<=7;++i) bank.update(i*sec,{line(0,3,4)});
    auto segments=bank.segments();
    assert(bank.landmarks().size()==1 && segments.size()==2);
    assert(segments[0].b.x<=1.000001 && segments[1].a.x>=2.999999);
    assert(segments[0].id==segments[1].id);
  }
  {
    LineBank bank;
    bank.update(sec,{line()});
    double angle=6.*3.14159265358979323846/180.;
    Observation angled{{0,0},{2*std::cos(angle),2*std::sin(angle)},.12,.9};
    assert(bank.update(2*sec,{angled}).created==1);
  }
  {
    Settings settings;settings.min_confidence=0;
    LineBank bank(settings);auto o=line();o.confidence=0;
    assert(bank.update(sec,{o}).invalid==1);
  }
  {
    // Small angular noise, endpoint reversal and translation along the same
    // line must leave one landmark. Test uses synthetic data, not real accuracy.
    LineBank bank;std::mt19937 rng(42);
    std::uniform_real_distribution<double> noise(-.015,.015);
    auto start=std::chrono::steady_clock::now();
    for(int i=1;i<=3000;++i) {
      Observation o{{noise(rng),noise(rng)},{2+noise(rng),noise(rng)},.12,.9};
      if(i%2) std::swap(o.a,o.b);
      bank.update(i*sec,{o});
    }
    assert(bank.landmarks().size()==1 && bank.segments().size()==1);
    assert(bank.landmarks()[0].views==3000 && bank.landmarks()[0].support.size()<=22);
    for(auto b:bank.landmarks()[0].support) assert(b.second<=20);
    std::cout<<"synthetic 3000 updates ms="<<std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-start).count()<<'\n';
  }
  std::cout<<"PASS: merge/reversal, one-vote, timestamps, parallel/crossing/gap, ambiguity, "
    "tail/gap support, invalid inputs, angle gate, TTL, capacity/settings, seeded stress\n";
}
