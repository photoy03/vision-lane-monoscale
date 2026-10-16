#include <cassert>
#include <chrono>
#include <iostream>
#include <random>
#include "lane_map/line_bank.hpp"
#include "lane_map/registration.hpp"
using namespace lane_fusion;
static Observation line(double y=0,double a=0,double b=2) {return {{a,y},{b,y},.12,.9};}
static constexpr int64_t sec=1000000000LL;
int main() {
  {
    // Shared occupancy/lane mode must preserve the odometry map frame exactly.
    Settings settings;settings.pose_registration=false;settings.confirm_views=1;
    RegisteredBank bank(settings);
    auto result=bank.update(sec,{line(0,10,12)},{10,0});
    assert(result.created==1 && bank.aligned==0 && bank.held==0);
    const auto corrected=bank.corrected({3,4});
    assert(std::abs(corrected.x-3)<1e-12 && std::abs(corrected.y-4)<1e-12);
    assert(std::abs(bank.yaw_correction())<1e-12);
  }
  {
    Settings settings;settings.confirm_views=1;settings.pose_registration=false;
    RegisteredBank bank(settings);
    assert(bank.update(sec,{line()},{0,0}).created==1);
    bank.reset();
    assert(bank.landmarks().empty() && bank.segments().empty());
    assert(bank.update(sec,{line()},{0,0}).created==1);
  }
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
    Settings settings;
    settings.confirm_views=1;
    settings.offset_m=.10;
    settings.duplicate_render_offset_m=.20;
    settings.min_length_m=.10;
    LineBank bank(settings);
    bank.update(sec,{line(0,0,10)});
    bank.update(2*sec,{line(.15,7.75,10.5)});
    const auto rendered=bank.render();
    assert(rendered.suppressed_duplicates==1);
    assert(rendered.segments.size()==2);
    const auto& tail=rendered.segments[1];
    assert(tail.a.x>=9.99 && tail.b.x>=10.40);
    assert(tail.id!=rendered.segments[0].id);
  }
  {
    // Close parallel lines observed together are protected as distinct paint.
    Settings settings;
    settings.confirm_views=1;
    settings.offset_m=.10;
    settings.duplicate_render_offset_m=.20;
    LineBank bank(settings);
    bank.update(sec,{line(0,0,6),line(.15,3.5,6.5)});
    const auto rendered=bank.render();
    assert(bank.landmarks().size()==2);
    assert(rendered.segments.size()==2);
    assert(rendered.suppressed_duplicates==0);
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
    // Confirmed clutter is not immortal: it survives the short tentative TTL
    // but disappears after the separately configured confirmed horizon.
    Settings settings;settings.confirm_views=2;settings.tentative_ttl_sec=2;settings.confirmed_ttl_sec=5;
    LineBank bank(settings);
    bank.update(sec,{line()});bank.update(2*sec,{line()});
    assert(bank.segments().size()==1);
    bank.update(8*sec,{});
    assert(bank.landmarks().empty());
  }
  {
    // Occupancy must persist across distinct map updates. A single hit and a
    // hit followed by clear evidence do not delete paint; three hits do.
    Settings settings;settings.confirm_views=1;
    LineBank bank(settings);bank.update(sec,{line(0,0,1)});
    auto occupied=[](Point p){return p.x>=.4&&p.x<=.6;};
    assert(bank.prune_occupied(occupied,3).support_bins==0);
    assert(bank.prune_occupied([](Point){return false;},3).support_bins==0);
    assert(bank.prune_occupied(occupied,3).support_bins==0);
    assert(bank.prune_occupied(occupied,3).support_bins==0);
    const auto removed=bank.prune_occupied(occupied,3);
    assert(removed.support_bins>=1);
    for(const auto& segment:bank.segments())
      assert(segment.b.x<=.400001||segment.a.x>=.599999);
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
    // The rolling range removes old lines and trims a line where it leaves
    // the vehicle-centered circle instead of merely hiding it at render time.
    LineBank bank;
    for(int i=1;i<=3;++i) bank.update(i*sec,{line(0,8,10)});
    const auto removed=bank.prune({0,0},2.0);
    assert(removed.landmarks==1 && bank.landmarks().empty());
    for(int i=4;i<=6;++i) bank.update(i*sec,{line(0,0,4)});
    const auto trimmed=bank.prune({0,0},2.0);
    assert(trimmed.support_bins>0 && bank.landmarks().size()==1);
    for(const auto& segment:bank.segments()) {
      assert(std::hypot(segment.a.x,segment.a.y)<=2.100001);
      assert(std::hypot(segment.b.x,segment.b.y)<=2.100001);
    }
  }
  {
    Settings settings;settings.confirm_views=1;
    LineBank bank(settings);
    bank.update(sec,{line(0,-2,2),line(10,-2,2)});
    const auto removed=bank.prune_box(-3,3,-3,3);
    assert(removed.landmarks==1);
    assert(bank.segments().size()==1);
    assert(bank.segments()[0].a.y>-3&&bank.segments()[0].b.y<3);
  }
  {
    LineBank bank;
    bank.update(sec,{line()});
    double angle=6.*3.14159265358979323846/180.;
    Observation angled{{0,0},{2*std::cos(angle),2*std::sin(angle)},.12,.9};
    assert(bank.update(2*sec,{angled}).created==1);
  }
  {
    // Legacy mode deliberately keeps the old overlap-only association.
    LineBank legacy;
    for(int i=1;i<=3;++i) legacy.update(i*sec,{line(0,0,2)});
    for(int i=4;i<=6;++i) legacy.update(i*sec,{line(0,2.2,4.2)});
    assert(legacy.landmarks().size()==2 && legacy.segments().size()==2);

    // Continuation mode associates the collinear endpoint gap with the same
    // landmark and emits one joined interval after the new tail is confirmed.
    Settings settings;settings.continuation_mode=true;settings.continuation_gap_m=.30;
    LineBank continuation(settings);
    for(int i=1;i<=3;++i) continuation.update(i*sec,{line(0,0,2)});
    int continued=0;
    for(int i=4;i<=6;++i)
      continued+=continuation.update(i*sec,{line(0,2.2,4.2)}).continued;
    assert(continued>=1);
    assert(continuation.landmarks().size()==1 && continuation.segments().size()==1);
    assert(continuation.segments()[0].a.x<=.000001);
    assert(continuation.segments()[0].b.x>=4.199999);
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
  {
    // Co-observed nearby traces are ambiguous: keep both instead of deleting
    // a potentially real parallel marking based on distance alone.
    Settings settings;settings.confirm_views=2;
    LineBank bank(settings);
    for(int i=1;i<=3;++i) bank.update(i*sec,{line(0),line(.12),line(.45)});
    assert(bank.landmarks().size()==3);
    assert(bank.segments().size()==3);
  }
  {
    // A second co-observed close pair is likewise preserved conservatively.
    Settings settings;settings.confirm_views=2;settings.offset_m=.08;
    LineBank bank(settings);
    for(int i=1;i<=3;++i) bank.update(i*sec,{line(0),line(.18),line(.45)});
    assert(bank.landmarks().size()==3);
    assert(bank.segments().size()==3);
  }
  {
    // A close line first seen at a different stamp can be clipped at render
    // time; a 2.6 m parking-spacing parallel line remains visible.
    Settings settings;settings.confirm_views=1;settings.offset_m=.08;
    LineBank conservative(settings);
    conservative.update(sec,{line(0,-1,3),line(2.6,-1,3)});
    conservative.update(2*sec,{line(.26,-1,3)});
    const auto baseline=conservative.render();
    assert(conservative.landmarks().size()==3);
    assert(baseline.segments.size()==3&&baseline.suppressed_duplicates==0);
    assert(baseline.near_duplicate_candidates==1);

    settings.duplicate_render_offset_m=.30;
    LineBank experimental(settings);
    experimental.update(sec,{line(0,-1,3),line(2.6,-1,3)});
    experimental.update(2*sec,{line(.26,-1,3)});
    const auto trial=experimental.render();
    assert(experimental.landmarks().size()==3);
    assert(trial.segments.size()==2&&trial.suppressed_duplicates==1);
    assert(trial.near_duplicate_candidates==0);
  }
  std::cout<<"PASS: merge/reversal, one-vote, timestamps, parallel/crossing/gap, ambiguity, "
    "tail/gap support, invalid inputs, angle gate, TTL, capacity/settings, seeded stress\n";
}
