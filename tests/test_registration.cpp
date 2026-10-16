#include "vision_lane_map_fusion/registration.hpp"
#include <cassert>
#include <iostream>
using namespace lane_fusion;
int main(){
 std::vector<RenderSegment> map={{{0,0},{5,0},.12,1,1,4,1}};
 auto r=register_lines({{{0,.10},{5,.10},.12,1}},map,{0,0});
 assert(r.accepted&&r.rank==1);assert(std::abs(r.delta.x)<1e-9);assert(std::abs(r.delta.y+.10)<1e-8);
 map.push_back({{0,0},{0,5},.12,1,2,4,1});
 r=register_lines({{{.06,.08},{5.06,.08},.12,1},{{.06,.08},{.06,5.08},.12,1}},map,{0,0});
 assert(r.accepted&&r.rank==2);assert(std::abs(r.delta.x+.06)<1e-8);
 assert(std::abs(r.delta.y+.08)<1e-8);
 r=register_lines({{{0,.3},{5,.3},.12,1}},map,{0,0});assert(!r.accepted&&r.near);
 map.push_back({{0,.10},{5,.10},.12,1,3,4,1});
 r=register_lines({{{0,.05},{5,.05},.12,1}},map,{0,0});assert(!r.accepted);
 Settings s;s.confirm_views=2;RegisteredBank bank(s);
 for(int i=1;i<=50;++i){double y=i<=2?0:.08;bank.update(i,{{{0,y},{5,y},.12,1}},{0,y});}
 assert(bank.landmarks().size()==1);assert(bank.aligned>0);
 std::cout<<"PASS: parallel null direction, crossing translation, outlier hold, ambiguous hold, repeated line\n";
}
