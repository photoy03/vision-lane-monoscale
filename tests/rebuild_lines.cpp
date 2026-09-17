#include "vision_lane_map_fusion/line_bank.hpp"
#include "vision_lane_map_fusion/registration.hpp"
#include <iostream>
using namespace lane_fusion;
int main(){ Settings s;s.confirm_views=2;s.min_confidence=.45;LineBank b(s);
 long long stamp;double x,y,yaw;int count;
 while(std::cin>>stamp>>x>>y>>yaw>>count){std::vector<Observation> obs;Transform p{x,y,yaw};
 for(int i=0;i<count;++i){Observation o;std::cin>>o.a.x>>o.a.y>>o.b.x>>o.b.y>>o.width>>o.confidence;o.a=p.apply(o.a);o.b=p.apply(o.b);obs.push_back(o);}b.update(stamp,obs);}
 std::cout<<"id,x1,y1,x2,y2,width,confidence,views\n";
 for(auto&s:b.segments())std::cout<<s.id<<','<<s.a.x<<','<<s.a.y<<','<<s.b.x<<','<<s.b.y<<','<<s.width<<','<<s.confidence<<','<<s.views<<'\n';
}
