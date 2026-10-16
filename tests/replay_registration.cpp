#include "vision_lane_map_fusion/registration.hpp"
#include <iostream>
using namespace lane_fusion;
int main(){Settings s;s.confirm_views=2;s.min_confidence=.45;RegisteredBank b(s);LineBank raw(s);
 long long stamp;double x,y,yaw;int count;size_t frames=0;
 while(std::cin>>stamp>>x>>y>>yaw>>count){std::vector<Observation> obs;Transform p{x,y,yaw};
 for(int i=0;i<count;++i){Observation o;std::cin>>o.a.x>>o.a.y>>o.b.x>>o.b.y>>o.width>>o.confidence;o.a=p.apply(o.a);o.b=p.apply(o.b);obs.push_back(o);}
 b.update(stamp,obs,{x,y});raw.update(stamp,obs);++frames;
 }
 std::cout<<"frames="<<frames<<" aligned="<<b.aligned<<" held="<<b.held<<" raw_landmarks="<<raw.landmarks().size()<<" registered_landmarks="<<b.landmarks().size()<<" raw_segments="<<raw.segments().size()<<" registered_segments="<<b.segments().size()<<'\n';
}
