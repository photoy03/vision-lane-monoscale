#include <cassert>
#include "lane_map/pending_lane_observations.hpp"
#include "lane_map/pending_readiness.hpp"

int main() {
  using namespace vision_lane;
  using S=PendingService; using K=PoseMatchKind;
  const int64_t second=1000000000;
  PendingReadinessIndex index;
  index.set(1,false); index.set(2,true);
  assert(!index.ready(1) && index.any_other_ready(1));
  assert(!index.any_other_ready(2));
  assert(pending_service(false,K::gap_too_large,false,4*second,2*second,4*second,true)==S::blocked_head_drop);
  assert(pending_service(false,K::gap_too_large,false,4*second-1,2*second,4*second,true)==S::wait);
  assert(pending_service(false,K::gap_too_large,false,5*second,2*second,4*second,false)==S::wait);
  // Missing right bracket must wait, regardless of later ready metadata.
  for (K kind : {K::buffer_empty,K::stamp_after_latest,K::no_bracket,K::predicted,K::nearest})
    assert(pending_service(false,kind,false,10*second,2*second,4*second,true)==S::wait);
  assert(pending_service(false,K::stamp_before_oldest,false,2*second,2*second,4*second,false)==S::evicted_drop);
  assert(pending_service(false,K::gap_too_large,true,12*second,2*second,4*second,true)==S::ttl_drop);
  // A freshly valid measured pose is served even at the TTL boundary.
  assert(pending_service(true,K::exact,true,12*second,2*second,4*second,true)==S::commit);
  assert(pending_service(true,K::interpolated,false,0,2*second,4*second,false)==S::commit);
  // Only a complete-but-unusable bracket is terminal; a missing right bracket
  // (stamp_after_latest) or an empty buffer may still resolve.
  assert(head_is_terminally_unresolvable(K::gap_too_large));
  assert(head_is_terminally_unresolvable(K::bracket_too_far));
  assert(!head_is_terminally_unresolvable(K::stamp_after_latest));
  assert(!head_is_terminally_unresolvable(K::buffer_empty));
  assert(!head_is_terminally_unresolvable(K::stamp_before_oldest));
  assert(!head_is_terminally_unresolvable(K::interpolated));
  index.erase(2); assert(!index.any_other_ready(1)); index.clear(); assert(index.size()==0);
  PendingLaneObservationQueue<int> queue(3);
  queue.push({1,"front",200,10,2}); queue.push({1,"front",100,10,1});
  assert(queue.entries().front().stamp_ns==100);
  // Readiness never changes chronological ordering, and a skipped ID cannot
  // re-enter the same epoch when its pose arrives later.
  index.set(2,true); index.set(1,false);
  assert(queue.front()->payload==1);
  auto retired=queue.pop_front(); assert(retired->payload==1);
  assert(queue.push(*retired).result==PendingInsertResult::duplicate);
  assert(queue.front()->payload==2);
  queue.clear(); index.clear();
  assert(queue.empty() && index.size()==0);
}
