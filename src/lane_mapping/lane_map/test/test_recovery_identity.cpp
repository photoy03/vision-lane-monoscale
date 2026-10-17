#include <cassert>
#include "lane_map/recovery_identity.hpp"

using namespace lane_fusion;

int main() {
  RecoveryIdentityLedger ledger(2);
  const RecoveryIdentity id1{"run", 1, 1};
  const RecoveryIdentity id2{"run", 1, 2};
  const RecoverySourceKey src1{"run", 1, "front", 100};
  const RecoverySourceKey src2{"run", 1, "rear", 100};
  const RecoverySourceKey src3{"run", 1, "front", 200};

  assert(ledger.observe(id1, src1) == RecoveryIdentityDecision::kNew);
  assert(ledger.observe(id1, src1) == RecoveryIdentityDecision::kDuplicateId);
  assert(ledger.observe(id1, src2) == RecoveryIdentityDecision::kIdConflict);
  assert(ledger.observe(id2, src1) == RecoveryIdentityDecision::kDuplicateSource);
  assert(ledger.observe(id2, src2) == RecoveryIdentityDecision::kNew);
  assert(ledger.size() == 2);
  assert(ledger.observe(RecoveryIdentity{"run", 1, 3}, src3) == RecoveryIdentityDecision::kNew);
  assert(ledger.size() == 2);
  // Evicting id1 also evicts its source key; it must not leave a stale alias.
  assert(ledger.observe(RecoveryIdentity{"run", 1, 4}, src1) == RecoveryIdentityDecision::kNew);
  ledger.clear();
  assert(ledger.size() == 0);
}
