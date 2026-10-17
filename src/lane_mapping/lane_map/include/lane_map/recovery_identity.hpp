#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <tuple>

namespace lane_fusion {

using RecoveryIdentity = std::tuple<std::string, uint64_t, uint64_t>;
using RecoverySourceKey = std::tuple<std::string, uint64_t, std::string, int64_t>;

enum class RecoveryIdentityDecision { kNew, kDuplicateId, kDuplicateSource, kIdConflict };

class RecoveryIdentityLedger {
 public:
  explicit RecoveryIdentityLedger(size_t capacity = 8192) : capacity_(capacity) {}

  RecoveryIdentityDecision observe(const RecoveryIdentity& identity, const RecoverySourceKey& source) {
    const auto by_id = by_id_.find(identity);
    if (by_id != by_id_.end()) {
      return by_id->second == source ? RecoveryIdentityDecision::kDuplicateId
                                     : RecoveryIdentityDecision::kIdConflict;
    }
    if (by_source_.count(source)) return RecoveryIdentityDecision::kDuplicateSource;
    if (capacity_ == 0) return RecoveryIdentityDecision::kDuplicateSource;
    by_id_.emplace(identity, source);
    by_source_.emplace(source, identity);
    order_.emplace_back(identity, source);
    while (order_.size() > capacity_) {
      const auto oldest = order_.front();
      order_.pop_front();
      by_id_.erase(oldest.first);
      by_source_.erase(oldest.second);
    }
    return RecoveryIdentityDecision::kNew;
  }

  void clear() {
    by_id_.clear();
    by_source_.clear();
    order_.clear();
  }

  size_t size() const { return by_id_.size(); }

 private:
  size_t capacity_;
  std::map<RecoveryIdentity, RecoverySourceKey> by_id_;
  std::map<RecoverySourceKey, RecoveryIdentity> by_source_;
  std::deque<std::pair<RecoveryIdentity, RecoverySourceKey>> order_;
};

}  // namespace lane_fusion
