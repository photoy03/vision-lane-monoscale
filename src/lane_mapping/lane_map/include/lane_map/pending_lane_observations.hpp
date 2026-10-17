#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <tuple>
#include <utility>

namespace vision_lane {

template<typename Payload>
struct PendingLaneObservation {
  uint64_t epoch{0};
  std::string camera_id;
  int64_t stamp_ns{0};
  int64_t arrival_steady_ns{0};
  Payload payload;
};

enum class PendingInsertResult {
  queued,
  queued_after_overflow,
  invalid,
  duplicate,
};

// A bounded, timestamp-ordered queue for local lane observations that have
// not yet received a measured pose. It deliberately knows nothing about map
// policy: callers must resolve the original timestamp with a measured pose
// before committing the payload.
template<typename Payload>
class PendingLaneObservationQueue {
 public:
  using Entry = PendingLaneObservation<Payload>;
  struct InsertResult {
    PendingInsertResult result{PendingInsertResult::invalid};
    std::optional<Entry> evicted;
  };

  explicit PendingLaneObservationQueue(size_t capacity = 128)
  : capacity_(capacity), seen_capacity_(std::max<size_t>(2048, capacity * 32)) {
    if (capacity_ == 0) throw std::invalid_argument("pending capacity must be positive");
  }

  InsertResult push(Entry entry) {
    if (entry.epoch == 0 || entry.camera_id.empty() || entry.stamp_ns <= 0 ||
        entry.arrival_steady_ns <= 0) return {PendingInsertResult::invalid, std::nullopt};
    const auto key = std::make_tuple(entry.epoch, entry.camera_id, entry.stamp_ns);
    if (seen_.count(key) != 0) return {PendingInsertResult::duplicate, std::nullopt};

    std::optional<Entry> evicted;
    if (queue_.size() >= capacity_) {
      evicted = std::move(queue_.front());
      queue_.pop_front();
    }
    const auto position = std::upper_bound(queue_.begin(), queue_.end(), entry,
      [](const Entry &a, const Entry &b) {
        if (a.stamp_ns != b.stamp_ns) return a.stamp_ns < b.stamp_ns;
        return a.camera_id < b.camera_id;
      });
    queue_.insert(position, std::move(entry));
    remember(key);
    return {evicted ? PendingInsertResult::queued_after_overflow : PendingInsertResult::queued,
      std::move(evicted)};
  }

  bool front_expired(int64_t now_steady_ns, int64_t ttl_ns) const {
    if (queue_.empty() || ttl_ns <= 0 || now_steady_ns < queue_.front().arrival_steady_ns)
      return false;
    return now_steady_ns - queue_.front().arrival_steady_ns >= ttl_ns;
  }

  const Entry *front() const { return queue_.empty() ? nullptr : &queue_.front(); }
  const std::deque<Entry> &entries() const { return queue_; }

  std::optional<Entry> pop_front() {
    if (queue_.empty()) return std::nullopt;
    Entry value = std::move(queue_.front());
    queue_.pop_front();
    return value;
  }

  bool contains(uint64_t epoch, const std::string &camera_id, int64_t stamp_ns) const {
    return seen_.count(std::make_tuple(epoch, camera_id, stamp_ns)) != 0 &&
      std::any_of(queue_.begin(), queue_.end(), [&](const Entry &entry) {
        return entry.epoch == epoch && entry.camera_id == camera_id &&
          entry.stamp_ns == stamp_ns;
      });
  }

  std::vector<Entry> drain_all() {
    std::vector<Entry> entries;
    entries.reserve(queue_.size());
    while (!queue_.empty()) {
      entries.push_back(std::move(queue_.front()));
      queue_.pop_front();
    }
    return entries;
  }

  size_t size() const { return queue_.size(); }
  bool empty() const { return queue_.empty(); }

  void clear() {
    queue_.clear();
    seen_.clear();
    seen_order_.clear();
  }

 private:
  using Key = std::tuple<uint64_t, std::string, int64_t>;

  void remember(const Key &key) {
    seen_.insert(key);
    seen_order_.push_back(key);
    while (seen_order_.size() > seen_capacity_) {
      seen_.erase(seen_order_.front());
      seen_order_.pop_front();
    }
  }

  size_t capacity_;
  size_t seen_capacity_;
  std::deque<Entry> queue_;
  std::set<Key> seen_;
  std::deque<Key> seen_order_;
};

}  // namespace vision_lane
