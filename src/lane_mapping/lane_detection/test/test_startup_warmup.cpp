#include <cassert>
#include <cstdint>
#include "startup_warmup.hpp"

int main() {
  using vision_lane::StartupWarmup;
  const int64_t s = 1000000000;
  {  // seconds = 0: the original rule, warm-up = frames 1..120, trackers reset at 121.
    StartupWarmup w(120, 0.0);
    for (uint64_t f = 1; f <= 120; ++f) { assert(w.update(f, static_cast<int64_t>(f) * s / 2)); assert(!w.ended()); }
    assert(!w.update(121, 121 * s / 2) && w.ended());
    assert(!w.update(122, 61 * s) && !w.ended());
    assert(!w.update(123, 1 * s) && !w.ended());  // a backward jump does not restart the original rule
  }
  {  // No warm-up configured at all.
    StartupWarmup w(0, 0.0);
    assert(!w.update(1, s) && !w.ended());
  }
  {  // 2.3 Hz: time (12 s) comes first, after about 28 frames.
    StartupWarmup w(120, 12.0, 20);
    uint64_t end = 0;
    for (uint64_t f = 1; f <= 120 && !end; ++f) {
      w.update(f, 100 * s + static_cast<int64_t>(f * s / 2.3));
      if (w.ended()) end = f;
    }
    assert(end >= 28 && end <= 29);
  }
  {  // 1 Hz: 12 s reached at frame 13, but the 20-frame floor holds it to frame 20.
    StartupWarmup w(120, 12.0, 20);
    uint64_t end = 0;
    for (uint64_t f = 1; f <= 40 && !end; ++f) {
      w.update(f, static_cast<int64_t>(f) * s);
      if (w.ended()) end = f;
    }
    assert(end == 20);
  }
  {  // 30 Hz: 120 frames (4 s) come before 12 s.
    StartupWarmup w(120, 12.0, 20);
    uint64_t end = 0;
    for (uint64_t f = 1; f <= 400 && !end; ++f) {
      w.update(f, static_cast<int64_t>(f) * s / 30);
      if (w.ended()) end = f;
    }
    assert(end == 121);
  }
  {  // A forward jump while warming up restarts the time rule.
    StartupWarmup w(1000, 12.0, 5);
    for (uint64_t f = 1; f <= 10; ++f) assert(w.update(f, static_cast<int64_t>(f) * s));
    assert(w.update(11, 30 * s));               // 20 s jump: base moves to 30 s
    uint64_t end = 0;
    for (uint64_t f = 12; f <= 40 && !end; ++f) {
      w.update(f, 30 * s + static_cast<int64_t>(f - 11) * s);
      if (w.ended()) end = f;
    }
    assert(end == 23);                           // 12 s after 30 s
  }
  {  // A backward jump after warm-up starts it again.
    StartupWarmup w(1000, 12.0, 5);
    uint64_t f = 1;
    for (; f <= 20; ++f) w.update(f, static_cast<int64_t>(f) * s);
    assert(!w.update(f++, 21 * s));
    assert(w.update(f++, 2 * s));                // clock back: warming again
    uint64_t end = 0;
    for (int k = 1; k <= 20 && !end; ++k, ++f) {
      w.update(f, (2 + k) * s);
      if (w.ended()) end = static_cast<uint64_t>(k);
    }
    assert(end == 12);
  }
  {  // Motion condition: a car standing still keeps warming up past the time
     // rule until it has moved min_motion_m (here 1 m at 0.05 m/s from 20 s on).
    StartupWarmup w(120, 12.0, 20, 1.0);
    uint64_t end = 0;
    double travelled = 0.0;
    for (uint64_t f = 1; f <= 119 && !end; ++f) {
      const double t = static_cast<double>(f) * 0.4;
      if (t > 20.0) travelled += 0.05 * 0.4;
      w.update(f, static_cast<int64_t>(t * 1e9), travelled);
      if (w.ended()) end = f;
    }
    assert(end >= 99 && end <= 101);  // 20 s + 1 m / 0.05 m/s = 40 s = frame 100
  }
  {  // Motion condition never met: the frame rule (120) still ends warm-up.
    StartupWarmup w(120, 12.0, 20, 1.0);
    uint64_t end = 0;
    for (uint64_t f = 1; f <= 200 && !end; ++f) {
      w.update(f, static_cast<int64_t>(f) * s / 2, 0.0);
      if (w.ended()) end = f;
    }
    assert(end == 121);
  }
  {  // Moving from the start: the time rule decides as without the condition.
    StartupWarmup w(120, 12.0, 20, 1.0);
    uint64_t end = 0;
    for (uint64_t f = 1; f <= 120 && !end; ++f) {
      w.update(f, 100 * s + static_cast<int64_t>(f * s / 2.3), 0.5 * static_cast<double>(f) / 2.3);
      if (w.ended()) end = f;
    }
    assert(end >= 28 && end <= 29);
  }
  return 0;
}
