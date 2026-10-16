#include <cassert>
#include <iostream>
#include <limits>
#include "monoscale_core/solve_schedule.hpp"
#include "monoscale_core/imu_gate.hpp"

int main() {
  using monoscale::solve_due;
  assert(!solve_due(0, 10, 1, 2, 40, 3, .1)); // no new frame, no update
  assert(!solve_due(1, 10, .03, 2, 40, 3, .1));
  assert(solve_due(2, 3, .06, 2, 40, 3, .1));
  assert(!solve_due(3, 0, .099, 2, 40, 3, .1));
  assert(solve_due(3, 0, .100, 2, 40, 3, .1));
  assert(solve_due(1, 0, .2, 2, 40, 3, .1)); // slow input, use its real stamp
  assert(!solve_due(39, 0, 2, 2, 40, 3, 0)); // legacy deadline disabled
  assert(solve_due(40, 0, 2, 2, 40, 3, 0));
  assert(solve_due(3, 0, .09, 2, 3, 3, .1)); // frame cap
  std::array<double, 4> q{0, 0, 0, 1};
  std::array<double, 9> cov{};
  assert(monoscale::imu_orientation_usable(q, cov, false));
  cov[0] = -1;
  assert(!monoscale::imu_orientation_usable(q, cov, false));
  cov[0] = cov[4] = 1e6;
  assert(monoscale::imu_orientation_usable(q, cov, false)); // yaw valid
  assert(!monoscale::imu_orientation_usable(q, cov, true)); // tilt unknown
  cov.fill(0);
  assert(monoscale::imu_orientation_usable(q, cov, true)); // not an accuracy guarantee
  q.fill(0);
  assert(!monoscale::imu_orientation_usable(q, cov, false));
  q[0] = std::numeric_limits<double>::quiet_NaN();
  assert(!monoscale::imu_orientation_usable(q, cov, false));
  std::cout << "PASS solve deadline/frame policy and IMU orientation gate\n";
}
