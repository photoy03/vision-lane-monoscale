// Search loop of parking_scenario/hybrid_astar.py (HybridAStar.plan), line for line.
// The Python code stays the reference; this file must give the same path. Loaded through ctypes.
#include <cmath>
#include <chrono>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {
const double TWO_PI = 2.0 * M_PI;
inline double pymod(double a, double b) { double r = std::fmod(a, b); return (r != 0.0 && ((r < 0.0) != (b < 0.0))) ? r + b : r; }
inline long floordiv(long a, long b) { long q = a / b; return ((a % b != 0) && ((a < 0) != (b < 0))) ? q - 1 : q; }

struct Input {
  const float* distance; const uint8_t* unknown; int rows, cols; double ox, oy, res;
  const double* circles; int n_circles;                       // (x, y, radius) each
  const double* hfield; int hrows, hcols, hk;
  const float* table; int tn, tbins; double textent, tcell;   // (tn, tn, tbins)
  const double* prim; int n_prim;                             // per primitive: x0 x1 y0 y1 yaw0 yaw1 reverse steer
  double start[3], goal[3];
  double xy_resolution; int heading_bins; double step_m, reverse_cost, switch_cost, steer_cost, unknown_cost;
  double capture_heading_rad, capture_length_m, time_limit_s, heuristic_weight; int max_expansions;
  double kappa_max;
};
struct Key { long x, y, b; bool operator==(const Key& o) const { return x == o.x && y == o.y && b == o.b; } };
struct KeyHash { size_t operator()(const Key& k) const { return (size_t)(k.x * 73856093L) ^ (size_t)(k.y * 19349663L) ^ (size_t)(k.b * 83492791L); } };
struct Node { double cost, x, y, yaw; Key parent; bool has_parent; int reverse; };   // reverse: -1 none, 0, 1
struct Entry { double f, cost; Key key; };
struct Later { bool operator()(const Entry& a, const Entry& b) const {       // min-heap in Python tuple order
  if (a.f != b.f) return a.f > b.f; if (a.cost != b.cost) return a.cost > b.cost;
  if (a.key.x != b.key.x) return a.key.x > b.key.x; if (a.key.y != b.key.y) return a.key.y > b.key.y; return a.key.b > b.key.b; } };

struct Search {
  const Input& in; double gc, gs, dtheta;
  explicit Search(const Input& i) : in(i), gc(std::cos(i.goal[2])), gs(std::sin(i.goal[2])), dtheta(TWO_PI / i.heading_bins) {}
  inline bool inside(long row, long col) const { return row >= 0 && row < in.rows && col >= 0 && col < in.cols; }
  bool collides(double x, double y, double yaw) const {
    double c = std::cos(yaw), s = std::sin(yaw);
    for (int i = 0; i < in.n_circles; ++i) {
      double cx = in.circles[3 * i], cy = in.circles[3 * i + 1], r = in.circles[3 * i + 2];
      long row = (long)((y + s * cx + c * cy - in.oy) / in.res), col = (long)((x + c * cx - s * cy - in.ox) / in.res);
      if (!inside(row, col) || (double)in.distance[row * in.cols + col] < r) return true;
    }
    return false;
  }
  Key key(double x, double y, double yaw) const {
    return Key{(long)std::floor(x / in.xy_resolution), (long)std::floor(y / in.xy_resolution),
               (long)pymod((double)(long)std::floor(pymod(yaw, TWO_PI) / dtheta), (double)in.heading_bins)};
  }
  double heuristic(double x, double y, double yaw) const {
    long row = (long)((y - in.oy) / in.res), col = (long)((x - in.ox) / in.res);
    row = std::min(std::max(floordiv(row, in.hk), 0L), (long)in.hrows - 1);
    col = std::min(std::max(floordiv(col, in.hk), 0L), (long)in.hcols - 1);
    double h = in.hfield[row * in.hcols + col];
    double dx = x - in.goal[0], dy = y - in.goal[1];
    double lx = gc * dx + gs * dy, ly = -gs * dx + gc * dy;
    double other;
    if (std::fabs(lx) >= in.textent || std::fabs(ly) >= in.textent) other = std::hypot(dx, dy);
    else {
      long b = (long)pymod((double)(long)(pymod(yaw - in.goal[2], TWO_PI) / (TWO_PI / in.tbins)), (double)in.tbins);
      other = (double)in.table[((long)((lx + in.textent) / in.tcell) * in.tn + (long)((ly + in.textent) / in.tcell)) * in.tbins + b];
    }
    return (other > h) ? other : h;                           // Python max(h, other): the first unless the second is larger
  }
  // one arc onto the goal axis, then straight along it; false = none
  bool capture(double x, double y, double yaw, std::vector<double>& run) const {
    run.clear();
    double ds = 0.5 * in.step_m;
    double dx = x - in.goal[0], dy = y - in.goal[1];
    double along = gc * dx + gs * dy, lateral = -gs * dx + gc * dy;
    double heading = std::atan2(std::sin(yaw - in.goal[2]), std::cos(yaw - in.goal[2]));
    if (std::fabs(heading) > in.capture_heading_rad || std::fabs(along) > in.capture_length_m + 6.0) return false;
    double end_along;
    if (std::fabs(lateral) < 0.03 && std::fabs(heading) < (1.0 * M_PI / 180.0)) end_along = along;
    else {
      if (std::fabs(lateral) < 1e-6 || std::fabs(heading) < 1e-6) return false;
      double kappa = (1.0 - std::cos(heading)) / lateral;
      if (std::fabs(kappa) > in.kappa_max) return false;
      double arc = -heading / kappa;
      end_along = along - std::sin(heading) / kappa;
      if (std::fabs(arc) > 8.0) return false;
      long n = std::max(1L, (long)std::ceil(std::fabs(arc) / ds));
      for (long i = 1; i <= n; ++i) {
        double d = arc * i / n;
        double a = along + (std::sin(heading + d * kappa) - std::sin(heading)) / kappa;
        double lat = lateral + (std::cos(heading) - std::cos(heading + d * kappa)) / kappa;
        double px = in.goal[0] + gc * a - gs * lat, py = in.goal[1] + gs * a + gc * lat, pyaw = in.goal[2] + heading + d * kappa;
        if (collides(px, py, pyaw)) return false;
        run.insert(run.end(), {px, py, pyaw, arc < 0 ? 1.0 : 0.0});
      }
    }
    if (std::fabs(end_along) > in.capture_length_m) return false;
    long n = std::max(1L, (long)std::ceil(std::fabs(end_along) / ds));
    for (long i = 1; i <= n; ++i) {
      double a = end_along * (1 - (double)i / n);
      double px = in.goal[0] + gc * a, py = in.goal[1] + gs * a;
      if (collides(px, py, in.goal[2])) return false;
      run.insert(run.end(), {px, py, in.goal[2], end_along > 0 ? 1.0 : 0.0});
    }
    return true;
  }
};
}  // namespace

extern "C" {
// HybridAStar._heuristic_field: obstacle-aware 2D distance to the goal cell on the grid taken every k cells.
// out: hrows x hcols doubles (inf where not reached). The queue order is the Python one (d, row, col).
void parking_hfield(const float* distance, int cols, int k, double half_width, double res,
                    int goal_row, int goal_col, double* out, int hrows, int hcols) {
  struct Item { double d; int r, c; };
  struct After { bool operator()(const Item& a, const Item& b) const {
    if (a.d != b.d) return a.d > b.d; if (a.r != b.r) return a.r > b.r; return a.c > b.c; } };
  const float limit = (float)half_width;                    // numpy compares the float32 grid in float32
  std::vector<uint8_t> open((size_t)hrows * hcols);
  for (int r = 0; r < hrows; ++r) for (int c = 0; c < hcols; ++c) {
    open[(size_t)r * hcols + c] = distance[(size_t)(r * k) * cols + (size_t)c * k] >= limit;
    out[(size_t)r * hcols + c] = INFINITY;
  }
  out[(size_t)goal_row * hcols + goal_col] = 0.0;
  std::priority_queue<Item, std::vector<Item>, After> heap;
  heap.push(Item{0.0, goal_row, goal_col});
  const double step = res * k;
  static const int dr[8] = {-1, 1, 0, 0, -1, -1, 1, 1}, dc[8] = {0, 0, -1, 1, -1, 1, -1, 1};
  static const double w[8] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};
  while (!heap.empty()) {
    Item it = heap.top(); heap.pop();
    if (it.d > out[(size_t)it.r * hcols + it.c]) continue;
    for (int m = 0; m < 8; ++m) {
      int nr = it.r + dr[m], nc = it.c + dc[m];
      if (nr < 0 || nr >= hrows || nc < 0 || nc >= hcols || !open[(size_t)nr * hcols + nc]) continue;
      double nd = it.d + w[m] * step;
      if (nd < out[(size_t)nr * hcols + nc]) { out[(size_t)nr * hcols + nc] = nd; heap.push(Item{nd, nr, nc}); }
    }
  }
}

// Returns 0 found, 1 search limit, 2 no path. out: (x, y, yaw, reverse) rows, the chain first (n_chain
// rows, the start pose with reverse -1), then the capture run (n_run rows). At most max_out rows.
int parking_plan(const Input* input, double* out, int max_out, int* n_chain, int* n_run, int* expansions) {
  const Input& in = *input;
  Search s(in);
  auto began = std::chrono::steady_clock::now();
  std::unordered_map<Key, Node, KeyHash> nodes;
  std::unordered_set<Key, KeyHash> closed;
  std::priority_queue<Entry, std::vector<Entry>, Later> heap;
  nodes.reserve(1 << 16); closed.reserve(1 << 15);
  Key start_key = s.key(in.start[0], in.start[1], in.start[2]);
  nodes[start_key] = Node{0.0, in.start[0], in.start[1], in.start[2], Key{0, 0, 0}, false, -1};
  heap.push(Entry{s.heuristic(in.start[0], in.start[1], in.start[2]), 0.0, start_key});
  std::vector<double> run;
  bool found = false; Key final_key{0, 0, 0};
  int reason = 2;
  *expansions = 0;
  const double inv_res = 1.0 / in.res;
  while (!heap.empty()) {
    if (*expansions >= in.max_expansions ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count() > in.time_limit_s) { reason = 1; break; }
    Entry top = heap.top(); heap.pop();
    if (closed.count(top.key)) continue;
    closed.insert(top.key);
    const Node node = nodes[top.key];
    ++*expansions;
    if (s.capture(node.x, node.y, node.yaw, run)) { found = true; final_key = top.key; break; }
    double c0 = std::cos(node.yaw), s0 = std::sin(node.yaw);
    for (int i = 0; i < in.n_prim; ++i) {
      const double* p = in.prim + 8 * i;
      bool passable = true; double nx = 0, ny = 0, nyaw = 0;
      for (int h = 0; h < 2 && passable; ++h) {
        double wx = node.x + c0 * p[h] - s0 * p[2 + h], wy = node.y + s0 * p[h] + c0 * p[2 + h], wyaw = node.yaw + p[4 + h];
        double cc = std::cos(wyaw), ss = std::sin(wyaw);
        for (int k = 0; k < in.n_circles; ++k) {
          double cx = in.circles[3 * k], cy = in.circles[3 * k + 1], r = in.circles[3 * k + 2];
          long col = (long)((wx + cc * cx - ss * cy - in.ox) * inv_res), row = (long)((wy + ss * cx + cc * cy - in.oy) * inv_res);
          if (!s.inside(row, col) || !((double)in.distance[row * in.cols + col] >= r)) { passable = false; break; }
        }
        nx = wx; ny = wy; nyaw = wyaw;
      }
      if (!passable) continue;
      bool reverse = p[6] != 0.0; double steer = p[7];
      Key nk = s.key(nx, ny, nyaw);
      if (closed.count(nk)) continue;
      long row = (long)((ny - in.oy) / in.res), col = (long)((nx - in.ox) / in.res);
      double step_cost = in.step_m * (reverse ? in.reverse_cost : 1.0);
      step_cost += in.steer_cost * std::fabs(steer) * in.step_m;
      if (s.inside(row, col) && in.unknown[row * in.cols + col]) step_cost += in.unknown_cost * in.step_m;
      if (node.reverse >= 0 && (node.reverse != 0) != reverse) step_cost += in.switch_cost;
      double new_cost = top.cost + step_cost;
      auto it = nodes.find(nk);
      if (it == nodes.end() || new_cost < it->second.cost) {
        nodes[nk] = Node{new_cost, nx, ny, nyaw, top.key, true, reverse ? 1 : 0};
        heap.push(Entry{new_cost + in.heuristic_weight * s.heuristic(nx, ny, nyaw), new_cost, nk});
      }
    }
  }
  *n_chain = 0; *n_run = 0;
  if (!found) return reason;
  std::vector<double> chain;
  Key k = final_key; bool more = true;
  while (more) {
    const Node& n = nodes[k];
    chain.insert(chain.end(), {(double)n.reverse, n.yaw, n.y, n.x});     // reversed below
    more = n.has_parent; k = n.parent;
  }
  int nc = (int)chain.size() / 4, nr = (int)run.size() / 4;
  if (nc + nr > max_out) return 1;
  for (int i = 0; i < nc; ++i) for (int j = 0; j < 4; ++j) out[4 * i + j] = chain[chain.size() - 1 - (4 * i + j)];
  for (size_t i = 0; i < run.size(); ++i) out[4 * nc + i] = run[i];
  *n_chain = nc; *n_run = nr;
  return 0;
}
}
