// Road-step warps on the GPU (2026-10-23 option `road_step_gpu`).
//
// The CPU path builds, per candidate, a coarse lattice of source positions (the fisheye
// composed with the plane-induced homography, every `stride` pixels), stretches it to the
// region with cv::resize(INTER_LINEAR) and samples the previous frame with cv::remap
// (INTER_LINEAR, BORDER_REPLICATE). Here the lattices of K candidates are uploaded at once
// and one kernel does the stretch and the sampling for every pixel of every candidate:
//   - the stretch follows cv::resize (INTER_LINEAR or INTER_CUBIC, as road_step_cubic_map) exactly (centre-aligned source
//     coordinate, clamped at the edges);
//   - the sample is cv::remap's own fixed-point rule (1/32 px, 15-bit weights, 8-bit
//     result, replicated borders), so a patch matches the CPU's pixel for pixel up to
//     the map's float rounding. (An exact float sample was tried first: 0.24 grey mean
//     difference, and enough to move the fit on badly conditioned frames.)
// Output: K float patches, row-major, region-sized, on the host.
#include <cuda_runtime.h>

#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>

namespace monoscale_tracker_gpu
{

namespace
{

// cv::resize INTER_CUBIC weights (A = -0.75) for the fractional part t.
__device__ void cubic_weights(float t, float w[4])
{
  const float A = -0.75f;
  w[0] = ((A * (t + 1.0f) - 5.0f * A) * (t + 1.0f) + 8.0f * A) * (t + 1.0f) - 4.0f * A;
  w[1] = ((A + 2.0f) * t - (A + 3.0f)) * t * t + 1.0f;
  w[2] = ((A + 2.0f) * (1.0f - t) - (A + 3.0f)) * (1.0f - t) * (1.0f - t) + 1.0f;
  w[3] = 1.0f - w[0] - w[1] - w[2];
}

// The lattice stretched to (x, y) of the region as cv::resize INTER_CUBIC does it: taps
// floor(f)-1 .. floor(f)+2, replicated at the edges, the fraction not reset there.
__device__ float2 cubic_at(const float2 * lattice, int cols, int rows, int x, int y, int roi_w, int roi_h)
{
  const float fx = static_cast<float>((x + 0.5) * (static_cast<double>(cols) / roi_w) - 0.5);
  const float fy = static_cast<float>((y + 0.5) * (static_cast<double>(rows) / roi_h) - 0.5);
  const int sx = static_cast<int>(floorf(fx));
  const int sy = static_cast<int>(floorf(fy));
  float wx[4];
  float wy[4];
  cubic_weights(fx - sx, wx);
  cubic_weights(fy - sy, wy);
  float2 acc = make_float2(0.0f, 0.0f);
  for (int j = 0; j < 4; ++j) {
    const int r = min(max(sy - 1 + j, 0), rows - 1);
    float2 row = make_float2(0.0f, 0.0f);
    for (int i = 0; i < 4; ++i) {
      const int c = min(max(sx - 1 + i, 0), cols - 1);
      const float2 v = lattice[r * cols + c];
      row.x += wx[i] * v.x;
      row.y += wx[i] * v.y;
    }
    acc.x += wy[j] * row.x;
    acc.y += wy[j] * row.y;
  }
  return acc;
}

// cv::remap(INTER_LINEAR, BORDER_REPLICATE) on 8-bit, as OpenCV computes it: the position
// rounded to 1/32 px (cvRound, to nearest even), integer bilinear weights
// (32 - tx)(32 - ty) * 32 summing to 2^15, the sum rounded and shifted by 15, each tap
// clamped into the frame. The value is the 8-bit pixel the CPU path produces.
__device__ float remap8(const unsigned char * src, size_t step, int w, int h, float mx, float my)
{
  const int X = __float2int_rn(mx * 32.0f);
  const int Y = __float2int_rn(my * 32.0f);
  const int ix = X >> 5;
  const int iy = Y >> 5;
  const int tx = X & 31;
  const int ty = Y & 31;
  const int x0 = min(max(ix, 0), w - 1);
  const int x1 = min(max(ix + 1, 0), w - 1);
  const int y0 = min(max(iy, 0), h - 1);
  const int y1 = min(max(iy + 1, 0), h - 1);
  const unsigned char * r0 = src + static_cast<size_t>(y0) * step;
  const unsigned char * r1 = src + static_cast<size_t>(y1) * step;
  const int value = r0[x0] * ((32 - tx) * (32 - ty) * 32) + r0[x1] * (tx * (32 - ty) * 32) +
    r1[x0] * ((32 - tx) * ty * 32) + r1[x1] * (tx * ty * 32);
  return static_cast<float>(min(max((value + (1 << 14)) >> 15, 0), 255));
}

__global__ void warp_kernel(
  const unsigned char * src, size_t src_step, int src_w, int src_h,
  const float2 * lattices, int lat_cols, int lat_rows,
  int roi_w, int roi_h, bool cubic, float * out)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  const int k = blockIdx.z;
  if (x >= roi_w || y >= roi_h) {return;}
  float px;
  float py;
  if (cubic) {
    const float2 p = cubic_at(lattices + static_cast<size_t>(k) * lat_cols * lat_rows,
      lat_cols, lat_rows, x, y, roi_w, roi_h);
    px = p.x;
    py = p.y;
  } else {
    // cv::resize INTER_LINEAR, float maps: fx = (dx + 0.5) * scale - 0.5, clamped.
    float fx = (x + 0.5f) * (static_cast<float>(lat_cols) / roi_w) - 0.5f;
    float fy = (y + 0.5f) * (static_cast<float>(lat_rows) / roi_h) - 0.5f;
    int x0 = static_cast<int>(floorf(fx));
    int y0 = static_cast<int>(floorf(fy));
    float ax = fx - x0;
    float ay = fy - y0;
    if (x0 < 0) {x0 = 0; ax = 0.0f;}
    if (x0 >= lat_cols - 1) {x0 = lat_cols - 1; ax = 0.0f;}
    if (y0 < 0) {y0 = 0; ay = 0.0f;}
    if (y0 >= lat_rows - 1) {y0 = lat_rows - 1; ay = 0.0f;}
    const int x1 = min(x0 + 1, lat_cols - 1);
    const int y1 = min(y0 + 1, lat_rows - 1);
    const float2 * lattice = lattices + static_cast<size_t>(k) * lat_cols * lat_rows;
    const float2 a = lattice[y0 * lat_cols + x0];
    const float2 b = lattice[y0 * lat_cols + x1];
    const float2 c = lattice[y1 * lat_cols + x0];
    const float2 d = lattice[y1 * lat_cols + x1];
    px = (1.0f - ay) * ((1.0f - ax) * a.x + ax * b.x) + ay * ((1.0f - ax) * c.x + ax * d.x);
    py = (1.0f - ay) * ((1.0f - ax) * a.y + ax * b.y) + ay * ((1.0f - ax) * c.y + ax * d.y);
  }
  const float value = remap8(src, src_step, src_w, src_h, px, py);
  out[(static_cast<size_t>(k) * roi_h + y) * roi_w + x] = value;
}

void check(cudaError_t status, const char * what)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string("road_warp: ") + what + ": " + cudaGetErrorString(status));
  }
}

}  // namespace

// `previous`: the frame on the device (CV_8UC1). `lattices`: K CV_32FC2 lattices of the
// same size. Fills `patches` with K CV_32F region-sized patches (host). Uses the calling
// thread's default stream (the tracker builds with --default-stream per-thread semantics
// not assumed: everything here is synchronous).
void warp_patches(
  const cv::cuda::GpuMat & previous, const std::vector<cv::Mat> & lattices,
  const cv::Size & region, bool cubic, std::vector<cv::Mat> & patches)
{
  const int k = static_cast<int>(lattices.size());
  patches.resize(static_cast<size_t>(k));
  if (k == 0) {return;}
  const int lat_cols = lattices[0].cols;
  const int lat_rows = lattices[0].rows;
  const size_t lattice_floats = static_cast<size_t>(lat_cols) * lat_rows * 2;
  thread_local std::vector<float> host_lattices;
  host_lattices.resize(lattice_floats * k);
  for (int i = 0; i < k; ++i) {
    const cv::Mat & l = lattices[static_cast<size_t>(i)];
    CV_Assert(l.type() == CV_32FC2 && l.cols == lat_cols && l.rows == lat_rows && l.isContinuous());
    std::copy(l.ptr<float>(), l.ptr<float>() + lattice_floats, host_lattices.data() + lattice_floats * i);
  }
  thread_local float2 * device_lattices = nullptr;
  thread_local size_t device_lattice_bytes = 0;
  thread_local float * device_out = nullptr;
  thread_local size_t device_out_bytes = 0;
  const size_t lattice_bytes = host_lattices.size() * sizeof(float);
  const size_t out_bytes = static_cast<size_t>(k) * region.width * region.height * sizeof(float);
  if (lattice_bytes > device_lattice_bytes) {
    if (device_lattices != nullptr) {cudaFree(device_lattices);}
    check(cudaMalloc(&device_lattices, lattice_bytes), "malloc lattices");
    device_lattice_bytes = lattice_bytes;
  }
  if (out_bytes > device_out_bytes) {
    if (device_out != nullptr) {cudaFree(device_out);}
    check(cudaMalloc(&device_out, out_bytes), "malloc patches");
    device_out_bytes = out_bytes;
  }
  check(cudaMemcpy(device_lattices, host_lattices.data(), lattice_bytes, cudaMemcpyHostToDevice), "upload");
  const dim3 block(32, 8);
  const dim3 grid((region.width + block.x - 1) / block.x, (region.height + block.y - 1) / block.y, k);
  warp_kernel<<<grid, block>>>(
    previous.ptr<unsigned char>(), previous.step, previous.cols, previous.rows,
    device_lattices, lat_cols, lat_rows, region.width, region.height, cubic, device_out);
  check(cudaGetLastError(), "launch");
  thread_local std::vector<float> host_out;
  host_out.resize(out_bytes / sizeof(float));
  check(cudaMemcpy(host_out.data(), device_out, out_bytes, cudaMemcpyDeviceToHost), "download");
  const size_t patch_floats = static_cast<size_t>(region.width) * region.height;
  for (int i = 0; i < k; ++i) {
    cv::Mat & p = patches[static_cast<size_t>(i)];
    p.create(region, CV_32F);
    std::copy(host_out.data() + patch_floats * i, host_out.data() + patch_floats * (i + 1), p.ptr<float>());
  }
}

}  // namespace monoscale_tracker_gpu

// ---------------------------------------------------------------------------------------
// v2 (road_step_gpu_gram): lattice, warp and the products' sums all on the device.
//
// For K candidates (each its inverse homography) and the target region of `current`, the
// host receives only the K+1 sums and the (K+1)x(K+1) cross sums over the region; every
// normalised correlation the search and the fit use is a function of those. Lattice as
// warp_lattice / build_warp_roi lay it (centres of a cols x rows grid over the region),
// stretched like cv::resize (linear or cubic), sampled bilinear with replicated borders.
namespace monoscale_tracker_gpu
{

namespace
{

constexpr int kMaxGram = 12;  // candidates + target

struct Lens
{
  double fx, fy, cx, cy;
  int equidistant;
};

__global__ void lattice_kernel(
  Lens lens, const double * inverse, int k, int roi_x, int roi_y, int roi_w, int roi_h,
  int cols, int rows, float2 * out)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  const int c = blockIdx.y;  // candidate
  if (i >= cols * rows || c >= k) {return;}
  const int r = i / cols;
  const int q = i % cols;
  const double sx_step = static_cast<double>(roi_w) / cols;
  const double sy_step = static_cast<double>(roi_h) / rows;
  // warp_lattice passes the point through cv::Point2f (float) to bearing().
  const float fxp = static_cast<float>(roi_x + (q + 0.5) * sx_step - 0.5);
  const float fyp = static_cast<float>(roi_y + (r + 0.5) * sy_step - 0.5);
  const double xn = (fxp - lens.cx) / lens.fx;
  const double yn = (fyp - lens.cy) / lens.fy;
  double b0, b1, b2;
  if (lens.equidistant) {
    const double theta = hypot(xn, yn);
    const double s = theta > 1e-9 ? sin(theta) / theta : 1.0;
    b0 = xn * s; b1 = yn * s; b2 = cos(theta);
  } else {
    const double n = sqrt(xn * xn + yn * yn + 1.0);
    b0 = xn / n; b1 = yn / n; b2 = 1.0 / n;
  }
  const double * h = inverse + 9 * c;
  const double v0 = h[0] * b0 + h[1] * b1 + h[2] * b2;
  const double v1 = h[3] * b0 + h[4] * b1 + h[5] * b2;
  const double v2 = h[6] * b0 + h[7] * b1 + h[8] * b2;
  const double rr = hypot(v0, v1);
  double scale;
  if (lens.equidistant) {
    const double theta = atan2(rr, v2);
    scale = rr > 1e-12 ? theta / rr : 1.0;
  } else {
    scale = fabs(v2) > 1e-12 ? 1.0 / v2 : 0.0;
  }
  out[static_cast<size_t>(c) * cols * rows + i] = make_float2(
    static_cast<float>(v0 * scale * lens.fx + lens.cx),
    static_cast<float>(v1 * scale * lens.fy + lens.cy));
}

__device__ float2 linear_at(const float2 * lattice, int cols, int rows, int x, int y, int roi_w, int roi_h)
{
  float fx = (x + 0.5f) * (static_cast<float>(cols) / roi_w) - 0.5f;
  float fy = (y + 0.5f) * (static_cast<float>(rows) / roi_h) - 0.5f;
  int x0 = static_cast<int>(floorf(fx));
  int y0 = static_cast<int>(floorf(fy));
  float ax = fx - x0;
  float ay = fy - y0;
  if (x0 < 0) {x0 = 0; ax = 0.0f;}
  if (x0 >= cols - 1) {x0 = cols - 1; ax = 0.0f;}
  if (y0 < 0) {y0 = 0; ay = 0.0f;}
  if (y0 >= rows - 1) {y0 = rows - 1; ay = 0.0f;}
  const int x1 = min(x0 + 1, cols - 1);
  const int y1 = min(y0 + 1, rows - 1);
  const float2 a = lattice[y0 * cols + x0];
  const float2 b = lattice[y0 * cols + x1];
  const float2 c = lattice[y1 * cols + x0];
  const float2 d = lattice[y1 * cols + x1];
  return make_float2(
    (1.0f - ay) * ((1.0f - ax) * a.x + ax * b.x) + ay * ((1.0f - ax) * c.x + ax * d.x),
    (1.0f - ay) * ((1.0f - ax) * a.y + ax * b.y) + ay * ((1.0f - ax) * c.y + ax * d.y));
}

__device__ double warp_sum(double v)
{
  for (int offset = 16; offset > 0; offset /= 2) {v += __shfl_down_sync(0xffffffff, v, offset);}
  return v;
}

// out: [0, m) sums, then m*m cross sums (row-major, upper triangle filled; m = k + 1,
// index k is the target).
__global__ void gram_kernel(
  const unsigned char * previous, size_t previous_step, int w, int h,
  const unsigned char * current, size_t current_step,
  const float2 * lattices, int k, int cols, int rows, bool cubic,
  int roi_x, int roi_y, int roi_w, int roi_h, double * out)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  const bool inside = x < roi_w && y < roi_h;
  const int m = k + 1;
  float v[kMaxGram];
  for (int i = 0; i < m; ++i) {v[i] = 0.0f;}
  if (inside) {
    for (int c = 0; c < k; ++c) {
      const float2 * lattice = lattices + static_cast<size_t>(c) * cols * rows;
      const float2 p = cubic ? cubic_at(lattice, cols, rows, x, y, roi_w, roi_h)
                             : linear_at(lattice, cols, rows, x, y, roi_w, roi_h);
      v[c] = remap8(previous, previous_step, w, h, p.x, p.y);
    }
    v[k] = current[static_cast<size_t>(roi_y + y) * current_step + roi_x + x];
  }
  const int lane = (threadIdx.y * blockDim.x + threadIdx.x) & 31;
  for (int i = 0; i < m; ++i) {
    const double s = warp_sum(static_cast<double>(v[i]));
    if (lane == 0) {atomicAdd(out + i, s);}
  }
  for (int i = 0; i < m; ++i) {
    for (int j = i; j < m; ++j) {
      const double s = warp_sum(static_cast<double>(v[i]) * v[j]);
      if (lane == 0) {atomicAdd(out + m + i * m + j, s);}
    }
  }
}

}  // namespace

// K candidates, each its inverse plane homography (row-major 3x3), against `current(roi)`.
// lens: fx, fy, cx, cy already scaled to the frame, equidistant flag. Fills sums (K+1) and
// cross ((K+1)^2, symmetric). Synchronous.
void gram_patches(
  const cv::cuda::GpuMat & previous, const cv::cuda::GpuMat & current, const cv::Rect & roi,
  int cols, int rows, bool cubic, const double lens[5], const std::vector<double> & inverses,
  std::vector<double> & sums, std::vector<double> & cross)
{
  const int k = static_cast<int>(inverses.size() / 9);
  if (k < 1 || k + 1 > kMaxGram) {throw std::runtime_error("gram_patches: candidate count out of range");}
  const int m = k + 1;
  thread_local double * device_inverse = nullptr;
  thread_local float2 * device_lattice = nullptr;
  thread_local double * device_out = nullptr;
  thread_local size_t lattice_capacity = 0;
  if (device_inverse == nullptr) {
    check(cudaMalloc(&device_inverse, sizeof(double) * 9 * kMaxGram), "malloc inverse");
    check(cudaMalloc(&device_out, sizeof(double) * (kMaxGram + kMaxGram * kMaxGram)), "malloc gram");
  }
  const size_t lattice_points = static_cast<size_t>(cols) * rows * k;
  if (lattice_points > lattice_capacity) {
    if (device_lattice != nullptr) {cudaFree(device_lattice);}
    check(cudaMalloc(&device_lattice, sizeof(float2) * lattice_points), "malloc lattice");
    lattice_capacity = lattice_points;
  }
  const size_t out_count = static_cast<size_t>(m + m * m);
  check(cudaMemcpy(device_inverse, inverses.data(), sizeof(double) * 9 * k, cudaMemcpyHostToDevice), "upload inverse");
  check(cudaMemset(device_out, 0, sizeof(double) * out_count), "memset");
  const Lens l{lens[0], lens[1], lens[2], lens[3], static_cast<int>(lens[4])};
  const dim3 lattice_grid((cols * rows + 127) / 128, k);
  lattice_kernel<<<lattice_grid, 128>>>(l, device_inverse, k, roi.x, roi.y, roi.width, roi.height,
    cols, rows, device_lattice);
  check(cudaGetLastError(), "lattice launch");
  const dim3 block(32, 8);
  const dim3 grid((roi.width + block.x - 1) / block.x, (roi.height + block.y - 1) / block.y);
  gram_kernel<<<grid, block>>>(
    previous.ptr<unsigned char>(), previous.step, previous.cols, previous.rows,
    current.ptr<unsigned char>(), current.step, device_lattice, k, cols, rows, cubic,
    roi.x, roi.y, roi.width, roi.height, device_out);
  check(cudaGetLastError(), "gram launch");
  std::vector<double> host(out_count);
  check(cudaMemcpy(host.data(), device_out, sizeof(double) * out_count, cudaMemcpyDeviceToHost), "download gram");
  sums.assign(host.begin(), host.begin() + m);
  cross.assign(static_cast<size_t>(m) * m, 0.0);
  for (int i = 0; i < m; ++i) {
    for (int j = i; j < m; ++j) {
      cross[static_cast<size_t>(i) * m + j] = host[static_cast<size_t>(m + i * m + j)];
      cross[static_cast<size_t>(j) * m + i] = host[static_cast<size_t>(m + i * m + j)];
    }
  }
}

}  // namespace monoscale_tracker_gpu
