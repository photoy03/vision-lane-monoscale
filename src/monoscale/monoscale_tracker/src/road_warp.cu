// Road-step warps on the GPU (2026-10-23 option `road_step_gpu`).
//
// The CPU path builds, per candidate, a coarse lattice of source positions (the fisheye
// composed with the plane-induced homography, every `stride` pixels), stretches it to the
// region with cv::resize(INTER_LINEAR) and samples the previous frame with cv::remap
// (INTER_LINEAR, BORDER_REPLICATE). Here the lattices of K candidates are uploaded at once
// and one kernel does the stretch and the sampling for every pixel of every candidate:
//   - the stretch follows cv::resize (INTER_LINEAR or INTER_CUBIC, as road_step_cubic_map) exactly (centre-aligned source
//     coordinate, clamped at the edges);
//   - the sample is bilinear in float with replicated borders. cv::remap rounds the
//     position to 1/32 px and the value to 8 bits; this does neither, so the patches
//     differ from the CPU's by that quantisation (measured on the CPU path as no
//     change in the fitted step, see road_scores' note).
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
  // Bilinear sample, replicated border.
  const float sx = fminf(fmaxf(px, 0.0f), static_cast<float>(src_w - 1));
  const float sy = fminf(fmaxf(py, 0.0f), static_cast<float>(src_h - 1));
  const int u0 = static_cast<int>(floorf(sx));
  const int v0 = static_cast<int>(floorf(sy));
  const int u1 = min(u0 + 1, src_w - 1);
  const int v1 = min(v0 + 1, src_h - 1);
  const float bx = sx - u0;
  const float by = sy - v0;
  const unsigned char * r0 = src + static_cast<size_t>(v0) * src_step;
  const unsigned char * r1 = src + static_cast<size_t>(v1) * src_step;
  const float value = (1.0f - by) * ((1.0f - bx) * r0[u0] + bx * r0[u1]) +
    by * ((1.0f - bx) * r1[u0] + bx * r1[u1]);
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
