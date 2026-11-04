// E-8 golden output: the CPU detection path (remap -> preprocess -> vectorize) over a
// fixed list of recorded JPEG frames, printed with exact (hex) floats. Two builds of the
// detection code give byte-identical files when their arithmetic is the same.
//
//   lane_pipeline_golden <calibration.yaml> <frames.txt: one JPEG path per line> <out.txt>
//
// GOLDEN_FROM_NODE_SOURCE builds it against the node source as it was before the
// Pipeline moved to a header (used once, to make the reference file).
#ifdef GOLDEN_FROM_NODE_SOURCE
#define main global_mapping_node_main
#include "../src/global_mapping_node.cpp"
#undef main
#else
#include "lane_detection/detection_pipeline.hpp"
using namespace lane_detection;
#endif

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::fprintf(stderr, "usage: %s <calibration.yaml> <frames.txt> <out.txt>\n", argv[0]);
    return 2;
  }
  Params params;
  params.camera_forward = 0.08;  // rig.yaml mount (the other mount values equal the defaults)
  Pipeline pipeline(params);
  std::ifstream list(argv[2]);
  std::FILE * out = std::fopen(argv[3], "w");
  if (!list || out == nullptr) {return 2;}
  std::string path;
  bool maps = false;
  int frame = 0;
  while (std::getline(list, path)) {
    if (path.empty()) {continue;}
    std::ifstream file(path, std::ios::binary);
    std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const cv::Mat image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
    if (image.empty()) {std::fprintf(stderr, "cannot decode %s\n", path.c_str()); return 1;}
    if (!maps) {
      maps = pipeline.initialize_maps(argv[1], image.cols, image.rows);
      if (!maps) {std::fprintf(stderr, "calibration load failed\n"); return 1;}
    }
    const cv::Mat bev = pipeline.remap_bev(image);
    const Prep prep = pipeline.preprocess(bev);
    int raw = 0;
    double dominant = 0.0;
    double confidence = 0.0;
    const auto observations = pipeline.vectorize(prep, &raw, &dominant, &confidence);
    std::fprintf(out, "frame %d raw %d dominant %a confidence %a count %zu binary_nonzero %d\n", frame, raw,
      dominant, confidence, observations.size(), cv::countNonZero(prep.binary));
    for (const auto & o : observations) {
      std::fprintf(out, "  %a %a %a %a %s %a %a %a %a\n", o.p0_m.x, o.p0_m.y, o.p1_m.x, o.p1_m.y,
        o.family.c_str(), o.angle, o.span, o.width, o.confidence);
    }
    ++frame;
  }
  std::fclose(out);
  return 0;
}
