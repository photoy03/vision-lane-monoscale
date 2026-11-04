// E-8 deterministic lane replay: the lane layer of one recorded run, every front frame in
// order, with the poses that run recorded (observer_eval_E8b 2). Same code as the nodes:
// detection (lane_detection/detection_pipeline.hpp, CPU), the warm-up rule
// (startup_warmup.hpp), pose matching and world tracking (lane_map/lane_mapper.hpp) and the
// bank update (lane_map/map_apply.hpp). Time is virtual: events are ordered by
//   pose      the receive time the run recorded for that kinematic_state message
//   frame     its stamp + a fixed detection latency (--detect-latency-ms)
// and the mapper's steady clock is that event time, so the output depends on the inputs only.
//
//   lane_offline --frames frames.txt --poses poses.txt --detection-params det.txt
//                --map-params map.txt --calibration calib.yaml --detect-latency-ms L --out DIR
// frames.txt: "stamp_ns path.jpg" per line (stamp order). poses.txt: "receive_ns stamp_ns x y
// qx qy qz qw vx vy wz cov0 cov7 cov35" per line. *params.txt: "name value" per line (a run's
// parameter dump flattened); a name that is missing takes the node default.
// Output in DIR: journal.jsonl (registered_keyframes format), snapshots.jsonl (the rendered
// lane map after every applied keyframe), creations.jsonl (each new bank line with the
// tracker track it came from), summary.json.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "lane_detection/detection_pipeline.hpp"
#include "lane_map/lane_mapper.hpp"
#include "lane_map/map_apply.hpp"
#include "lane_map/node_settings.hpp"
#include "startup_warmup.hpp"

namespace {

using KeyValues = std::map<std::string, std::string>;

KeyValues read_key_values(const std::string & path)
{
  KeyValues out;
  std::ifstream in(path);
  if (!in) {throw std::runtime_error("cannot read " + path);}
  std::string name, value;
  while (in >> name >> value) {out[name] = value;}
  return out;
}

// get.template operator()<T>(name, default) over a dump; the node would have declared the
// same name with the same default.
struct DumpGet {
  const KeyValues * values;
  template<class T> T operator()(const char * name, T fallback) const
  {
    const auto it = values->find(name);
    if (it == values->end()) {return fallback;}
    if constexpr (std::is_same_v<T, bool>) {return it->second == "true" || it->second == "True";}
    else if constexpr (std::is_integral_v<T>) {return static_cast<T>(std::stoll(it->second));}
    else {return static_cast<T>(std::stod(it->second));}
  }
};

struct Event {
  int64_t time_ns{0};
  int order{0};      // poses before frames at the same time, as the node drains odom first
  size_t index{0};
};

std::string arg(int argc, char ** argv, const std::string & name)
{
  for (int i = 1; i + 1 < argc; ++i) {if (argv[i] == name) {return argv[i + 1];}}
  throw std::runtime_error("missing " + name);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const KeyValues det = read_key_values(arg(argc, argv, "--detection-params"));
    const KeyValues map = read_key_values(arg(argc, argv, "--map-params"));
    const std::string calibration = arg(argc, argv, "--calibration");
    const double latency_ms = std::stod(arg(argc, argv, "--detect-latency-ms"));
    const std::string out_dir = arg(argc, argv, "--out");
    const DumpGet det_get{&det};
    const DumpGet map_get{&map};

    // Detection, as global_mapping_node builds it.
    const lane_detection::Params params = lane_detection::load_params(
      [&](const char * name, double value) {return det_get(name, value);},
      [&](const char * name, int value) {return det_get(name, value);});
    lane_detection::Pipeline pipeline(params);
    vision_lane::StartupWarmup warmup(det_get("startup_warmup_frames", 0), det_get("startup_warmup_sec", 0.0),
      det_get("startup_warmup_min_frames", 20), det_get("startup_warmup_min_motion_m", 0.0));
    // The lane map, as map_fusion_node builds it in mapper mode (warm-up 0 there).
    lane_fusion::RegisteredBank bank(lane_map_settings::bank_settings(map_get));
    lane_mapping::LaneMapper mapper(lane_map_settings::mapper_settings(map_get));
    lane_map_apply::Settings apply;
    apply.warmup = 0;
    apply.min_travel = map_get("keyframe_translation_m", 0.05);
    apply.min_yaw = map_get("keyframe_yaw_deg", 1.0) * lane_map_apply::kPi / 180.0;
    apply.fixed_extent = map.count("extent_mode") ? map.at("extent_mode") == "github_fixed" : true;
    apply.xmin = map_get("map_x_min_m", -10.0); apply.xmax = map_get("map_x_max_m", 20.0);
    apply.ymin = map_get("map_y_min_m", -15.0); apply.ymax = map_get("map_y_max_m", 15.0);
    apply.rolling_distance = map_get("rolling_distance_m", 60.0);
    const std::string camera = "front";

    // Inputs.
    std::vector<std::pair<int64_t, std::string>> frames;
    {
      std::ifstream in(arg(argc, argv, "--frames"));
      long long stamp; std::string path;
      while (in >> stamp >> path) {frames.push_back({stamp, path});}
    }
    std::vector<std::pair<int64_t, monoscale_pose::TimedPose>> poses;
    {
      std::ifstream in(arg(argc, argv, "--poses"));
      long long receive, stamp;
      double px, py, qx, qy, qz, qw, vx, vy, wz, c0, c7, c35;
      while (in >> receive >> stamp >> px >> py >> qx >> qy >> qz >> qw >> vx >> vy >> wz >> c0 >> c7 >> c35) {
        // map_fusion_node on_odom(), arithmetic unchanged.
        const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        if (stamp <= 0 || !std::isfinite(norm) || norm < 1e-9 || !std::isfinite(px) || !std::isfinite(py)) {continue;}
        const double x = qx / norm, y = qy / norm, z = qz / norm, w = qw / norm;
        const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        const double c = std::cos(yaw), s = std::sin(yaw);
        const auto safe = [](double v) {return std::isfinite(v) && v > 0.0 ? v : 0.0;};
        monoscale_pose::TimedPose pose{stamp, px, py, yaw, c * vx - s * vy, s * vx + c * vy, wz,
          std::max(safe(c0), safe(c7)), safe(c35), safe(c0) > 0.0 && safe(c7) > 0.0 && safe(c35) > 0.0, receive};
        poses.push_back({receive, pose});
      }
    }
    std::vector<Event> events;
    for (size_t i = 0; i < poses.size(); ++i) {events.push_back({poses[i].first, 0, i});}
    const int64_t latency_ns = static_cast<int64_t>(std::llround(latency_ms * 1e6));
    for (size_t i = 0; i < frames.size(); ++i) {events.push_back({frames[i].first + latency_ns, 1, i});}
    std::stable_sort(events.begin(), events.end(), [](const Event & a, const Event & b) {
        return a.time_ns != b.time_ns ? a.time_ns < b.time_ns : a.order < b.order;});

    std::ofstream journal(out_dir + "/journal.jsonl");
    std::ofstream snapshots(out_dir + "/snapshots.jsonl");
    std::ofstream creations(out_dir + "/creations.jsonl");
    creations << std::setprecision(17);
    std::ofstream registration(out_dir + "/registration.jsonl");
    registration << std::setprecision(17);
    if (!journal || !snapshots) {throw std::runtime_error("cannot write into " + out_dir);}
    journal << std::setprecision(17);
    snapshots << std::setprecision(17);

    bool maps_ready = false, warming_up = true, have_base = false;
    double base_x = 0.0, base_y = 0.0, displacement = 0.0;
    uint64_t frame = 0, observation_id = 0, received = 0, keyframes = 0, warmup_end_frame = 0;
    int64_t warmup_end_stamp = 0, last_received_stamp = 0;
    std::optional<lane_map_apply::Pose> last_pose;
    std::vector<lane_mapping::MapperEvent> mapper_events;
    std::map<std::string, uint64_t> outcomes;

    auto apply_resolved = [&](const lane_mapping::ResolvedObservation & r) {
        // map_fusion_node feed_resolved() -> on_recovered() -> on_vectors().
        const int64_t stamp = r.stamp_ns;
        ++received;
        if (stamp <= last_received_stamp || stamp <= 0) {++outcomes["REJECTED_NONMONOTONIC"]; return;}
        last_received_stamp = stamp;
        std::vector<lane_map_apply::Segment> segments;
        std::vector<uint32_t> tracks;
        for (const auto & o : r.confirmed) {
          const auto a = lane_tracking::metric_to_base(o.p0_m), b = lane_tracking::metric_to_base(o.p1_m);
          segments.push_back({{a.x, a.y}, {b.x, b.y}, o.width, o.confidence, true, false, r.camera_id, stamp, o.track_id});
        }
        const lane_map_apply::Pose here{r.pose.x, r.pose.y, r.pose.yaw};
        std::set<uint32_t> before;
        for (const auto & l : bank.landmarks()) {before.insert(l.id);}
        const auto result = lane_map_apply::apply(bank, apply, received, stamp, here,
            last_pose ? &*last_pose : nullptr, r.camera_id, segments);
        // E-2b: which tracker track each new bank line came from (offline log only).
        for (const auto & l : bank.landmarks()) {
          if (before.count(l.id)) {continue;}
          double best = 1e9;
          uint32_t track = 0;
          for (const size_t i : result.sources) {
            const auto a = lane_map_apply::world(segments[i].a, here), b = lane_map_apply::world(segments[i].b, here);
            const double d = std::hypot(0.5 * (a.x + b.x) - l.center.x, 0.5 * (a.y + b.y) - l.center.y);
            if (d < best) {best = d; track = segments[i].track_id;}
          }
          creations << "{\"stamp_ns\":" << stamp << ",\"landmark\":" << l.id << ",\"track\":" << track
                    << ",\"center\":[" << l.center.x << ',' << l.center.y << "],\"u\":[" << l.u.x << ',' << l.u.y
                    << "],\"source_distance\":" << best << "}\n";
        }
        ++outcomes[result.outcome + ":" + result.reason];
        // E-2b drift table (observer_eval_E2B_drift 3): the bank's registration state after
        // every input that reached it (offline log only).
        if (!result.warmup && !result.motion_skipped) {
          const auto & reg = bank.last;
          const auto c = bank.corrected({0.0, 0.0});
          registration << "{\"stamp_ns\":" << stamp << ",\"correction\":[" << c.x << ',' << c.y << ','
                       << bank.yaw_correction() << "],\"accepted\":" << (reg.accepted ? "true" : "false")
                       << ",\"near\":" << (reg.near ? "true" : "false") << ",\"pairs\":" << reg.pairs
                       << ",\"rank\":" << reg.rank << ",\"rms\":" << reg.rms << ",\"delta\":[" << reg.delta.x << ','
                       << reg.delta.y << ',' << reg.delta.yaw << "],\"held_total\":" << bank.held
                       << ",\"aligned_total\":" << bank.aligned << ",\"created\":" << result.update.created
                       << ",\"merged\":" << result.update.merged << ",\"usable\":" << result.usable_segments
                       << ",\"pose\":[" << here.x << ',' << here.y << ',' << here.theta << "]}\n";
        }
        if (!result.keyframe_created) {return;}
        ++keyframes;
        last_pose = here;
        journal << "{\"stamp_ns\":" << stamp << ",\"pose\":[" << here.x << ',' << here.y << ',' << here.theta
                << "],\"corrected_pose\":[" << bank.corrected({here.x, here.y}).x << ',' << bank.corrected({here.x, here.y}).y
                << ',' << here.theta + bank.yaw_correction() << "],\"local_frame\":\"base_link\",\"map_frame\":\"map\""
                << ",\"merged\":" << result.update.merged << ",\"created\":" << result.update.created
                << ",\"session_id\":\"offline\",\"epoch\":" << r.epoch << ",\"observation_id\":" << r.observation_id
                << ",\"camera_id\":\"" << r.camera_id << "\",\"local_segments\":[";
        bool first = true;
        for (const size_t i : result.sources) {
          const auto & s = segments[i];
          if (!first) {journal << ',';}
          first = false;
          journal << '[' << s.a.x << ',' << s.a.y << ',' << s.b.x << ',' << s.b.y << ',' << s.width << ','
                  << s.confidence << ',' << s.track_id << ']';
        }
        journal << "]}\n";
        const auto rendered = bank.render();
        snapshots << "{\"stamp_ns\":" << stamp << ",\"segments\":[";
        first = true;
        for (const auto & s : rendered.segments) {
          if (!first) {snapshots << ',';}
          first = false;
          snapshots << '[' << s.id << ',' << s.a.x << ',' << s.a.y << ',' << s.b.x << ',' << s.b.y << ','
                    << s.width << ',' << s.confidence << ',' << s.views << ']';
        }
        snapshots << "]}\n";
      };
    auto drain = [&](int64_t now) {
        for (int pass = 0; pass < 64; ++pass) {
          const auto resolved = mapper.drain(now, &mapper_events);
          for (const auto & r : resolved) {apply_resolved(r);}
          if (resolved.empty()) {break;}
        }
      };

    for (const auto & event : events) {
      if (event.order == 0) {
        const auto & pose = poses[event.index].second;
        // global_mapping_node warm-up odom subscription.
        if (!have_base) {base_x = pose.x; base_y = pose.y; have_base = true; displacement = 0.0;}
        if (warming_up || displacement == 0.0) {displacement = std::hypot(pose.x - base_x, pose.y - base_y);}
        mapper.add_pose(pose);
        drain(event.time_ns);
        continue;
      }
      const auto & [stamp, path] = frames[event.index];
      std::ifstream file(path, std::ios::binary);
      std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
      const cv::Mat image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
      if (image.empty()) {++outcomes["FRAME_DECODE_FAILED"]; continue;}
      ++frame;
      warming_up = warmup.update(frame, stamp, displacement);
      if (warmup.ended()) {warmup_end_frame = frame; warmup_end_stamp = stamp;}
      if (!maps_ready) {
        maps_ready = pipeline.initialize_maps(calibration, image.cols, image.rows);
        if (!maps_ready) {throw std::runtime_error("calibration load failed: " + calibration);}
      }
      const bool run = (frame - 1) % std::max(1, params.vectorize_every_n) == 0;
      if (!run || warming_up) {continue;}
      const cv::Mat bev = pipeline.remap_bev(image);
      const lane_detection::Prep prep = pipeline.preprocess(bev);
      int raw = 0;
      double dominant = 0.0, confidence = 0.0;
      const auto observations = pipeline.vectorize(prep, &raw, &dominant, &confidence);
      // global_mapping_node publish_observations() -> map_fusion_node on_observation().
      std::vector<lane_tracking::Observation> handed;
      for (const auto & o : observations) {
        const auto a = lane_tracking::metric_to_base(o.p0_m), b = lane_tracking::metric_to_base(o.p1_m);
        const double ax = a.x, ay = a.y, bx = b.x, by = b.y;   // MarkingSegment fields are float64
        lane_tracking::Observation h;
        h.p0_m = lane_tracking::base_to_metric({static_cast<float>(ax), static_cast<float>(ay)});
        h.p1_m = lane_tracking::base_to_metric({static_cast<float>(bx), static_cast<float>(by)});
        h.family = o.family; h.span = o.span; h.width = o.width; h.confidence = o.confidence;
        handed.push_back(h);
      }
      mapper.add_observation(1, ++observation_id, camera, stamp, event.time_ns, std::move(handed), &mapper_events);
      drain(event.time_ns);
    }
    const int64_t end = events.empty() ? 0 : events.back().time_ns;
    drain(end + 60LL * 1000000000LL);   // let anything that can still resolve resolve
    mapper.shutdown(&mapper_events);

    std::map<std::string, uint64_t> mapper_counts;
    for (const auto & e : mapper_events) {++mapper_counts[e.event];}
    std::ofstream summary(out_dir + "/summary.json");
    summary << "{\"frames\":" << frame << ",\"observations\":" << observation_id << ",\"poses\":" << poses.size()
            << ",\"warmup_end_frame\":" << warmup_end_frame << ",\"warmup_end_stamp_ns\":" << warmup_end_stamp
            << ",\"warmup_reason\":\"" << warmup.reason() << "\",\"applied_inputs\":" << received
            << ",\"keyframes\":" << keyframes << ",\"outcomes\":{";
    bool first = true;
    for (const auto & [k, v] : outcomes) {summary << (first ? "" : ",") << '"' << k << "\":" << v; first = false;}
    summary << "},\"mapper_events\":{";
    first = true;
    for (const auto & [k, v] : mapper_counts) {summary << (first ? "" : ",") << '"' << k << "\":" << v; first = false;}
    summary << "}}\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "lane_offline: " << error.what() << std::endl;
    return 1;
  }
}
