// Recorded input events for the deterministic occupancy replay (occ_offline) and the
// JobBuilder golden (observer_eval_E9j 2-3). An events file (tools/run/occ_offline.py)
// lists, in virtual-time order:
//   <virtual_ns> O <stamp_ns> <x> <y> <qx> <qy> <qz> <qw>      odometry (kinematic_state)
//   <virtual_ns> I <camera> <stamp_ns> <db3 path> <message id>  a compressed image
// Images are read from the measurement bag's sqlite file (read only) and decoded the way
// compressed_image_replay_node does, into the sensor_msgs Image the node would receive.
#ifndef MONOSCALE_OCCUPANCY_GRID_MAP__OCC_EVENTS_HPP_
#define MONOSCALE_OCCUPANCY_GRID_MAP__OCC_EVENTS_HPP_

#include <sqlite3.h>

#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace occ_events
{

struct Event
{
  int64_t virtual_ns = 0;
  char kind = 0;  // 'O' odometry, 'I' image
  std::string camera;
  int64_t stamp_ns = 0;
  double x = 0, y = 0, qx = 0, qy = 0, qz = 0, qw = 1;
  std::string db;
  int64_t message_id = 0;
};

inline std::vector<Event> load(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {throw std::runtime_error("cannot read " + path);}
  std::vector<Event> out;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream f(line);
    Event e;
    f >> e.virtual_ns >> e.kind;
    if (e.kind == 'O') {
      f >> e.stamp_ns >> e.x >> e.y >> e.qx >> e.qy >> e.qz >> e.qw;
    } else if (e.kind == 'I') {
      f >> e.camera >> e.stamp_ns >> e.db >> e.message_id;
    } else {
      throw std::runtime_error("bad event line: " + line);
    }
    if (!f) {throw std::runtime_error("bad event line: " + line);}
    out.push_back(std::move(e));
  }
  return out;
}

// Read-only access to the measurement bags' images.
class BagImages
{
public:
  ~BagImages()
  {
    for (auto & [path, db] : dbs_) {sqlite3_close(db);}
  }

  // The image as compressed_image_replay_node publishes it: decoded unchanged, BGRA
  // dropped to BGR, mono8 or bgr8, the compressed message's header (frame_id replaced
  // by `frame_id` when given, as the node checks it against its parameter).
  sensor_msgs::msg::Image image(const Event & e, const std::string & frame_id)
  {
    sqlite3 * db = open(e.db);
    sqlite3_stmt * stmt = nullptr;
    if (sqlite3_prepare_v2(db, "SELECT data FROM messages WHERE id = ?", -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error(std::string("sqlite: ") + sqlite3_errmsg(db));
    }
    sqlite3_bind_int64(stmt, 1, e.message_id);
    if (sqlite3_step(stmt) != SQLITE_ROW) {
      sqlite3_finalize(stmt);
      throw std::runtime_error("no message " + std::to_string(e.message_id) + " in " + e.db);
    }
    const auto * blob = static_cast<const uint8_t *>(sqlite3_column_blob(stmt, 0));
    const size_t bytes = static_cast<size_t>(sqlite3_column_bytes(stmt, 0));
    rclcpp::SerializedMessage serialized(bytes);
    auto & raw = serialized.get_rcl_serialized_message();
    std::copy(blob, blob + bytes, raw.buffer);
    raw.buffer_length = bytes;
    sqlite3_finalize(stmt);
    sensor_msgs::msg::CompressedImage compressed;
    serializer_.deserialize_message(&serialized, &compressed);

    cv::Mat decoded = cv::imdecode(compressed.data, cv::IMREAD_UNCHANGED);
    if (decoded.empty()) {throw std::runtime_error("cannot decode image " + std::to_string(e.message_id));}
    if (decoded.channels() == 4) {cv::cvtColor(decoded, decoded, cv::COLOR_BGRA2BGR);}
    sensor_msgs::msg::Image image;
    image.header = compressed.header;
    if (!frame_id.empty()) {image.header.frame_id = frame_id;}
    image.height = static_cast<uint32_t>(decoded.rows);
    image.width = static_cast<uint32_t>(decoded.cols);
    image.encoding = decoded.channels() == 1 ?
      sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;
    image.is_bigendian = false;
    image.step = static_cast<uint32_t>(decoded.cols * decoded.elemSize());
    const auto * begin = decoded.ptr<uint8_t>(0);
    image.data.assign(begin, begin + image.step * image.height);
    return image;
  }

private:
  sqlite3 * open(const std::string & path)
  {
    auto it = dbs_.find(path);
    if (it != dbs_.end()) {return it->second;}
    sqlite3 * db = nullptr;
    const std::string uri = "file:" + path + "?mode=ro";
    if (sqlite3_open_v2(uri.c_str(), &db, SQLITE_OPEN_READONLY | SQLITE_OPEN_URI, nullptr) != SQLITE_OK) {
      throw std::runtime_error("cannot open " + path + " read-only");
    }
    dbs_[path] = db;
    return db;
  }

  std::map<std::string, sqlite3 *> dbs_;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer_;
};

}  // namespace occ_events

#endif  // MONOSCALE_OCCUPANCY_GRID_MAP__OCC_EVENTS_HPP_
