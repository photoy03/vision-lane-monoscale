#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <vision_lane_interfaces/msg/marking_segment_array.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "vision_lane_map_fusion/registration.hpp"

using Array = vision_lane_interfaces::msg::MarkingSegmentArray;
using Steady = std::chrono::steady_clock;

class MapFusionNode : public rclcpp::Node {
 public:
  MapFusionNode():Node("lane_map_fusion"),bank_(settings()) {
    input_frame_=declare_parameter<std::string>("input_frame","base_link");
    map_frame_=declare_parameter<std::string>("map_frame","lane_map");
    camera_=declare_parameter<std::string>("camera_id","front");
    cameras_=declare_parameter<std::vector<std::string>>("camera_ids",std::vector<std::string>{"front"});
    warmup_=declare_parameter<int>("warmup_frames",120);
    min_travel_=declare_parameter<double>("keyframe_translation_m",.05);
    min_yaw_=declare_parameter<double>("keyframe_yaw_deg",1.)*pi_/180.;
    max_age_=declare_parameter<double>("max_input_age_sec",.5);
    xmin_=declare_parameter<double>("map_x_min_m",-10.);
    xmax_=declare_parameter<double>("map_x_max_m",20.);
    ymin_=declare_parameter<double>("map_y_min_m",-15.);
    ymax_=declare_parameter<double>("map_y_max_m",15.);
    res_=declare_parameter<double>("map_resolution_m",.05);
    const double hz=declare_parameter<double>("map_publish_hz",5.);
    for(double value:{min_travel_,min_yaw_,max_age_,xmin_,xmax_,ymin_,ymax_,res_,hz})
      if(!std::isfinite(value)) throw std::invalid_argument("nonfinite map parameter");
    if(warmup_<0||min_travel_<=0||min_yaw_<=0||max_age_<=0||res_<=0||
      xmax_<=xmin_||ymax_<=ymin_||hz<=0||hz>20||
      (xmax_-xmin_)/res_>4096||(ymax_-ymin_)/res_>4096)
      throw std::invalid_argument("invalid map bounds or timing parameters");
    height_=static_cast<int>(std::ceil((xmax_-xmin_)/res_));
    width_=static_cast<int>(std::ceil((ymax_-ymin_)/res_));
    const auto journal_path=declare_parameter<std::string>("keyframe_path","");
    if(!journal_path.empty()) {
      if(std::ifstream(journal_path).good()) throw std::runtime_error("keyframe file already exists");
      journal_.open(journal_path);
      if(!journal_) throw std::runtime_error("cannot create keyframe file");
    }
    image_=create_publisher<sensor_msgs::msg::Image>("/parking/cpp/global_marking_map_fused",1);
    vectors_=create_publisher<Array>("/parking/cpp/global_marking_segments_fused",1);
    status_=create_publisher<std_msgs::msg::String>("/parking/cpp/map_fusion/status",10);
    subscription_=create_subscription<Array>(declare_parameter<std::string>("input_topic","/parking/cpp/marking_segments"),
      rclcpp::SensorDataQoS(),[this](Array::ConstSharedPtr m){on_vectors(*m);});
    timer_=create_wall_timer(std::chrono::duration<double>(1./hz),[this]{publish_map();});
    RCLCPP_INFO(get_logger(),"CPU registered line map: lane_map output, no odom/TF writes; confirm=%d keyframes, "
      "angle=%.1fdeg offset=%.3fm. Raw map is unchanged.",bank_.settings().confirm_views,
      bank_.settings().angle_deg,bank_.settings().offset_m);
  }
 private:
  static constexpr double pi_=3.14159265358979323846;
  lane_fusion::Settings settings() {
    lane_fusion::Settings s;
    s.angle_deg=declare_parameter<double>("match_angle_deg",s.angle_deg);
    s.offset_m=declare_parameter<double>("match_offset_m",s.offset_m);
    s.overlap_m=declare_parameter<double>("match_overlap_m",s.overlap_m);
    s.overlap_ratio=declare_parameter<double>("match_overlap_ratio",s.overlap_ratio);
    s.bin_m=declare_parameter<double>("support_bin_m",s.bin_m);
    s.confirm_views=declare_parameter<int>("confirm_keyframes",s.confirm_views);
    s.min_length_m=declare_parameter<double>("min_segment_length_m",s.min_length_m);
    s.max_length_m=declare_parameter<double>("max_segment_length_m",s.max_length_m);
    s.min_confidence=declare_parameter<double>("min_confidence",s.min_confidence);
    s.max_landmarks=declare_parameter<int>("max_landmarks",s.max_landmarks);
    s.tentative_ttl_sec=declare_parameter<double>("tentative_ttl_sec",s.tentative_ttl_sec);
    return s;
  }
  static int64_t ns(const builtin_interfaces::msg::Time& t) {return int64_t(t.sec)*1000000000LL+t.nanosec;}
  static lane_fusion::Point world(lane_fusion::Point p,const geometry_msgs::msg::Pose2D& pose) {
    const double c=std::cos(pose.theta),s=std::sin(pose.theta);
    return {pose.x+c*p.x-s*p.y,pose.y+s*p.x+c*p.y};
  }
  void on_vectors(const Array& m) {
    const auto start=Steady::now();++received_;
    const auto& pose=m.observer_pose;
    const int64_t stamp=ns(m.header.stamp);
    if(stamp<=last_received_stamp_||stamp<=0) {++bad_stamp_;return;}
    if(m.header.frame_id!=input_frame_||std::find(cameras_.begin(),cameras_.end(),m.camera_id)==cameras_.end()||!m.pose_valid||
       !std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.theta)) {++bad_pose_;return;}
    const double age=now().seconds()-static_cast<double>(stamp)*1e-9;
    if(age>max_age_||age<-.1) {++bad_stamp_;return;}
    last_received_stamp_=stamp;
    if(received_<=static_cast<uint64_t>(warmup_)) return;
    if(last_pose_) {
      const double travel=std::hypot(pose.x-last_pose_->x,pose.y-last_pose_->y);
      const double yaw=std::abs(std::remainder(pose.theta-last_pose_->theta,2*pi_));
      if(travel<min_travel_&&yaw<min_yaw_) {++motion_skipped_;return;}
    }
    std::vector<lane_fusion::Observation> observations;
    std::vector<const vision_lane_interfaces::msg::MarkingSegment*> sources;
    for(const auto& s:m.segments) {
      if(observations.size()>=128) {++capacity_;break;}
      if(!s.confirmed||s.reprojected||s.camera_id!=m.camera_id||std::abs(ns(s.source_stamp)-stamp)>1000) {
        ++filtered_segments_;continue;
      }
      const lane_fusion::Point a{s.start.x,s.start.y}, b{s.end.x,s.end.y};
      if(!lane_fusion::finite(a)||!lane_fusion::finite(b)||!std::isfinite(s.mean_width_m)||
         !std::isfinite(s.confidence)) continue;
      auto ga=world(a,pose),gb=world(b,pose);
      observations.push_back({ga,gb,s.mean_width_m,s.confidence});sources.push_back(&s);
    }
    if(observations.empty()) {++empty_frames_;return;}
    const auto result=bank_.update(stamp,observations,{pose.x,pose.y});
    merged_+=result.merged;created_+=result.created;duplicates_+=result.duplicate;
    ambiguous_+=result.ambiguous;invalid_+=result.invalid;capacity_+=result.capacity;
    if(result.merged+result.created==0) return;
    ++keyframes_;last_pose_=pose;last_header_=m.header;last_header_.frame_id=map_frame_;
    // These are temporally filtered local vectors, not raw images or raw Hough.
    if(journal_.is_open() && journal_.good()) {
      journal_<<std::setprecision(17)<<"{\"stamp_ns\":"<<stamp<<",\"pose\":["<<pose.x<<','<<pose.y<<','<<pose.theta
        <<"],\"corrected_pose\":["<<bank_.corrected({pose.x,pose.y}).x<<','<<bank_.corrected({pose.x,pose.y}).y<<','<<pose.theta+bank_.yaw_correction()<<"],\"local_frame\":\""<<input_frame_<<"\",\"map_frame\":\""<<map_frame_
        <<"\",\"merged\":"<<result.merged<<",\"created\":"<<result.created<<",\"local_segments\":[";
      bool first=true;
      for(const auto* s:sources) {
        if(!first) journal_<<',';
        first=false;
        journal_<<'['<<s->start.x<<','<<s->start.y<<','<<s->end.x<<','<<s->end.y<<','
          <<s->mean_width_m<<','<<s->confidence<<','<<s->track_id<<']';
      }
      journal_<<"]}\n";
      if(keyframes_%20==0) journal_.flush();
    }
    fusion_ms_=std::chrono::duration<double,std::milli>(Steady::now()-start).count();
  }
  cv::Point pixel(lane_fusion::Point p) const {
    return {static_cast<int>(std::llround((ymax_-p.y)/res_)),static_cast<int>(std::llround((xmax_-p.x)/res_))};
  }
  void publish_map() {
    const auto start=Steady::now();
    const auto segments=bank_.segments();
    cv::Mat canvas=cv::Mat::zeros(height_,width_,CV_8UC3);
    Array output;output.header=last_header_;output.header.frame_id=map_frame_;
    output.camera_id=camera_;output.pose_valid=last_pose_.has_value();
    if(last_pose_) { output.observer_pose=*last_pose_; auto p=bank_.corrected({last_pose_->x,last_pose_->y}); output.observer_pose.x=p.x; output.observer_pose.y=p.y; output.observer_pose.theta+=bank_.yaw_correction(); }
    for(const auto& s:segments) {
      auto a=pixel(s.a),b=pixel(s.b);
      if(cv::clipLine(cv::Size(width_,height_),a,b)) {
        const int thickness=std::max(1,static_cast<int>(std::llround(s.width/res_)));
        cv::line(canvas,a,b,cv::Scalar(240,240,240),thickness,cv::LINE_AA);
      }
      vision_lane_interfaces::msg::MarkingSegment item;
      item.track_id=s.id;item.camera_id=camera_;item.family="map_line";
      item.start.x=s.a.x;item.start.y=s.a.y;item.end.x=s.b.x;item.end.y=s.b.y;
      item.angle_rad=std::atan2(s.b.y-s.a.y,s.b.x-s.a.x);item.span_m=lane_fusion::norm(s.b-s.a);
      item.mean_width_m=s.width;item.confidence=s.confidence;item.hits_total=s.views;
      item.confirmed=true;item.reprojected=false;
      item.source_stamp.sec=static_cast<int32_t>(s.stamp_ns/1000000000LL);
      item.source_stamp.nanosec=static_cast<uint32_t>(s.stamp_ns%1000000000LL);
      output.segments.push_back(item);
    }
    sensor_msgs::msg::Image image;
    image.header=output.header;image.height=height_;image.width=width_;image.encoding="bgr8";
    image.step=static_cast<uint32_t>(width_*3);image.is_bigendian=false;
    image.data.assign(canvas.data,canvas.data+canvas.total()*canvas.elemSize());
    image_->publish(image);vectors_->publish(output);
    const double render_ms=std::chrono::duration<double,std::milli>(Steady::now()-start).count();
    std::ostringstream out;
    out<<"{\"received\":"<<received_<<",\"keyframes\":"<<keyframes_
      <<",\"landmarks\":"<<bank_.landmarks().size()<<",\"rendered_segments\":"<<segments.size()
      <<",\"merged\":"<<merged_<<",\"created\":"<<created_<<",\"same_frame_duplicates\":"<<duplicates_
      <<",\"ambiguous_skipped\":"<<ambiguous_<<",\"invalid_segments\":"<<invalid_
      <<",\"capacity_skipped\":"<<capacity_<<",\"bad_pose\":"<<bad_pose_<<",\"bad_stamp\":"<<bad_stamp_
      <<",\"warming_up\":"<<(received_<=static_cast<uint64_t>(warmup_)?"true":"false")
      <<",\"motion_skipped\":"<<motion_skipped_<<",\"filtered_segments\":"<<filtered_segments_
      <<",\"empty_frames\":"<<empty_frames_
      <<",\"last_map_stamp_ns\":"<<ns(last_header_.stamp)<<",\"fusion_last_ms\":"<<fusion_ms_
      <<",\"render_publish_last_ms\":"<<render_ms<<",\"aligned_frames\":"<<bank_.aligned<<",\"held_frames\":"<<bank_.held<<",\"registration_rank\":"<<bank_.last.rank<<",\"pose_correction\":true}";
    std_msgs::msg::String state;state.data=out.str();status_->publish(state);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),3000,"LINE MAP: keyframes=%lu landmarks=%zu visible=%zu merged=%lu "
      "ambiguous=%lu invalid_pose/stamp=%lu/%lu; odom unchanged",keyframes_,bank_.landmarks().size(),
      segments.size(),merged_,ambiguous_,bad_pose_,bad_stamp_);
  }
  lane_fusion::RegisteredBank bank_;
  std::string input_frame_,map_frame_,camera_;
  std::vector<std::string> cameras_;
  int warmup_{120},width_{0},height_{0};
  double min_travel_{0},min_yaw_{0},max_age_{0},xmin_{0},xmax_{0},ymin_{0},ymax_{0},res_{0},fusion_ms_{0};
  uint64_t received_{0},keyframes_{0},merged_{0},created_{0},duplicates_{0},ambiguous_{0},invalid_{0},capacity_{0},bad_pose_{0},bad_stamp_{0};
  uint64_t motion_skipped_{0},filtered_segments_{0},empty_frames_{0};
  int64_t last_received_stamp_{0};std_msgs::msg::Header last_header_;
  std::optional<geometry_msgs::msg::Pose2D> last_pose_;
  std::ofstream journal_;
  rclcpp::Subscription<Array>::SharedPtr subscription_;
  rclcpp::Publisher<Array>::SharedPtr vectors_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MapFusionNode>());
  rclcpp::shutdown();return 0;
}
