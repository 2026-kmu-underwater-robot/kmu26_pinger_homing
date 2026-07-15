#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <exception>
#include <fstream>
#include <iomanip>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "hit25_auv_ros2_msg/msg/buoy_observation.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Rgba
{
  float r{1.0F};
  float g{1.0F};
  float b{1.0F};
  float a{1.0F};
};

struct RobotPose
{
  Vec3 xyz;
  double yaw{0.0};
};

struct CommandState
{
  double forward{0.0};
  double sway{0.0};
  double heave{0.0};
  double yaw{0.0};
  double pitch{0.0};
  std::string phase;
};

struct DetectionState
{
  std::string buoy_id;
  std::string class_name;
  Vec3 p_intake;
  Vec3 target_xyz;
  double distance{0.0};
  double confidence{0.0};
  double bearing{0.0};
  bool has_target_xyz{false};
  bool range_estimated{false};
};

struct YoloDetectionState
{
  std::string label;
  double confidence{0.0};
  std::vector<double> xyxy_norm;
  std::vector<double> center_norm;
};

struct YoloStatus
{
  bool valid{false};
  bool active{false};
  bool model_found{false};
  std::string error;
  int count{0};
  int frame_width{0};
  int frame_height{0};
  std::vector<YoloDetectionState> detections;
};

struct BuoyState
{
  std::string id;
  std::string class_name;
  std::string course;
  std::string state;
  Vec3 xyz;
  Vec3 target_xyz;
  Vec3 attach_xyz;
  bool has_xyz{false};
  bool has_target_xyz{false};
  bool has_attach_xyz{false};
  bool processed{false};
  bool failed{false};
  double release_margin{0.0};
};

struct MissionStatus
{
  bool valid{false};
  std::string state{"NO_STATUS"};
  std::string robot_state;
  std::string robot_state_label;
  std::string mode;
  std::string target_id;
  std::string target_class;
  std::string own_course{"a"};
  std::optional<RobotPose> robot;
  std::optional<CommandState> command;
  std::optional<DetectionState> detection;
  std::optional<Vec3> surface_collector_xyz;
  std::optional<Vec3> score_zone_xyz;
  std::string score_zone_id;
  double score_zone_radius{0.85};
  double boundary_x{0.0};
  double boundary_margin{0.8};
  double boundary_standoff{0.7};
  int remaining_attached{0};
  int processed_count{0};
  int failed_count{0};
  int collected_count{0};
  int scored_count{0};
  std::vector<BuoyState> buoys;
};

std::string read_text_file(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    return "";
  }
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

std::optional<size_t> find_json_key(const std::string & text, const std::string & key)
{
  const std::string token = "\"" + key + "\"";
  const size_t pos = text.find(token);
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  return pos;
}

size_t skip_ws(const std::string & text, size_t pos)
{
  while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) {
    ++pos;
  }
  return pos;
}

std::optional<std::string> balanced_block_at(
  const std::string & text, size_t start, char open_char, char close_char)
{
  if (start >= text.size() || text[start] != open_char) {
    return std::nullopt;
  }
  bool in_string = false;
  bool escaped = false;
  int depth = 0;
  for (size_t i = start; i < text.size(); ++i) {
    const char c = text[i];
    if (in_string) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        in_string = false;
      }
      continue;
    }
    if (c == '"') {
      in_string = true;
      continue;
    }
    if (c == open_char) {
      ++depth;
    } else if (c == close_char) {
      --depth;
      if (depth == 0) {
        return text.substr(start, i - start + 1);
      }
    }
  }
  return std::nullopt;
}

std::optional<std::string> object_after_key(const std::string & text, const std::string & key)
{
  const auto key_pos = find_json_key(text, key);
  if (!key_pos) {
    return std::nullopt;
  }
  const size_t colon = text.find(':', *key_pos);
  if (colon == std::string::npos) {
    return std::nullopt;
  }
  const size_t value_pos = skip_ws(text, colon + 1);
  return balanced_block_at(text, value_pos, '{', '}');
}

std::optional<std::string> array_after_key(const std::string & text, const std::string & key)
{
  const auto key_pos = find_json_key(text, key);
  if (!key_pos) {
    return std::nullopt;
  }
  const size_t colon = text.find(':', *key_pos);
  if (colon == std::string::npos) {
    return std::nullopt;
  }
  const size_t value_pos = skip_ws(text, colon + 1);
  return balanced_block_at(text, value_pos, '[', ']');
}

std::vector<std::string> object_array_after_key(const std::string & text, const std::string & key)
{
  std::vector<std::string> objects;
  const auto array_text = array_after_key(text, key);
  if (!array_text) {
    return objects;
  }
  bool in_string = false;
  bool escaped = false;
  int object_depth = 0;
  size_t object_start = 0;
  for (size_t i = 0; i < array_text->size(); ++i) {
    const char c = (*array_text)[i];
    if (in_string) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        in_string = false;
      }
      continue;
    }
    if (c == '"') {
      in_string = true;
      continue;
    }
    if (c == '{') {
      if (object_depth == 0) {
        object_start = i;
      }
      ++object_depth;
    } else if (c == '}') {
      --object_depth;
      if (object_depth == 0) {
        objects.push_back(array_text->substr(object_start, i - object_start + 1));
      }
    }
  }
  return objects;
}

std::optional<std::string> json_string(const std::string & text, const std::string & key)
{
  const std::regex re("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
  std::smatch match;
  if (std::regex_search(text, match, re)) {
    return match[1].str();
  }
  return std::nullopt;
}

std::optional<double> json_number(const std::string & text, const std::string & key)
{
  static const std::string number = "(-?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][+-]?\\d+)?)";
  const std::regex re("\"" + key + "\"\\s*:\\s*" + number);
  std::smatch match;
  if (std::regex_search(text, match, re)) {
    try {
      return std::stod(match[1].str());
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<bool> json_bool(const std::string & text, const std::string & key)
{
  const std::regex re("\"" + key + "\"\\s*:\\s*(true|false)");
  std::smatch match;
  if (std::regex_search(text, match, re)) {
    return match[1].str() == "true";
  }
  return std::nullopt;
}

std::optional<Vec3> json_vec3(const std::string & text, const std::string & key)
{
  static const std::string number = "(-?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][+-]?\\d+)?)";
  const std::regex re(
    "\"" + key + "\"\\s*:\\s*\\[\\s*" + number + "\\s*,\\s*" + number + "\\s*,\\s*" +
    number + "\\s*\\]");
  std::smatch match;
  if (!std::regex_search(text, match, re)) {
    return std::nullopt;
  }
  try {
    return Vec3{std::stod(match[1].str()), std::stod(match[2].str()), std::stod(match[3].str())};
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::vector<double> json_number_array(const std::string & text, const std::string & key)
{
  std::vector<double> values;
  const auto array = array_after_key(text, key);
  if (!array) {
    return values;
  }
  static const std::regex re("-?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][+-]?\\d+)?");
  for (auto it = std::sregex_iterator(array->begin(), array->end(), re);
       it != std::sregex_iterator(); ++it) {
    try {
      values.push_back(std::stod(it->str()));
    } catch (const std::exception &) {
      values.clear();
      return values;
    }
  }
  return values;
}

std::string trim_id(const std::string & id)
{
  constexpr size_t max_len = 34;
  if (id.size() <= max_len) {
    return id;
  }
  return id.substr(0, max_len - 3) + "...";
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

Vec3 rotate_body_to_world(const Vec3 & body, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Vec3{c * body.x - s * body.y, s * body.x + c * body.y, body.z};
}

Vec3 add(const Vec3 & a, const Vec3 & b)
{
  return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

std::string lower_ascii(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

Rgba color_for_class(const std::string & class_name, float alpha = 1.0F)
{
  const std::string label = lower_ascii(class_name);
  if (label.find("red") != std::string::npos) {
    return Rgba{1.0F, 0.05F, 0.03F, alpha};
  }
  if (label.find("yellow") != std::string::npos) {
    return Rgba{1.0F, 0.84F, 0.04F, alpha};
  }
  if (label.find("orange") != std::string::npos) {
    return Rgba{1.0F, 0.43F, 0.02F, alpha};
  }
  if (label.find("white") != std::string::npos || label.find("pinger") != std::string::npos) {
    return Rgba{0.95F, 0.97F, 1.0F, alpha};
  }
  return Rgba{0.55F, 0.75F, 1.0F, alpha};
}

bool is_terminal_buoy_state(const std::string & state)
{
  return state == "SCORED" || state == "COLLECTED" || state == "FAILED";
}

geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::msg::Point point(const Vec3 & v)
{
  return point(v.x, v.y, v.z);
}

void set_color(visualization_msgs::msg::Marker & marker, const Rgba & color)
{
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = color.a;
}

}  // namespace

class MissionRvizVisualizer : public rclcpp::Node
{
public:
  MissionRvizVisualizer()
  : Node("mission_rviz_visualizer")
  {
    status_path_ = declare_parameter<std::string>(
      "mission_status_json",
      "/tmp/kmu26_mission_fsm_status.json");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/mission/rviz_markers");
    marker_frame_ = declare_parameter<std::string>("marker_frame", "map");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/mujoco/ground_truth/pose");
    pose_type_ = lower_ascii(declare_parameter<std::string>("pose_type", "pose_stamped"));
    if (pose_type_ == "odom") {
      pose_type_ = "odometry";
    }
    if (pose_type_ != "odometry" && pose_type_ != "pose_stamped") {
      pose_type_ = "pose_stamped";
    }
    yolo_detection_topic_ =
      declare_parameter<std::string>("yolo_detection_topic", "/uuv_mujoco/yolo_buoy_detections");
    observation_topic_ =
      declare_parameter<std::string>("observation_topic", "/vision/buoy_observation");
    fsm_status_topic_ =
      declare_parameter<std::string>("fsm_status_topic", "/mission/fsm/status");
    hydrophone_body_topic_ = declare_parameter<std::string>(
      "hydrophone_body_topic", "/mission/hydrophone/direction_body");
    default_own_course_ = lower_ascii(declare_parameter<std::string>("own_course", "a"));
    default_boundary_x_ = declare_parameter<double>("course_boundary_x_m", 0.0);
    default_boundary_margin_ = declare_parameter<double>("course_boundary_margin_m", 0.8);
    default_boundary_standoff_ = declare_parameter<double>("course_boundary_standoff_m", 0.7);
    yolo_zone_gate_enabled_ = declare_parameter<bool>("yolo_zone_gate_enabled", true);
    yolo_zone_gate_crossing_range_m_ =
      declare_parameter<double>("yolo_zone_gate_crossing_range_m", 8.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    x_min_ = declare_parameter<double>("course_x_min_m", -15.5);
    x_max_ = declare_parameter<double>("course_x_max_m", 15.5);
    y_min_ = declare_parameter<double>("course_y_min_m", -15.5);
    y_max_ = declare_parameter<double>("course_y_max_m", 15.5);
    z_min_ = declare_parameter<double>("course_z_min_m", -9.5);
    z_max_ = declare_parameter<double>("course_z_max_m", 0.8);

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1).transient_local());
    if (pose_type_ == "odometry") {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        pose_topic_, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          RobotPose pose;
          pose.xyz = Vec3{
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z};
          pose.yaw = yaw_from_quaternion(msg->pose.pose.orientation);
          last_pose_ = pose;
        });
    } else {
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          RobotPose pose;
          pose.xyz = Vec3{
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z};
          pose.yaw = yaw_from_quaternion(msg->pose.orientation);
          last_pose_ = pose;
        });
    }
    const auto telemetry_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    yolo_sub_ = create_subscription<std_msgs::msg::String>(
      yolo_detection_topic_, telemetry_qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        latest_yolo_text_ = msg->data;
        latest_yolo_wall_s_ = now().seconds();
      });
    observation_sub_ = create_subscription<hit25_auv_ros2_msg::msg::BuoyObservation>(
      observation_topic_, rclcpp::SensorDataQoS(),
      [this](hit25_auv_ros2_msg::msg::BuoyObservation::SharedPtr msg) {
        latest_observation_ = *msg;
        latest_observation_wall_s_ = now().seconds();
      });
    fsm_status_sub_ = create_subscription<std_msgs::msg::String>(
      fsm_status_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::String::SharedPtr msg) {
        latest_fsm_status_text_ = msg->data;
        latest_fsm_status_wall_s_ = now().seconds();
      });
    hydrophone_body_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      hydrophone_body_topic_, telemetry_qos,
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        const double norm = std::sqrt(
          msg->vector.x * msg->vector.x + msg->vector.y * msg->vector.y +
          msg->vector.z * msg->vector.z);
        if (!std::isfinite(norm) || norm < 1.0e-6) return;
        latest_hydrophone_body_ = Vec3{
          msg->vector.x / norm, msg->vector.y / norm, msg->vector.z / norm};
        latest_hydrophone_wall_s_ = now().seconds();
      });

    const double safe_rate = std::clamp(publish_rate_hz_, 1.0, 30.0);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / safe_rate)),
      [this]() { publish_markers(); });

    RCLCPP_INFO(
      get_logger(),
      "mission RViz markers: topic=%s status=%s frame=%s pose=%s(%s) observation=%s fsm=%s",
      marker_topic_.c_str(), status_path_.c_str(), marker_frame_.c_str(),
      pose_topic_.c_str(), pose_type_.c_str(),
      observation_topic_.c_str(), fsm_status_topic_.c_str());
  }

private:
  MissionStatus parse_status(const std::string & text) const
  {
    MissionStatus status;
    status.own_course = default_own_course_ == "b" ? "b" : "a";
    status.boundary_x = default_boundary_x_;
    status.boundary_margin = default_boundary_margin_;
    status.boundary_standoff = default_boundary_standoff_;
    if (text.empty()) {
      return status;
    }
    status.valid = true;
    status.state = json_string(text, "state").value_or("NO_STATUS");
    status.robot_state = json_string(text, "robot_state").value_or("");
    status.robot_state_label = json_string(text, "robot_state_label").value_or(status.robot_state);
    status.mode = json_string(text, "mode").value_or("");
    status.target_id = json_string(text, "target_id").value_or("");
    status.target_class = json_string(text, "target_class").value_or(
      json_string(text, "target_label").value_or(""));
    status.remaining_attached = static_cast<int>(json_number(text, "remaining_attached").value_or(0.0));
    status.processed_count = static_cast<int>(json_number(text, "processed_count").value_or(0.0));
    status.failed_count = static_cast<int>(json_number(text, "failed_count").value_or(0.0));
    status.collected_count = static_cast<int>(json_number(text, "collected_count").value_or(0.0));
    status.scored_count = static_cast<int>(json_number(text, "scored_count").value_or(0.0));

    if (const auto policy = object_after_key(text, "mission_policy")) {
      status.own_course = json_string(*policy, "own_course").value_or(status.own_course);
      status.boundary_x = json_number(*policy, "course_boundary_x_m").value_or(status.boundary_x);
      status.boundary_margin =
        json_number(*policy, "course_boundary_margin_m").value_or(status.boundary_margin);
      status.boundary_standoff =
        json_number(*policy, "course_boundary_standoff_m").value_or(status.boundary_standoff);
    }

    if (const auto score = object_after_key(text, "score_zone")) {
      status.score_zone_id = json_string(*score, "id").value_or("");
      status.score_zone_xyz = json_vec3(*score, "xyz");
      status.score_zone_radius =
        json_number(*score, "radius_m").value_or(status.score_zone_radius);
    }

    if (const auto surface = object_after_key(text, "surface_collection")) {
      status.surface_collector_xyz = json_vec3(*surface, "collector_xyz");
    }

    if (const auto robot = object_after_key(text, "robot")) {
      RobotPose pose;
      pose.xyz.x = json_number(*robot, "x").value_or(0.0);
      pose.xyz.y = json_number(*robot, "y").value_or(0.0);
      pose.xyz.z = json_number(*robot, "z").value_or(0.0);
      pose.yaw = json_number(*robot, "yaw_rad").value_or(0.0);
      status.robot = pose;
    }

    if (const auto command = object_after_key(text, "command")) {
      CommandState cmd;
      cmd.forward = json_number(*command, "forward").value_or(0.0);
      cmd.sway = json_number(*command, "sway").value_or(0.0);
      cmd.heave = json_number(*command, "heave").value_or(0.0);
      cmd.yaw = json_number(*command, "yaw").value_or(0.0);
      cmd.pitch = json_number(*command, "pitch").value_or(0.0);
      cmd.phase = json_string(*command, "phase").value_or("");
      status.command = cmd;
    }

    if (const auto detection = object_after_key(text, "detection")) {
      DetectionState det;
      det.buoy_id = json_string(*detection, "buoy_id").value_or("");
      det.class_name = json_string(*detection, "class_name").value_or("");
      if (const auto p = json_vec3(*detection, "p_intake")) {
        det.p_intake = *p;
      }
      if (const auto target = json_vec3(*detection, "target_xyz")) {
        det.target_xyz = *target;
        det.has_target_xyz = true;
      }
      det.distance = json_number(*detection, "distance_m").value_or(0.0);
      det.confidence = json_number(*detection, "confidence").value_or(0.0);
      det.bearing = json_number(*detection, "bearing_rad").value_or(0.0);
      status.detection = det;
    }

    for (const auto & object : object_array_after_key(text, "buoys")) {
      BuoyState buoy;
      buoy.id = json_string(object, "id").value_or("");
      buoy.class_name = json_string(object, "class_name").value_or("");
      buoy.course = json_string(object, "course").value_or("");
      buoy.state = json_string(object, "state").value_or("");
      if (const auto xyz = json_vec3(object, "xyz")) {
        buoy.xyz = *xyz;
        buoy.has_xyz = true;
      }
      if (const auto target = json_vec3(object, "target_xyz")) {
        buoy.target_xyz = *target;
        buoy.has_target_xyz = true;
      }
      if (const auto attach = json_vec3(object, "attach_xyz")) {
        buoy.attach_xyz = *attach;
        buoy.has_attach_xyz = true;
      }
      buoy.processed = json_bool(object, "processed").value_or(false);
      buoy.failed = json_bool(object, "failed").value_or(false);
      buoy.release_margin = json_number(object, "probe_release_margin_m").value_or(0.0);
      status.buoys.push_back(std::move(buoy));
    }
    return status;
  }

  YoloStatus parse_yolo_status(const std::string & text) const
  {
    YoloStatus status;
    if (text.empty()) {
      return status;
    }
    status.valid = true;
    status.active = json_bool(text, "active").value_or(false);
    status.model_found = json_bool(text, "model_found").value_or(false);
    status.error = json_string(text, "error").value_or("");
    status.count = static_cast<int>(json_number(text, "count").value_or(0.0));
    status.frame_width = static_cast<int>(json_number(text, "frame_width").value_or(0.0));
    status.frame_height = static_cast<int>(json_number(text, "frame_height").value_or(0.0));
    for (const auto & object : object_array_after_key(text, "detections")) {
      YoloDetectionState det;
      det.label = json_string(object, "label").value_or("buoy");
      det.confidence = json_number(object, "confidence").value_or(0.0);
      det.xyxy_norm = json_number_array(object, "xyxy_norm");
      det.center_norm = json_number_array(object, "center_norm");
      if (det.center_norm.size() < 2 && det.xyxy_norm.size() >= 4) {
        det.center_norm = {
          0.5 * (det.xyxy_norm[0] + det.xyxy_norm[2]),
          0.5 * (det.xyxy_norm[1] + det.xyxy_norm[3])};
      }
      status.detections.push_back(std::move(det));
    }
    return status;
  }

  YoloStatus yolo_from_observation(
    const hit25_auv_ros2_msg::msg::BuoyObservation & observation) const
  {
    YoloStatus status;
    status.valid = true;
    status.active = true;
    status.model_found = true;
    status.frame_width = static_cast<int>(observation.image_width);
    status.frame_height = static_cast<int>(observation.image_height);
    if (!observation.detected || observation.image_width == 0 || observation.image_height == 0) {
      return status;
    }
    const double width = static_cast<double>(observation.image_width);
    const double height = static_cast<double>(observation.image_height);
    const double cx = observation.bbox_center_x / width;
    const double cy = observation.bbox_center_y / height;
    const double bw = observation.bbox_width / width;
    const double bh = observation.bbox_height / height;
    YoloDetectionState detection;
    detection.label = observation.class_label.empty() ? "buoy" : observation.class_label;
    detection.confidence = observation.confidence;
    detection.center_norm = {cx, cy};
    detection.xyxy_norm = {
      cx - 0.5 * bw, cy - 0.5 * bh, cx + 0.5 * bw, cy + 0.5 * bh};
    status.detections.push_back(std::move(detection));
    status.count = 1;
    return status;
  }

  DetectionState detection_from_observation(
    const hit25_auv_ros2_msg::msg::BuoyObservation & observation) const
  {
    DetectionState detection;
    detection.buoy_id = "camera_target";
    detection.class_name = observation.class_label.empty() ? "buoy" : observation.class_label;
    detection.confidence = observation.confidence;
    detection.bearing = observation.bearing_rad;
    if (observation.range_valid && std::isfinite(observation.range_m) && observation.range_m > 0.0) {
      detection.distance = observation.range_m;
    } else {
      const double height_ratio = observation.image_height > 0 ?
        observation.bbox_height / static_cast<double>(observation.image_height) : 0.0;
      detection.distance = std::clamp(0.55 / std::max(height_ratio, 0.04), 0.6, 8.0);
      detection.range_estimated = true;
    }
    detection.p_intake = Vec3{
      detection.distance * std::cos(detection.bearing),
      -detection.distance * std::sin(detection.bearing), 0.0};
    return detection;
  }

  visualization_msgs::msg::Marker base_marker(
    const std::string & ns, int id, int type, const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = marker_frame_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime.sec = 1;
    marker.lifetime.nanosec = 0;
    return marker;
  }

  void add_delete_all(visualization_msgs::msg::MarkerArray & array, const rclcpp::Time & stamp) const
  {
    auto marker = base_marker("mission_clear", 0, visualization_msgs::msg::Marker::CUBE, stamp);
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(std::move(marker));
  }

  void add_text(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const std::string & ns, const Vec3 & xyz, const std::string & text, double size,
    const Rgba & color) const
  {
    auto marker = base_marker(ns, id++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, stamp);
    marker.pose.position = point(xyz);
    marker.scale.z = size;
    marker.text = text;
    set_color(marker, color);
    array.markers.push_back(std::move(marker));
  }

  void add_line_strip(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const std::string & ns, const std::vector<Vec3> & points, double width,
    const Rgba & color) const
  {
    auto marker = base_marker(ns, id++, visualization_msgs::msg::Marker::LINE_STRIP, stamp);
    marker.scale.x = width;
    set_color(marker, color);
    for (const auto & p : points) {
      marker.points.push_back(point(p));
    }
    array.markers.push_back(std::move(marker));
  }

  void add_line_list(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const std::string & ns, const std::vector<Vec3> & points, double width,
    const Rgba & color) const
  {
    auto marker = base_marker(ns, id++, visualization_msgs::msg::Marker::LINE_LIST, stamp);
    marker.scale.x = width;
    set_color(marker, color);
    for (const auto & p : points) {
      marker.points.push_back(point(p));
    }
    array.markers.push_back(std::move(marker));
  }

  void add_sphere(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const std::string & ns, const Vec3 & xyz, double diameter, const Rgba & color) const
  {
    auto marker = base_marker(ns, id++, visualization_msgs::msg::Marker::SPHERE, stamp);
    marker.pose.position = point(xyz);
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    set_color(marker, color);
    array.markers.push_back(std::move(marker));
  }

  void add_cylinder(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const std::string & ns, const Vec3 & xyz, double diameter, double height,
    const Rgba & color) const
  {
    auto marker = base_marker(ns, id++, visualization_msgs::msg::Marker::CYLINDER, stamp);
    marker.pose.position = point(xyz);
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = height;
    set_color(marker, color);
    array.markers.push_back(std::move(marker));
  }

  void add_cube(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const std::string & ns, const Vec3 & xyz, const Vec3 & scale, const Rgba & color) const
  {
    auto marker = base_marker(ns, id++, visualization_msgs::msg::Marker::CUBE, stamp);
    marker.pose.position = point(xyz);
    marker.scale.x = scale.x;
    marker.scale.y = scale.y;
    marker.scale.z = scale.z;
    set_color(marker, color);
    array.markers.push_back(std::move(marker));
  }

  void add_arrow(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const std::string & ns, const Vec3 & start, const Vec3 & end, double shaft,
    double head, const Rgba & color) const
  {
    auto marker = base_marker(ns, id++, visualization_msgs::msg::Marker::ARROW, stamp);
    marker.points.push_back(point(start));
    marker.points.push_back(point(end));
    marker.scale.x = shaft;
    marker.scale.y = head;
    marker.scale.z = head;
    set_color(marker, color);
    array.markers.push_back(std::move(marker));
  }

  void add_course_regions(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const MissionStatus & status) const
  {
    const double bx = status.boundary_x;
    const double a_center = 0.5 * (x_min_ + bx);
    const double b_center = 0.5 * (bx + x_max_);
    const double a_width = std::max(0.1, std::abs(bx - x_min_));
    const double b_width = std::max(0.1, std::abs(x_max_ - bx));
    const bool own_is_a = status.own_course != "b";
    const Rgba own{0.05F, 0.85F, 0.40F, 0.12F};
    const Rgba other{1.0F, 0.12F, 0.08F, 0.08F};
    add_cube(
      array, id, stamp, "mission_zone", Vec3{a_center, 0.0, 0.03},
      Vec3{a_width, y_max_ - y_min_, 0.04}, own_is_a ? own : other);
    add_cube(
      array, id, stamp, "mission_zone", Vec3{b_center, 0.0, 0.04},
      Vec3{b_width, y_max_ - y_min_, 0.04}, own_is_a ? other : own);

    add_line_strip(
      array, id, stamp, "mission_boundary",
      {Vec3{bx, y_min_, 0.08}, Vec3{bx, y_max_, 0.08}}, 0.08,
      Rgba{0.0F, 0.95F, 1.0F, 1.0F});
    add_line_strip(
      array, id, stamp, "mission_boundary",
      {Vec3{bx - status.boundary_margin, y_min_, 0.05},
       Vec3{bx - status.boundary_margin, y_max_, 0.05}},
      0.035, Rgba{1.0F, 0.82F, 0.0F, 0.85F});
    add_line_strip(
      array, id, stamp, "mission_boundary",
      {Vec3{bx + status.boundary_margin, y_min_, 0.05},
       Vec3{bx + status.boundary_margin, y_max_, 0.05}},
      0.035, Rgba{1.0F, 0.82F, 0.0F, 0.85F});
    add_line_list(
      array, id, stamp, "mission_boundary_posts",
      {Vec3{bx, y_min_, z_min_}, Vec3{bx, y_min_, z_max_},
       Vec3{bx, y_max_, z_min_}, Vec3{bx, y_max_, z_max_}},
      0.035, Rgba{0.0F, 0.95F, 1.0F, 0.75F});
    add_text(
      array, id, stamp, "mission_zone_text", Vec3{a_center, y_min_ + 1.2, 0.55},
      own_is_a ? "OWN ZONE A" : "OPPONENT ZONE A", 0.35,
      own_is_a ? Rgba{0.2F, 1.0F, 0.55F, 1.0F} : Rgba{1.0F, 0.35F, 0.25F, 1.0F});
    add_text(
      array, id, stamp, "mission_zone_text", Vec3{b_center, y_min_ + 1.2, 0.55},
      own_is_a ? "OPPONENT ZONE B" : "OWN ZONE B", 0.35,
      own_is_a ? Rgba{1.0F, 0.35F, 0.25F, 1.0F} : Rgba{0.2F, 1.0F, 0.55F, 1.0F});
  }

  void add_score_zone(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const MissionStatus & status) const
  {
    if (!status.score_zone_xyz) {
      return;
    }
    const Vec3 z = *status.score_zone_xyz;
    add_cylinder(
      array, id, stamp, "mission_score_zone", z, status.score_zone_radius * 2.0, 0.12,
      Rgba{0.2F, 0.55F, 1.0F, 0.30F});
    const double r = status.score_zone_radius;
    std::vector<Vec3> ring;
    for (int i = 0; i <= 48; ++i) {
      const double a = 2.0 * kPi * static_cast<double>(i) / 48.0;
      ring.push_back(Vec3{z.x + std::cos(a) * r, z.y + std::sin(a) * r, z.z + 0.1});
    }
    add_line_strip(array, id, stamp, "mission_score_zone_ring", ring, 0.04, Rgba{0.35F, 0.7F, 1.0F, 1.0F});
    add_text(
      array, id, stamp, "mission_score_zone_text", Vec3{z.x, z.y, z.z + 0.7},
      status.score_zone_id.empty() ? "SCORE ZONE" : status.score_zone_id, 0.32,
      Rgba{0.55F, 0.85F, 1.0F, 1.0F});
  }

  void add_surface_collector(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const MissionStatus & status) const
  {
    if (!status.surface_collector_xyz) {
      return;
    }
    const Vec3 c = *status.surface_collector_xyz;
    add_sphere(array, id, stamp, "mission_surface_collector", c, 0.28, Rgba{0.0F, 0.95F, 1.0F, 1.0F});
    add_cube(
      array, id, stamp, "mission_surface_collector_box", c, Vec3{0.68, 0.48, 0.60},
      Rgba{0.0F, 0.85F, 1.0F, 0.16F});
    add_text(
      array, id, stamp, "mission_surface_collector_text", Vec3{c.x, c.y, c.z + 0.55},
      "COLLECTOR\nphysical pocket", 0.20, Rgba{0.55F, 1.0F, 1.0F, 0.95F});
  }

  void add_buoys(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const MissionStatus & status) const
  {
    for (const auto & buoy : status.buoys) {
      if (!buoy.has_xyz) {
        continue;
      }
      const bool target = !status.target_id.empty() && buoy.id == status.target_id;
      const bool terminal = is_terminal_buoy_state(buoy.state);
      float alpha = buoy.failed ? 0.35F : (terminal ? 0.55F : 0.92F);
      if (target) {
        alpha = 1.0F;
      }
      Rgba color = color_for_class(buoy.class_name, alpha);
      const double diameter = target ? 0.68 : (terminal ? 0.38 : 0.48);
      add_sphere(array, id, stamp, "mission_buoys", buoy.xyz, diameter, color);

      if (target) {
        add_cylinder(
          array, id, stamp, "mission_target_column", Vec3{buoy.xyz.x, buoy.xyz.y, buoy.xyz.z + 0.65},
          0.28, 1.1, Rgba{0.1F, 1.0F, 0.9F, 0.45F});
      }

      if (buoy.has_attach_xyz && !terminal) {
        add_line_list(
          array, id, stamp, "mission_buoy_attach",
          {buoy.attach_xyz, buoy.xyz}, 0.025, Rgba{0.35F, 0.35F, 1.0F, 0.65F});
      }

      if (target || !terminal || buoy.failed) {
        std::ostringstream label;
        label << buoy.class_name << " " << buoy.course << "\n" << buoy.state;
        if (target) {
          label << "\nTARGET";
        }
        add_text(
          array, id, stamp, "mission_buoy_labels",
          Vec3{buoy.xyz.x, buoy.xyz.y, buoy.xyz.z + (target ? 1.0 : 0.65)}, label.str(),
          target ? 0.28 : 0.20, target ? Rgba{0.55F, 1.0F, 0.95F, 1.0F} : Rgba{0.9F, 0.92F, 0.95F, 0.85F});
      }
    }
  }

  void add_search_and_detection(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const MissionStatus & status, const RobotPose & pose) const
  {
    const bool detected = status.detection.has_value();
    const bool searching =
      status.robot_state == "BUOY_SEARCHING" || status.state.find("SEARCH") != std::string::npos;
    const double range = detected ? std::max(2.0, status.detection->distance) : 7.0;
    const double fov = 55.0 * kPi / 180.0;
    const Vec3 origin = Vec3{pose.xyz.x, pose.xyz.y, pose.xyz.z + 0.12};
    std::vector<Vec3> fan_points;
    fan_points.push_back(origin);
    fan_points.push_back(add(origin, rotate_body_to_world(Vec3{range, 0.0, 0.0}, pose.yaw)));
    fan_points.push_back(origin);
    fan_points.push_back(add(origin, rotate_body_to_world(Vec3{range * std::cos(fov), range * std::sin(fov), 0.0}, pose.yaw)));
    fan_points.push_back(origin);
    fan_points.push_back(add(origin, rotate_body_to_world(Vec3{range * std::cos(-fov), range * std::sin(-fov), 0.0}, pose.yaw)));
    add_line_list(
      array, id, stamp, "mission_search_fan", fan_points, 0.045,
      detected ? Rgba{0.1F, 1.0F, 0.45F, 0.9F} : Rgba{1.0F, 0.82F, 0.05F, searching ? 0.9F : 0.45F});

    std::vector<Vec3> arc;
    for (int i = -12; i <= 12; ++i) {
      const double a = fov * static_cast<double>(i) / 12.0;
      arc.push_back(add(origin, rotate_body_to_world(Vec3{range * std::cos(a), range * std::sin(a), 0.0}, pose.yaw)));
    }
    add_line_strip(
      array, id, stamp, "mission_search_arc", arc, 0.035,
      detected ? Rgba{0.1F, 1.0F, 0.45F, 0.75F} : Rgba{1.0F, 0.82F, 0.05F, 0.7F});

    if (!detected) {
      const Vec3 label_pos = add(origin, rotate_body_to_world(Vec3{std::min(range, 4.0), 0.0, 0.7}, pose.yaw));
      add_text(
        array, id, stamp, "mission_search_label", label_pos,
        searching ? "SEARCHING\nfront scan" : "front scan", 0.25,
        Rgba{1.0F, 0.88F, 0.2F, 0.9F});
      return;
    }

    const DetectionState & det = *status.detection;
    Vec3 detection_world = det.has_target_xyz ? det.target_xyz : add(pose.xyz, rotate_body_to_world(det.p_intake, pose.yaw));
    add_sphere(
      array, id, stamp, "mission_detection_point", detection_world, 0.56,
      color_for_class(det.class_name, 1.0F));
    add_line_list(
      array, id, stamp, "mission_detection_line", {origin, detection_world}, 0.06,
      Rgba{0.1F, 1.0F, 0.45F, 1.0F});
    std::ostringstream label;
    label << "DETECTED " << det.class_name << "\n";
    label << trim_id(det.buoy_id) << "\n";
    label << std::fixed << std::setprecision(2)
          << (det.range_estimated ? "visual range~ " : "range ")
          << det.distance << "m  conf " << det.confidence;
    add_text(
      array, id, stamp, "mission_detection_label",
      Vec3{detection_world.x, detection_world.y, detection_world.z + 0.95}, label.str(), 0.27,
      Rgba{0.45F, 1.0F, 0.75F, 1.0F});
  }

  void add_yolo_detections(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const MissionStatus & status, const YoloStatus & yolo, const RobotPose & pose) const
  {
    if (!yolo.valid) {
      return;
    }
    constexpr double plane_range = 3.0;
    constexpr double plane_width = 3.2;
    constexpr double plane_height = 1.8;
    const auto project = [&](double u, double v) {
      const double clamped_u = std::clamp(u, 0.0, 1.0);
      const double clamped_v = std::clamp(v, 0.0, 1.0);
      const Vec3 body{
        plane_range,
        (0.5 - clamped_u) * plane_width,
        (0.5 - clamped_v) * plane_height + 0.25};
      return add(pose.xyz, rotate_body_to_world(body, pose.yaw));
    };

    const Vec3 p00 = project(0.0, 0.0);
    const Vec3 p10 = project(1.0, 0.0);
    const Vec3 p11 = project(1.0, 1.0);
    const Vec3 p01 = project(0.0, 1.0);
    add_line_list(
      array, id, stamp, "mission_yolo_plane",
      {p00, p10, p10, p11, p11, p01, p01, p00}, 0.025,
      Rgba{0.28F, 0.72F, 1.0F, 0.55F});

    if (yolo.detections.empty()) {
      const std::string label = !yolo.model_found ? "YOLO model missing" :
        (!yolo.error.empty() ? "YOLO error" : (yolo.active ? "YOLO scanning" : "YOLO idle"));
      add_text(
        array, id, stamp, "mission_yolo_label", project(0.5, 0.08), label, 0.22,
        yolo.model_found ? Rgba{0.45F, 0.78F, 1.0F, 0.9F} : Rgba{1.0F, 0.35F, 0.25F, 0.9F});
      return;
    }

    const size_t max_boxes = std::min<size_t>(8, yolo.detections.size());
    for (size_t i = 0; i < max_boxes; ++i) {
      const auto & det = yolo.detections[i];
      double x1 = 0.44;
      double y1 = 0.44;
      double x2 = 0.56;
      double y2 = 0.56;
      if (det.xyxy_norm.size() >= 4) {
        x1 = det.xyxy_norm[0];
        y1 = det.xyxy_norm[1];
        x2 = det.xyxy_norm[2];
        y2 = det.xyxy_norm[3];
      } else if (det.center_norm.size() >= 2) {
        x1 = det.center_norm[0] - 0.06;
        y1 = det.center_norm[1] - 0.06;
        x2 = det.center_norm[0] + 0.06;
        y2 = det.center_norm[1] + 0.06;
      }
      x1 = std::clamp(x1, 0.0, 1.0);
      y1 = std::clamp(y1, 0.0, 1.0);
      x2 = std::clamp(x2, 0.0, 1.0);
      y2 = std::clamp(y2, 0.0, 1.0);
      if (x2 < x1) {
        std::swap(x1, x2);
      }
      if (y2 < y1) {
        std::swap(y1, y2);
      }
      const Vec3 b00 = project(x1, y1);
      const Vec3 b10 = project(x2, y1);
      const Vec3 b11 = project(x2, y2);
      const Vec3 b01 = project(x1, y2);
      const double center_u = 0.5 * (x1 + x2);
      const double center_v = 0.5 * (y1 + y2);
      const bool ignored_by_zone =
        yolo_detection_points_to_opponent_zone(status, pose, center_u, plane_width, plane_range);
      add_line_list(
        array, id, stamp, "mission_yolo_bbox",
        {b00, b10, b10, b11, b11, b01, b01, b00}, 0.055,
        ignored_by_zone ? Rgba{0.55F, 0.58F, 0.62F, 0.42F} : color_for_class(det.label, 0.95F));

      std::ostringstream label;
      label << "YOLO " << (ignored_by_zone ? "IGNORED" : det.label) << "\n"
            << (ignored_by_zone ? "opponent zone ray" : det.label) << " "
            << std::fixed << std::setprecision(2) << det.confidence;
      const Vec3 label_pos = project(center_u, std::max(0.0, y1 - 0.04));
      add_text(
        array, id, stamp, "mission_yolo_bbox_label", label_pos, label.str(), 0.19,
        ignored_by_zone ? Rgba{0.78F, 0.80F, 0.84F, 0.85F} : color_for_class(det.label, 1.0F));

      if (ignored_by_zone) {
        add_line_list(
          array, id, stamp, "mission_yolo_zone_gate",
          {pose.xyz, project(center_u, center_v)}, 0.025, Rgba{1.0F, 0.25F, 0.18F, 0.55F});
      }
    }
  }

  bool yolo_detection_points_to_opponent_zone(
    const MissionStatus & status, const RobotPose & pose, double center_u, double plane_width,
    double plane_range) const
  {
    if (!yolo_zone_gate_enabled_) {
      return false;
    }
    const std::string course = lower_ascii(status.own_course);
    if (course != "a" && course != "b") {
      return false;
    }
    const double opponent_sign = course == "a" ? 1.0 : -1.0;
    const Vec3 ray_at_plane =
      rotate_body_to_world(Vec3{plane_range, (0.5 - std::clamp(center_u, 0.0, 1.0)) * plane_width, 0.0}, pose.yaw);
    if (opponent_sign * ray_at_plane.x <= 1.0e-6) {
      return false;
    }
    const double gate_x =
      status.boundary_x + opponent_sign * std::max(0.0, status.boundary_standoff);
    const double t = (gate_x - pose.xyz.x) / ray_at_plane.x;
    if (t < 0.0) {
      return false;
    }
    const double crossing_range_m = t * std::hypot(ray_at_plane.x, ray_at_plane.y);
    return crossing_range_m <= std::max(0.1, yolo_zone_gate_crossing_range_m_);
  }

  void add_robot_state(
    visualization_msgs::msg::MarkerArray & array, int & id, const rclcpp::Time & stamp,
    const MissionStatus & status, const RobotPose & pose) const
  {
    add_sphere(
      array, id, stamp, "mission_robot", pose.xyz, 0.34, Rgba{0.25F, 0.75F, 1.0F, 0.95F});
    add_arrow(
      array, id, stamp, "mission_robot_heading", pose.xyz,
      add(pose.xyz, rotate_body_to_world(Vec3{1.4, 0.0, 0.0}, pose.yaw)),
      0.07, 0.18, Rgba{0.25F, 0.75F, 1.0F, 1.0F});

    if (latest_hydrophone_body_ && stamp.seconds() - latest_hydrophone_wall_s_ <= 1.5) {
      const Vec3 start{pose.xyz.x, pose.xyz.y, pose.xyz.z + 0.28};
      const Vec3 direction_world = rotate_body_to_world(*latest_hydrophone_body_, pose.yaw);
      const Vec3 end{
        start.x + direction_world.x * 2.2,
        start.y + direction_world.y * 2.2,
        start.z + direction_world.z * 2.2};
      add_arrow(
        array, id, stamp, "mission_hydrophone_estimate", start, end,
        0.08, 0.22, Rgba{1.0F, 0.08F, 0.05F, 1.0F});
      add_text(
        array, id, stamp, "mission_hydrophone_label",
        Vec3{end.x, end.y, end.z + 0.25}, "PINGER ESTIMATE", 0.22,
        Rgba{1.0F, 0.25F, 0.18F, 1.0F});
    }

    if (status.command) {
      const Vec3 cmd_body{status.command->forward, status.command->sway, 0.0};
      const Vec3 cmd_world = rotate_body_to_world(cmd_body, pose.yaw);
      const double magnitude = std::hypot(cmd_world.x, cmd_world.y);
      if (magnitude > 0.02) {
        const Vec3 start{pose.xyz.x, pose.xyz.y, pose.xyz.z + 0.35};
        const Vec3 end{start.x + cmd_world.x * 1.4, start.y + cmd_world.y * 1.4, start.z};
        add_arrow(
          array, id, stamp, "mission_command_vector", start, end, 0.06, 0.17,
          Rgba{1.0F, 0.45F, 0.1F, 1.0F});
      }
    }

    std::ostringstream text;
    text << "FSM " << status.state << "\n";
    text << "Robot " << (status.robot_state_label.empty() ? status.robot_state : status.robot_state_label) << "\n";
    if (!status.target_id.empty()) {
      text << "Target " << status.target_class << " " << trim_id(status.target_id) << "\n";
    }
    text << "rem " << status.remaining_attached << " | ok " << status.processed_count
         << " | fail " << status.failed_count << " | scored " << status.scored_count;
    if (status.command) {
      text << "\ncmd f/s/h/y/p " << std::fixed << std::setprecision(2)
           << status.command->forward << " " << status.command->sway << " "
           << status.command->heave << " " << status.command->yaw << " "
           << status.command->pitch;
    }
    add_text(
      array, id, stamp, "mission_robot_status", Vec3{pose.xyz.x, pose.xyz.y, pose.xyz.z + 1.45},
      text.str(), 0.27, Rgba{0.88F, 0.96F, 1.0F, 1.0F});
  }

  RobotPose robot_pose_for_status(const MissionStatus & status) const
  {
    if (status.robot) {
      return *status.robot;
    }
    if (last_pose_) {
      return *last_pose_;
    }
    return RobotPose{};
  }

  void publish_markers()
  {
    const auto stamp = now();
    const bool fsm_fresh = latest_fsm_status_wall_s_ > 0.0 &&
      (stamp.seconds() - latest_fsm_status_wall_s_) <= 2.5;
    const std::string text = fsm_fresh ? latest_fsm_status_text_ : read_text_file(status_path_);
    MissionStatus status = parse_status(text);
    const bool observation_fresh = latest_observation_.has_value() &&
      latest_observation_wall_s_ > 0.0 && (stamp.seconds() - latest_observation_wall_s_) <= 1.0;
    if (observation_fresh && latest_observation_->detected) {
      status.detection = detection_from_observation(*latest_observation_);
      if (status.target_class.empty()) {
        status.target_class = latest_observation_->class_label;
      }
      if (status.target_id.empty()) {
        status.target_id = "camera_target";
      }
    }
    const bool yolo_fresh = latest_yolo_wall_s_ > 0.0 && (stamp.seconds() - latest_yolo_wall_s_) <= 2.5;
    const YoloStatus yolo = observation_fresh ? yolo_from_observation(*latest_observation_) :
      (yolo_fresh ? parse_yolo_status(latest_yolo_text_) : YoloStatus{});
    const RobotPose pose = robot_pose_for_status(status);

    visualization_msgs::msg::MarkerArray array;
    add_delete_all(array, stamp);
    int id = 1;
    add_course_regions(array, id, stamp, status);
    add_score_zone(array, id, stamp, status);
    add_surface_collector(array, id, stamp, status);
    add_buoys(array, id, stamp, status);
    add_search_and_detection(array, id, stamp, status, pose);
    add_yolo_detections(array, id, stamp, status, yolo, pose);
    add_robot_state(array, id, stamp, status, pose);

    if (!status.valid) {
      add_text(
        array, id, stamp, "mission_waiting_status", Vec3{0.0, 0.0, 1.0},
        "Mission RViz\nwaiting for /mission/fsm/status", 0.35,
        Rgba{1.0F, 0.85F, 0.25F, 1.0F});
    }

    marker_pub_->publish(array);
  }

  std::string status_path_;
  std::string marker_topic_;
  std::string marker_frame_;
  std::string pose_topic_;
  std::string pose_type_;
  std::string yolo_detection_topic_;
  std::string observation_topic_;
  std::string fsm_status_topic_;
  std::string hydrophone_body_topic_;
  std::string latest_yolo_text_;
  std::string latest_fsm_status_text_;
  std::string default_own_course_{"a"};
  double latest_yolo_wall_s_{0.0};
  double latest_observation_wall_s_{0.0};
  double latest_fsm_status_wall_s_{0.0};
  double latest_hydrophone_wall_s_{0.0};
  double default_boundary_x_{0.0};
  double default_boundary_margin_{0.8};
  double default_boundary_standoff_{0.7};
  bool yolo_zone_gate_enabled_{true};
  double yolo_zone_gate_crossing_range_m_{8.0};
  double publish_rate_hz_{5.0};
  double x_min_{-15.5};
  double x_max_{15.5};
  double y_min_{-15.5};
  double y_max_{15.5};
  double z_min_{-9.5};
  double z_max_{0.8};
  std::optional<RobotPose> last_pose_;
  std::optional<hit25_auv_ros2_msg::msg::BuoyObservation> latest_observation_;
  std::optional<Vec3> latest_hydrophone_body_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
  rclcpp::Subscription<hit25_auv_ros2_msg::msg::BuoyObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fsm_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr hydrophone_body_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionRvizVisualizer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok()) {
    executor.spin_once(5ms);
    std::this_thread::sleep_for(50ms);
  }
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
