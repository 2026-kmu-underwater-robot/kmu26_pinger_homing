#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <Eigen/Dense>
#include <hit25_auv_ros2_msg/msg/buoy_observation.hpp>
#include <hit25_auv_ros2_msg/msg/collector_state.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "vision_buoy/vision_guidance.hpp"

namespace {

using Clock = std::chrono::steady_clock;
constexpr int kNeutral = 1500;
constexpr std::size_t kPitch = 1;
constexpr std::size_t kHeave = 2;
constexpr std::size_t kYaw = 3;
constexpr std::size_t kForward = 4;
constexpr std::size_t kSway = 5;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double wrap_pi(double value) {
  constexpr double pi = 3.14159265358979323846;
  while (value > pi) value -= 2.0 * pi;
  while (value < -pi) value += 2.0 * pi;
  return value;
}

double age(const Clock::time_point &stamp) {
  if (stamp.time_since_epoch().count() == 0) return 1.0e9;
  return std::chrono::duration<double>(Clock::now() - stamp).count();
}

struct Command {
  double forward{0.0};
  double sway{0.0};
  double heave{0.0};
  double yaw{0.0};
  double pitch{0.0};
};

struct RangeMotionSample {
  Eigen::Vector3d delta_position{0.0, 0.0, 0.0};
  double delta_range{0.0};
  double delta_time{0.0};
};

struct AcousticPositionSample {
  Eigen::Vector2d vehicle_xy{0.0, 0.0};
  double vehicle_z{0.0};
  double slant_range{0.0};
};

struct PhaseRangePositionSample {
  Eigen::Vector3d vehicle_position{0.0, 0.0, 0.0};
  double cumulative_range_change{0.0};
  double elapsed_s{0.0};
};

struct TimedPhasePosition {
  Clock::time_point stamp{};
  Eigen::Vector3d position{0.0, 0.0, 0.0};
};

enum class Phase {
  IDLE,
  WAIT_VEHICLE,
  WAIT_ARM,
  PINGER_SEARCH,
  PINGER_HOMING,
  PINGER_FINE_ALIGN,
  PINGER_CAPTURE,
  PINGER_VERIFY,
  PINGER_BACKOFF,
  TURN_TO_OWN_ZONE,
  VISION_CONTROL,
  SEARCH,
  APPROACH,
  FINE_ALIGN,
  CAPTURE,
  VERIFY_CAPTURE,
  BACKOFF,
  SURFACE_SEARCH,
  SURFACE_COLLECT,
  RETURN_ZONE,
  SCORE_ZONE_IN,
  RELEASE,
  COMPLETE,
  FAILED,
};

bool is_pinger_phase(Phase phase) {
  return phase == Phase::PINGER_SEARCH || phase == Phase::PINGER_HOMING ||
         phase == Phase::PINGER_FINE_ALIGN || phase == Phase::PINGER_CAPTURE ||
         phase == Phase::PINGER_VERIFY || phase == Phase::PINGER_BACKOFF;
}

bool is_underwater_buoy_phase(Phase phase) {
  return phase == Phase::SEARCH || phase == Phase::APPROACH ||
         phase == Phase::FINE_ALIGN || phase == Phase::CAPTURE ||
         phase == Phase::VERIFY_CAPTURE || phase == Phase::BACKOFF;
}

const char *phase_name(Phase phase) {
  switch (phase) {
    case Phase::IDLE: return "IDLE";
    case Phase::WAIT_VEHICLE: return "WAIT_VEHICLE";
    case Phase::WAIT_ARM: return "WAIT_ARM";
    case Phase::PINGER_SEARCH: return "PINGER_SEARCH";
    case Phase::PINGER_HOMING: return "PINGER_HOMING";
    case Phase::PINGER_FINE_ALIGN: return "PINGER_YOLO_FINE_ALIGN";
    case Phase::PINGER_CAPTURE: return "PINGER_CAPTURE";
    case Phase::PINGER_VERIFY: return "PINGER_VERIFY";
    case Phase::PINGER_BACKOFF: return "PINGER_BACKOFF";
    case Phase::TURN_TO_OWN_ZONE: return "TURN_TO_OWN_ZONE";
    case Phase::VISION_CONTROL: return "VISION_CONTROL";
    case Phase::SEARCH: return "BUOY_SEARCHING";
    case Phase::APPROACH: return "BUOY_DETECTED_APPROACH";
    case Phase::FINE_ALIGN: return "BUOY_FINE_ALIGN";
    case Phase::CAPTURE: return "CAPTURE";
    case Phase::VERIFY_CAPTURE: return "VERIFY_CAPTURE";
    case Phase::BACKOFF: return "BUOY_BACKOFF";
    case Phase::SURFACE_SEARCH: return "SURFACE_BUOY_SEARCHING";
    case Phase::SURFACE_COLLECT: return "SURFACE_COLLECT";
    case Phase::RETURN_ZONE: return "RETURN_ZONE";
    case Phase::SCORE_ZONE_IN: return "SCORE_ZONE_IN";
    case Phase::RELEASE: return "RELEASE";
    case Phase::COMPLETE: return "COMPLETE";
    case Phase::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

}  // namespace

class ObservationMissionFsm final : public rclcpp::Node {
 public:
  ObservationMissionFsm() : Node("observation_mission_fsm") {
    observation_topic_ = declare_parameter<std::string>(
        "observation_topic", "/vision/buoy_observation");
    collector_topic_ = declare_parameter<std::string>("collector_topic", "/collector/state");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");
    output_topic_ = declare_parameter<std::string>(
        "output_topic", "/control/mission/rc_override");
    status_topic_ = declare_parameter<std::string>("status_topic", "/mission/fsm/status");
    delegate_vision_control_ = declare_parameter<bool>("delegate_vision_control", true);
    vision_enable_topic_ = declare_parameter<std::string>(
        "vision_enable_topic", "/control/vision/enable");
    vision_state_topic_ = declare_parameter<std::string>(
        "vision_state_topic", "/control/vision/state");
    vision_state_timeout_s_ = declare_parameter<double>("vision_state_timeout_s", 1.5);
    vision_search_handoff_s_ = declare_parameter<double>("vision_search_handoff_s", 0.75);
    vision_complete_requires_detach_count_ = declare_parameter<bool>(
        "vision_complete_requires_detach_count", true);
    status_json_path_ = declare_parameter<std::string>("status_json_path", "");
    status_json_period_s_ = declare_parameter<double>("status_json_period_s", 0.10);
    hydrophone_direction_topic_ = declare_parameter<std::string>(
        "hydrophone_direction_topic", "/homing/direction");
    hydrophone_direction_frame_ = declare_parameter<std::string>(
        "hydrophone_direction_frame", "world");
    hydrophone_body_topic_ = declare_parameter<std::string>(
        "hydrophone_body_topic", "/mission/hydrophone/direction_body");
    delta_range_topic_ = declare_parameter<std::string>(
        "delta_range_topic", "/audio_phase_estimator/delta_range_m");
    iq_magnitude_topic_ = declare_parameter<std::string>(
        "iq_magnitude_topic", "/audio_phase_estimator/iq_magnitude");
    enabled_ = declare_parameter<bool>("enabled", false);
    dry_run_ = declare_parameter<bool>("dry_run", true);
    require_armed_ = declare_parameter<bool>("require_armed", true);
    expected_detach_count_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("expected_detach_count", 1)));
    expected_net_count_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("expected_net_count", expected_detach_count_)));
    use_pinger_first_ = declare_parameter<bool>("use_pinger_first", true);
    require_pinger_yolo_for_capture_ = declare_parameter<bool>(
        "require_pinger_yolo_for_capture", true);
    start_surface_phase_ = declare_parameter<bool>("start_surface_phase", false);
    hydrophone_timeout_s_ = declare_parameter<double>("hydrophone_timeout_s", 3.0);
    pinger_position_fit_timeout_s_ = declare_parameter<double>(
        "pinger_position_fit_timeout_s", 8.0);
    pinger_min_probe_s_ = declare_parameter<double>("pinger_min_probe_s", 6.0);
    pinger_max_probe_s_ = declare_parameter<double>("pinger_max_probe_s", 45.0);
    require_pinger_position_fit_ = declare_parameter<bool>("require_pinger_position_fit", true);
    use_acoustic_position_fusion_ = declare_parameter<bool>(
        "use_acoustic_position_fusion", false);
    pinger_acoustic_position_lock_range_m_ = declare_parameter<double>(
        "pinger_acoustic_position_lock_range_m", 1.4);
    pinger_acoustic_position_min_range_m_ = declare_parameter<double>(
        "pinger_acoustic_position_min_range_m", 0.0);
    use_phase_range_position_fusion_ = declare_parameter<bool>(
        "use_phase_range_position_fusion", false);
    phase_range_position_timeout_s_ = declare_parameter<double>(
        "phase_range_position_timeout_s", 120.0);
    phase_range_min_fit_duration_s_ = declare_parameter<double>(
        "phase_range_min_fit_duration_s", 11.5);
    phase_range_measurement_delay_s_ = declare_parameter<double>(
        "phase_range_measurement_delay_s", 0.128);
    allow_internal_hydrophone_direction_fallback_ = declare_parameter<bool>(
        "allow_internal_hydrophone_direction_fallback", false);
    prefer_upstream_hydrophone_direction_ = declare_parameter<bool>(
        "prefer_upstream_hydrophone_direction", true);
    prefer_internal_hydrophone_direction_ = declare_parameter<bool>(
        "prefer_internal_hydrophone_direction", true);
    pinger_position_fit_bearing_tolerance_rad_ = declare_parameter<double>(
        "pinger_position_fit_bearing_tolerance_rad", 0.45);
    pinger_forward_fast_ = declare_parameter<double>("pinger_forward_fast", 0.55);
    pinger_forward_turn_ = declare_parameter<double>("pinger_forward_turn", 0.10);
    pinger_heading_drive_tolerance_rad_ = declare_parameter<double>(
        "pinger_heading_drive_tolerance_rad", 0.22);
    pinger_near_slow_range_m_ = declare_parameter<double>(
        "pinger_near_slow_range_m", 8.0);
    pinger_near_forward_ = declare_parameter<double>("pinger_near_forward", 0.30);
    pinger_final_slow_range_m_ = declare_parameter<double>(
        "pinger_final_slow_range_m", 2.5);
    pinger_final_forward_ = declare_parameter<double>("pinger_final_forward", 0.18);
    pinger_probe_forward_ = declare_parameter<double>("pinger_probe_forward", 0.24);
    pinger_probe_yaw_ = declare_parameter<double>("pinger_probe_yaw", 0.30);
    pinger_homing_leg_s_ = declare_parameter<double>("pinger_homing_leg_s", 10.0);
    pinger_homing_sway_amplitude_ = declare_parameter<double>(
        "pinger_homing_sway_amplitude", 0.0);
    pinger_homing_sway_period_s_ = declare_parameter<double>(
        "pinger_homing_sway_period_s", 6.0);
    pinger_homing_yaw_dither_amplitude_ = declare_parameter<double>(
        "pinger_homing_yaw_dither_amplitude", 0.06);
    pinger_homing_yaw_dither_period_s_ = declare_parameter<double>(
        "pinger_homing_yaw_dither_period_s", 5.0);
    pinger_homing_drive_s_ = declare_parameter<double>("pinger_homing_drive_s", 0.0);
    pinger_homing_pause_s_ = declare_parameter<double>("pinger_homing_pause_s", 0.0);
    pinger_spin_rehome_yaw_rad_ = declare_parameter<double>(
        "pinger_spin_rehome_yaw_rad", 5.75);
    pinger_spin_rehome_max_translation_m_ = declare_parameter<double>(
        "pinger_spin_rehome_max_translation_m", 0.75);
    pinger_spin_rehome_stop_s_ = declare_parameter<double>(
        "pinger_spin_rehome_stop_s", 1.0);
    pinger_range_regression_margin_m_ = declare_parameter<double>(
        "pinger_range_regression_margin_m", 0.40);
    pinger_range_regression_hold_s_ = declare_parameter<double>(
        "pinger_range_regression_hold_s", 0.65);
    pinger_range_progress_grace_s_ = declare_parameter<double>(
        "pinger_range_progress_grace_s", 0.80);
    pinger_range_progress_check_s_ = declare_parameter<double>(
        "pinger_range_progress_check_s", 2.8);
    pinger_range_min_progress_m_ = declare_parameter<double>(
        "pinger_range_min_progress_m", 0.12);
    pinger_doppler_approach_delta_m_ = declare_parameter<double>(
        "pinger_doppler_approach_delta_m", 0.003);
    pinger_doppler_recede_delta_m_ = declare_parameter<double>(
        "pinger_doppler_recede_delta_m", 0.003);
    pinger_doppler_reversal_max_range_m_ = declare_parameter<double>(
        "pinger_doppler_reversal_max_range_m", 4.0);
    pinger_doppler_approach_samples_ = static_cast<int>(std::max<std::int64_t>(
        2, declare_parameter<std::int64_t>("pinger_doppler_approach_samples", 3)));
    pinger_doppler_recede_samples_ = static_cast<int>(std::max<std::int64_t>(
        2, declare_parameter<std::int64_t>("pinger_doppler_recede_samples", 2)));
    pinger_doppler_brake_s_ = declare_parameter<double>("pinger_doppler_brake_s", 0.70);
    pinger_doppler_brake_reverse_ = declare_parameter<double>(
        "pinger_doppler_brake_reverse", -0.22);
    pinger_depth_z_ = declare_parameter<double>("pinger_depth_z", -8.5);
    pinger_acoustic_source_depth_z_ = declare_parameter<double>(
        "pinger_acoustic_source_depth_z", -8.865);
    pinger_transit_depth_z_ = declare_parameter<double>("pinger_transit_depth_z", -7.7);
    pinger_depth_transition_range_m_ = declare_parameter<double>(
        "pinger_depth_transition_range_m", 3.0);
    pinger_depth_kp_ = declare_parameter<double>("pinger_depth_kp", 0.18);
    pinger_yaw_kp_ = declare_parameter<double>("pinger_yaw_kp", 0.85);
    pinger_yaw_kd_ = declare_parameter<double>("pinger_yaw_kd", 0.16);
    pinger_yaw_limit_ = declare_parameter<double>("pinger_yaw_limit", 0.44);
    pinger_scan_yaw_ = declare_parameter<double>("pinger_scan_yaw", 0.07);
    pinger_scan_yaw_gain_ = declare_parameter<double>("pinger_scan_yaw_gain", 0.80);
    pinger_scan_yaw_limit_ = declare_parameter<double>("pinger_scan_yaw_limit", 0.30);
    pinger_acoustic_crawl_forward_ = declare_parameter<double>(
        "pinger_acoustic_crawl_forward", 0.20);
    pinger_acoustic_crawl_bearing_rad_ = declare_parameter<double>(
        "pinger_acoustic_crawl_bearing_rad", 0.45);
    pinger_acoustic_capture_range_m_ = declare_parameter<double>(
        "pinger_acoustic_capture_range_m", 0.78);
    pinger_acoustic_capture_bearing_rad_ = declare_parameter<double>(
        "pinger_acoustic_capture_bearing_rad", 0.16);
    pinger_heave_kp_ = declare_parameter<double>("pinger_heave_kp", 0.42);
    pinger_heave_limit_ = declare_parameter<double>("pinger_heave_limit", 0.34);
    pinger_vertical_direction_deadband_ = declare_parameter<double>(
        "pinger_vertical_direction_deadband", 0.08);
    pinger_vertical_alignment_tolerance_ = declare_parameter<double>(
        "pinger_vertical_alignment_tolerance", 0.22);
    pinger_vertical_forward_limit_ = declare_parameter<double>(
        "pinger_vertical_forward_limit", 0.0);
    pinger_vertical_yaw_limit_ = declare_parameter<double>(
        "pinger_vertical_yaw_limit", 0.08);
    pinger_acoustic_vertical_zero_range_m_ = declare_parameter<double>(
        "pinger_acoustic_vertical_zero_range_m", 0.80);
    pinger_acoustic_vertical_full_range_m_ = declare_parameter<double>(
        "pinger_acoustic_vertical_full_range_m", 2.0);
    underwater_target_depth_z_ = declare_parameter<double>(
        "underwater_target_depth_z", -8.5);
    underwater_depth_kp_ = declare_parameter<double>("underwater_depth_kp", 0.30);
    underwater_visual_heave_gain_ = declare_parameter<double>(
        "underwater_visual_heave_gain", 0.05);
    pinger_yolo_min_bbox_ratio_ = declare_parameter<double>(
        "pinger_yolo_min_bbox_ratio", 0.06);
    pinger_yolo_acoustic_range_m_ = declare_parameter<double>(
        "pinger_yolo_acoustic_range_m", 2.5);
    pinger_visual_bearing_tolerance_rad_ = declare_parameter<double>(
        "pinger_visual_bearing_tolerance_rad", 0.35);
    pinger_visual_reacquire_timeout_s_ = declare_parameter<double>(
        "pinger_visual_reacquire_timeout_s", 1.0);
    pinger_camera_hfov_rad_ = declare_parameter<double>("pinger_camera_hfov_rad", 1.211);
    pinger_capture_commit_range_m_ = declare_parameter<double>(
        "pinger_capture_commit_range_m", 0.55);
    pinger_capture_min_bbox_ratio_ = declare_parameter<double>(
        "pinger_capture_min_bbox_ratio", 0.08);
    pinger_rake_lane_blend_start_m_ = declare_parameter<double>(
        "pinger_rake_lane_blend_start_m", 0.90);
    pinger_rake_lane_full_range_m_ = declare_parameter<double>(
        "pinger_rake_lane_full_range_m", 0.45);
    pinger_iq_range_reference_ = declare_parameter<double>(
        "pinger_iq_range_reference", 0.325);
    observation_timeout_s_ = declare_parameter<double>("observation_timeout_s", 1.5);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.8);
    state_timeout_s_ = declare_parameter<double>("state_timeout_s", 8.0);
    vehicle_disconnect_grace_s_ = std::max(
        0.0, declare_parameter<double>("vehicle_disconnect_grace_s", 0.0));
    collector_timeout_s_ = declare_parameter<double>("collector_timeout_s", 6.0);
    capture_drive_s_ = declare_parameter<double>("capture_drive_s", 1.4);
    capture_insert_forward_ = declare_parameter<double>("capture_insert_forward", 0.28);
    capture_backoff_s_ = declare_parameter<double>("capture_backoff_s", 0.55);
    capture_backoff_forward_ = declare_parameter<double>("capture_backoff_forward", -0.16);
    capture_heading_kp_ = declare_parameter<double>("capture_heading_kp", 0.90);
    capture_heading_kd_ = declare_parameter<double>("capture_heading_kd", 0.15);
    capture_heading_yaw_limit_ = declare_parameter<double>("capture_heading_yaw_limit", 0.18);
    capture_center_tolerance_x_ = declare_parameter<double>(
        "capture_center_tolerance_x", 0.08);
    capture_center_tolerance_y_ = declare_parameter<double>(
        "capture_center_tolerance_y", 0.12);
    capture_alignment_hold_s_ = declare_parameter<double>(
        "capture_alignment_hold_s", 0.35);
    capture_aim_offset_x_ = declare_parameter<double>("capture_aim_offset_x", 0.0);
    capture_aim_offset_y_ = declare_parameter<double>("capture_aim_offset_y", 0.0);
    alignment_hold_s_ = declare_parameter<double>("alignment_hold_s", 0.15);
    fine_bbox_ratio_ = declare_parameter<double>("fine_bbox_ratio", 0.18);
    capture_bbox_ratio_ = declare_parameter<double>("capture_bbox_ratio", 0.32);
    center_tolerance_ = declare_parameter<double>("center_tolerance", 0.16);
    min_confidence_ = declare_parameter<double>("min_confidence", 0.35);
    target_class_name_ = declare_parameter<std::string>("target_class_name", "buoy");
    pinger_visual_class_names_ = declare_parameter<std::vector<std::string>>(
        "pinger_visual_class_names", std::vector<std::string>{"buoy", "stick"});
    for (auto &name : pinger_visual_class_names_) {
      std::transform(name.begin(), name.end(), name.begin(),
                     [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    }
    underwater_visual_class_names_ = declare_parameter<std::vector<std::string>>(
        "underwater_visual_class_names", std::vector<std::string>{"stick", "buoy"});
    for (auto &name : underwater_visual_class_names_) {
      std::transform(name.begin(), name.end(), name.begin(),
                     [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    }
    observation_filter_alpha_ = declare_parameter<double>("observation_filter_alpha", 0.45);
    yaw_kp_ = declare_parameter<double>("yaw_kp", 0.30);
    yaw_kd_ = declare_parameter<double>("yaw_kd", 0.02);
    heave_kp_ = declare_parameter<double>("heave_kp", 0.22);
    max_visual_yaw_ = declare_parameter<double>("max_visual_yaw", 0.18);
    max_visual_heave_ = declare_parameter<double>("max_visual_heave", 0.18);
    search_yaw_ = declare_parameter<double>("search_yaw", 0.20);
    command_slew_per_s_ = declare_parameter<double>("command_slew_per_s", 0.8);
    own_course_ = declare_parameter<std::string>("own_course", "a");
    std::transform(own_course_.begin(), own_course_.end(), own_course_.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    course_boundary_x_ = declare_parameter<double>("course_boundary_x", 0.0);
    course_boundary_margin_ = declare_parameter<double>("course_boundary_margin", 0.8);
    course_boundary_standoff_ = declare_parameter<double>("course_boundary_standoff", 0.7);
    detached_exclusion_radius_m_ = declare_parameter<double>(
        "detached_exclusion_radius_m", 1.8);
    search_area_center_x_ = declare_parameter<double>("search_area_center_x", -8.5);
    search_area_center_y_ = declare_parameter<double>("search_area_center_y", 0.0);
    search_area_width_m_ = declare_parameter<double>("search_area_width_m", 9.0);
    search_area_height_m_ = declare_parameter<double>("search_area_height_m", 16.0);
    search_lane_spacing_m_ = declare_parameter<double>("search_lane_spacing_m", 3.0);
    search_waypoint_tolerance_m_ = declare_parameter<double>(
        "search_waypoint_tolerance_m", 0.9);
    search_forward_ = declare_parameter<double>("search_forward", 0.48);
    search_turn_yaw_limit_ = declare_parameter<double>("search_turn_yaw_limit", 0.65);
    search_heading_tolerance_rad_ = declare_parameter<double>(
        "search_heading_tolerance_rad", 0.70);
    search_escape_forward_ = declare_parameter<double>("search_escape_forward", 0.55);
    search_accept_bbox_ratio_ = declare_parameter<double>(
        "search_accept_bbox_ratio", 0.065);
    target_height_m_ = declare_parameter<double>("target_height_m", 0.55);
    camera_vertical_fov_rad_ = declare_parameter<double>("camera_vertical_fov_rad", 0.75);
    score_x_ = declare_parameter<double>("score_zone_x", 0.0);
    score_y_ = declare_parameter<double>("score_zone_y", 0.0);
    score_radius_ = declare_parameter<double>("score_zone_radius", 0.8);
    release_timeout_s_ = declare_parameter<double>("release_timeout_s", 8.0);
    surface_collect_depth_z_ = declare_parameter<double>("surface_collect_depth_z", -0.31);
    surface_collect_depth_tolerance_m_ = declare_parameter<double>(
        "surface_collect_depth_tolerance_m", 0.12);
    surface_depth_kp_ = declare_parameter<double>("surface_depth_kp", 0.55);
    surface_yaw_kp_ = declare_parameter<double>("surface_yaw_kp", 0.55);
    surface_yaw_limit_ = declare_parameter<double>("surface_yaw_limit", 0.35);
    surface_forward_ = declare_parameter<double>("surface_forward", 0.45);
    surface_turn_forward_ = declare_parameter<double>("surface_turn_forward", 0.0);
    surface_center_tolerance_ = declare_parameter<double>(
        "surface_center_tolerance", 0.10);
    surface_steer_timeout_s_ = declare_parameter<double>(
        "surface_steer_timeout_s", 0.65);
    rc_span_ = declare_parameter<double>("rc_pwm_span", 400.0);
    invert_heave_ = declare_parameter<bool>("invert_rc_heave", true);
    invert_yaw_ = declare_parameter<bool>("invert_rc_yaw", true);

    const auto telemetry_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    vision_state_sub_ = create_subscription<std_msgs::msg::String>(
        vision_state_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          vision_state_ = msg->data;
          std::transform(
              vision_state_.begin(), vision_state_.end(), vision_state_.begin(),
              [](unsigned char value) { return static_cast<char>(std::toupper(value)); });
          vision_state_received_ = Clock::now();
        });
    observation_sub_ = create_subscription<hit25_auv_ros2_msg::msg::BuoyObservation>(
        observation_topic_, telemetry_qos,
        [this](const hit25_auv_ros2_msg::msg::BuoyObservation::SharedPtr msg) {
          if (msg->detected && msg->image_width > 0 && msg->image_height > 0 &&
              std::isfinite(msg->normalized_error_x) &&
              std::isfinite(msg->normalized_error_y) &&
              msg->confidence >= min_confidence_ && class_matches(*msg)) {
            observation_ = *msg;
            observation_received_ = Clock::now();
            update_filtered_observation(*msg);
            if (is_pinger_phase(phase_)) {
              pinger_visual_last_seen_ = observation_received_;
            }
          }
        });
    collector_sub_ = create_subscription<hit25_auv_ros2_msg::msg::CollectorState>(
        collector_topic_, telemetry_qos,
        [this](const hit25_auv_ros2_msg::msg::CollectorState::SharedPtr msg) {
          collector_ = *msg;
          collector_received_ = Clock::now();
          ++collector_sequence_;
          std::string target = msg->target_id;
          std::transform(target.begin(), target.end(), target.begin(),
                         [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
          if (is_pinger_phase(phase_) && msg->detached && !target.empty() &&
              target.find("pinger") == std::string::npos) {
            non_pinger_detached_during_pinger_ = true;
          }
          if (target.find("pinger") != std::string::npos &&
              (msg->detached || msg->captured || msg->netted)) {
            pinger_detached_observed_ = true;
            if (!msg->target_id.empty()) detached_target_ids_.insert(msg->target_id);
          }
          if (!msg->target_id.empty() && msg->detached) {
            if (detached_target_ids_.insert(msg->target_id).second) {
              remember_detach_contact();
            }
          }
          if (!msg->target_id.empty() && msg->netted) {
            netted_target_ids_.insert(msg->target_id);
          }
          if (!msg->target_id.empty() && msg->released) {
            released_target_ids_.insert(msg->target_id);
          }
          detached_count_ = static_cast<int>(detached_target_ids_.size());
          netted_count_ = static_cast<int>(netted_target_ids_.size());
          released_count_ = static_cast<int>(released_target_ids_.size());
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, telemetry_qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          if (enabled_ && is_pinger_phase(phase_)) {
            const Eigen::Vector3d position(
                msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->pose.pose.position.z);
            if (pinger_path_previous_position_) {
              const double distance = (position - *pinger_path_previous_position_).norm();
              if (std::isfinite(distance) && distance < 1.0) {
                pinger_path_length_m_ += distance;
              }
            }
            pinger_path_previous_position_ = position;
          }
          record_phase_odom_position(*msg);
          update_range_motion_fit(*msg);
          odom_ = *msg;
          odom_received_ = Clock::now();
        });
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        state_topic_, telemetry_qos,
        [this](const mavros_msgs::msg::State::SharedPtr msg) {
          const auto now = Clock::now();
          vehicle_state_ = *msg;
          state_received_ = now;
          if (msg->connected) last_vehicle_connected_ = now;
          if (msg->armed) last_vehicle_armed_ = now;
        });
    hydrophone_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        hydrophone_direction_topic_, telemetry_qos,
        [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
          const double norm = std::sqrt(
              msg->vector.x * msg->vector.x + msg->vector.y * msg->vector.y +
              msg->vector.z * msg->vector.z);
          if (!std::isfinite(norm) || norm < 1.0e-6) return;
          hydrophone_direction_[0] = msg->vector.x / norm;
          hydrophone_direction_[1] = msg->vector.y / norm;
          hydrophone_direction_[2] = msg->vector.z / norm;
          hydrophone_received_ = Clock::now();
          if (enabled_ && phase_ == Phase::PINGER_SEARCH &&
              age(phase_started_) >= 2.0 && !phase_fitted_pinger_xy_) {
            const Eigen::Vector3d direction(
                hydrophone_direction_[0], hydrophone_direction_[1], hydrophone_direction_[2]);
            if (!phase_seed_direction_world_) {
              phase_seed_direction_world_ = direction;
            } else {
              const Eigen::Vector3d filtered =
                  0.75 * *phase_seed_direction_world_ + 0.25 * direction;
              if (filtered.norm() > 1.0e-6) {
                phase_seed_direction_world_ = filtered.normalized();
              }
            }
          }
        });
    delta_range_sub_ = create_subscription<std_msgs::msg::Float64>(
        delta_range_topic_, rclcpp::QoS(50),
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          if (!std::isfinite(msg->data) || std::abs(msg->data) > 0.10) return;
          pending_delta_range_ += msg->data;
          ++pending_delta_count_;
          delta_range_received_ = Clock::now();
          raw_delta_range_window_.push_back(msg->data);
          while (raw_delta_range_window_.size() > 3) raw_delta_range_window_.pop_front();
          std::vector<double> sorted_delta(
              raw_delta_range_window_.begin(), raw_delta_range_window_.end());
          std::sort(sorted_delta.begin(), sorted_delta.end());
          const double robust_delta = sorted_delta[sorted_delta.size() / 2];
          cumulative_phase_range_change_ += robust_delta;
          update_pinger_doppler_trend(robust_delta);
          update_phase_range_position_fit();
        });
    iq_magnitude_sub_ = create_subscription<std_msgs::msg::Float64>(
        iq_magnitude_topic_, rclcpp::QoS(50),
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          if (!std::isfinite(msg->data) || msg->data <= 1.0e-6) return;
          const double range = std::pow(pinger_iq_range_reference_ / msg->data, 2.0);
          if (!std::isfinite(range) || range < 0.05 || range > 100.0) return;
          acoustic_range_window_.push_back(range);
          while (acoustic_range_window_.size() > 9) acoustic_range_window_.pop_front();
          std::vector<double> sorted_ranges(
              acoustic_range_window_.begin(), acoustic_range_window_.end());
          std::nth_element(
              sorted_ranges.begin(), sorted_ranges.begin() + sorted_ranges.size() / 2,
              sorted_ranges.end());
          const double filtered_range = sorted_ranges[sorted_ranges.size() / 2];
          latest_raw_acoustic_range_m_ = filtered_range;
          raw_acoustic_range_received_ = Clock::now();
          acoustic_range_m_ = acoustic_range_m_
              ? 0.30 * *acoustic_range_m_ + 0.70 * filtered_range
              : filtered_range;
          if (!phase_initial_acoustic_range_m_) {
            phase_initial_acoustic_range_m_ = filtered_range;
          }
          iq_magnitude_received_ = Clock::now();
        });
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(output_topic_, rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));
    hydrophone_body_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        hydrophone_body_topic_, rclcpp::QoS(10));
    vision_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
        vision_enable_topic_, rclcpp::QoS(1).reliable().transient_local());
    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "/mission/fsm/enable",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
               std_srvs::srv::SetBool::Response::SharedPtr response) {
          enabled_ = request->data;
          reset(enabled_ ? Phase::WAIT_VEHICLE : Phase::IDLE);
          response->success = true;
          response->message = enabled_ ? "observation mission enabled" : "observation mission stopped";
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(33), [this]() { tick(); });
    build_search_waypoints();
    reset(enabled_ ? Phase::WAIT_VEHICLE : Phase::IDLE);
    RCLCPP_INFO(
        get_logger(), "sensor mission ready observation=%s hydrophone=%s collector=%s output=%s dry_run=%s",
        observation_topic_.c_str(), hydrophone_direction_topic_.c_str(), collector_topic_.c_str(),
        output_topic_.c_str(), dry_run_ ? "true" : "false");
  }

 private:
  void reset(Phase phase) {
    phase_ = phase;
    phase_started_ = Clock::now();
    mission_started_ = phase_started_;
    phase_collector_sequence_ = collector_sequence_;
    detached_count_ = 0;
    netted_count_ = 0;
    released_count_ = 0;
    vision_state_.clear();
    vision_state_received_ = {};
    acknowledged_detach_count_ = 0;
    acknowledged_netted_count_ = 0;
    detached_target_ids_.clear();
    detached_contact_xy_.clear();
    netted_target_ids_.clear();
    released_target_ids_.clear();
    failure_reason_.clear();
    alignment_started_.reset();
    filter_initialized_ = false;
    filtered_error_x_ = 0.0;
    filtered_error_y_ = 0.0;
    filtered_height_ratio_ = 0.0;
    filtered_error_rate_x_ = 0.0;
    capture_heading_valid_ = false;
    capture_heading_yaw_rad_ = 0.0;
    capture_depth_z_ = 0.0;
    previous_command_ = Command{};
    previous_command_time_ = Clock::now();
    range_motion_samples_.clear();
    fit_previous_position_.reset();
    fitted_direction_world_.reset();
    fitted_pinger_xy_.reset();
    acoustic_position_samples_.clear();
    phase_range_position_samples_.clear();
    phase_odom_history_.clear();
    phase_fitted_pinger_xy_.reset();
    phase_seed_direction_world_.reset();
    phase_initial_acoustic_range_m_.reset();
    phase_range_origin_time_ = {};
    phase_position_fit_attempted_ = {};
    phase_fitted_position_received_ = {};
    cumulative_phase_range_change_ = 0.0;
    phase_fitted_initial_range_m_ = 0.0;
    phase_fitted_bias_range_rate_mps_ = 0.0;
    phase_fitted_position_condition_ = 0.0;
    phase_fitted_position_residual_ = 0.0;
    raw_delta_range_window_.clear();
    latest_raw_acoustic_range_m_.reset();
    acoustic_range_window_.clear();
    acoustic_range_m_.reset();
    pinger_leg_start_range_m_.reset();
    pinger_leg_best_range_m_.reset();
    pinger_leg_phase_range_valid_ = false;
    pinger_leg_phase_range_start_m_ = 0.0;
    pinger_leg_best_phase_range_change_m_ = 0.0;
    pinger_range_worsening_started_.reset();
    pinger_range_worsening_ = false;
    pinger_recovery_probe_ = false;
    pinger_spin_rehome_active_ = false;
    pinger_spin_watch_initialized_ = false;
    pinger_spin_accumulated_yaw_rad_ = 0.0;
    pinger_spin_rehome_count_ = 0;
    pinger_doppler_approach_count_ = 0;
    pinger_doppler_recede_count_ = 0;
    pinger_doppler_approach_confirmed_ = false;
    pinger_doppler_reversal_detected_ = false;
    non_pinger_detached_during_pinger_ = false;
    pending_delta_range_ = 0.0;
    pending_delta_count_ = 0;
    pinger_visual_acquired_ = false;
    pinger_visual_last_seen_ = {};
    pinger_detached_observed_ = false;
    pinger_started_.reset();
    pinger_path_previous_position_.reset();
    pinger_path_length_m_ = 0.0;
    pinger_completed_elapsed_s_ = 0.0;
    pinger_search_reentry_count_ = 0;
    pinger_recovery_count_ = 0;
    acoustic_candidate_xy_.reset();
    acoustic_candidate_streak_ = 0;
    pinger_acoustic_position_promoted_ = false;
    acoustic_candidate_condition_ = 0.0;
    acoustic_candidate_residual_ = 0.0;
    fitted_pinger_position_condition_ = 0.0;
    fitted_pinger_position_residual_ = 0.0;
    search_waypoint_index_ = 0;
    search_waypoint_initialized_ = false;
  }

  void transition(Phase next) {
    if (next == phase_) return;
    const bool was_pinger_phase = is_pinger_phase(phase_);
    const bool will_be_pinger_phase = is_pinger_phase(next);
    RCLCPP_INFO(get_logger(), "FSM %s -> %s", phase_name(phase_), phase_name(next));
    const Phase previous = phase_;
    phase_ = next;
    phase_started_ = Clock::now();
    if (next == Phase::PINGER_HOMING || next == Phase::PINGER_FINE_ALIGN) {
      begin_pinger_homing_leg();
      reset_pinger_spin_watchdog();
      pinger_spin_rehome_active_ = false;
    } else if (previous == Phase::PINGER_HOMING) {
      pinger_range_worsening_started_.reset();
    }
    if (next == Phase::PINGER_SEARCH && pinger_recovery_probe_) {
      // A rejected bearing is a braking event.  Do not let the generic slew
      // limiter carry the previous forward command into the recovery probe.
      previous_command_ = Command{};
      previous_command_time_ = phase_started_;
    }
    if (next == Phase::PINGER_SEARCH && is_pinger_phase(previous) &&
        previous != Phase::PINGER_SEARCH) {
      ++pinger_search_reentry_count_;
      if (pinger_recovery_probe_) ++pinger_recovery_count_;
    }
    if (will_be_pinger_phase && !was_pinger_phase) {
      pinger_started_ = phase_started_;
    } else if (!will_be_pinger_phase) {
      if (pinger_started_) pinger_completed_elapsed_s_ = age(*pinger_started_);
      pinger_started_.reset();
    }
    if (next == Phase::PINGER_CAPTURE || next == Phase::CAPTURE) {
      capture_heading_valid_ = odom_fresh();
      if (capture_heading_valid_) {
        capture_heading_yaw_rad_ = current_yaw_rad();
        capture_depth_z_ = odom_.pose.pose.position.z;
      }
    } else if (next != Phase::PINGER_VERIFY && next != Phase::PINGER_BACKOFF &&
               next != Phase::VERIFY_CAPTURE && next != Phase::BACKOFF) {
      capture_heading_valid_ = false;
    }
    if (next != Phase::FINE_ALIGN) alignment_started_.reset();
    if (next != Phase::VERIFY_CAPTURE && next != Phase::PINGER_VERIFY) {
      phase_collector_sequence_ = collector_sequence_;
    }
  }

  Phase underwater_entry_phase() const {
    // The full-mission FSM owns odometry-based lawnmower search. The imported
    // visual FSM is enabled only after a real target observation is accepted.
    return Phase::SEARCH;
  }

  bool vision_state_fresh() const {
    return !vision_state_.empty() && age(vision_state_received_) <= vision_state_timeout_s_;
  }

  void publish_vision_enable(bool active) {
    if (!vision_enable_pub_) return;
    std_msgs::msg::Bool message;
    message.data = active;
    vision_enable_pub_->publish(message);
  }

  bool observation_fresh() const {
    return age(observation_received_) <= observation_timeout_s_ && observation_.detected &&
           std::isfinite(observation_.normalized_error_x) &&
           std::isfinite(observation_.normalized_error_y) &&
           observation_.image_width > 0 && observation_.image_height > 0 &&
           observation_.confidence >= min_confidence_ && class_matches(observation_);
  }

  bool hydrophone_fresh() const {
    const bool direction_fresh =
        (use_phase_range_position_fusion_ && phase_range_position_fit_fresh()) ||
        (use_acoustic_position_fusion_ && pinger_position_fit_fresh()) ||
        (prefer_internal_hydrophone_direction_ && adaptive_direction_fit_fresh()) ||
        upstream_direction_fresh() ||
        (allow_internal_hydrophone_direction_fallback_ && fitted_direction_fresh());
    return direction_fresh && age(odom_received_) <= 1.5;
  }

  bool upstream_direction_fresh() const {
    return age(hydrophone_received_) <= hydrophone_timeout_s_;
  }

  bool pinger_yolo_fresh() const {
    // The detector publishes a candidate only when it matches the hydrophone bearing.
    // A generic buoy on the same bearing is not the pinger. Only hand control
    // to vision after acoustic ranging says the emitting source is nearby.
    if (!pinger_started_ || observation_received_ < *pinger_started_) return false;
    return acoustic_pinger_near() && pinger_visual_fresh();
  }

  bool pinger_visual_fresh() const {
    if (!observation_fresh() ||
        age(pinger_visual_last_seen_) > pinger_visual_reacquire_timeout_s_ ||
        bbox_ratio() < pinger_yolo_min_bbox_ratio_) {
      return false;
    }
    // Validate the initial vision handoff against the acoustic bearing.
    // Once vision owns the pinger, keep that lock independent of the slower
    // single-hydrophone EKF so a stale bearing cannot reject the right buoy.
    if (!pinger_visual_acquired_ && hydrophone_fresh()) {
      const auto direction_body = selected_hydrophone_direction_body();
      const double expected_image_bearing = -std::atan2(direction_body[1], direction_body[0]);
      const double filtered_image_bearing = filtered_error_x_ * 0.5 * pinger_camera_hfov_rad_;
      if (std::abs(wrap_pi(
              filtered_image_bearing - expected_image_bearing)) >
          pinger_visual_bearing_tolerance_rad_) {
        return false;
      }
    }
    return true;
  }

  bool acoustic_pinger_near() const {
    const auto fitted_range = dynamic_pinger_range_m();
    const bool amplitude_fresh = acoustic_range_fresh();
    const bool amplitude_near = amplitude_fresh &&
        *acoustic_range_m_ <= pinger_yolo_acoustic_range_m_;
    if (amplitude_fresh) {
      // The calibrated IQ range remains monotonic down to the receiver floor.
      // A weak multilateration geometry must not veto this direct measurement.
      return amplitude_near;
    }
    return fitted_range && *fitted_range <= pinger_yolo_acoustic_range_m_;
  }

  bool acoustic_range_fresh() const {
    return acoustic_range_m_ &&
           age(iq_magnitude_received_) <= hydrophone_timeout_s_;
  }

  void begin_pinger_homing_leg() {
    pinger_leg_start_range_m_.reset();
    pinger_leg_best_range_m_.reset();
    pinger_leg_phase_range_valid_ = age(delta_range_received_) <= hydrophone_timeout_s_;
    pinger_leg_phase_range_start_m_ = cumulative_phase_range_change_;
    pinger_leg_best_phase_range_change_m_ = 0.0;
    pinger_range_worsening_started_.reset();
    pinger_range_worsening_ = false;
    pinger_doppler_approach_count_ = 0;
    pinger_doppler_recede_count_ = 0;
    pinger_doppler_approach_confirmed_ = false;
    pinger_doppler_reversal_detected_ = false;
    raw_delta_range_window_.clear();
    if (acoustic_range_fresh()) {
      pinger_leg_start_range_m_ = *acoustic_range_m_;
      pinger_leg_best_range_m_ = *acoustic_range_m_;
    }
  }

  void update_pinger_doppler_trend(double delta_range_m) {
    if (!enabled_ ||
        (phase_ != Phase::PINGER_HOMING && phase_ != Phase::PINGER_FINE_ALIGN) ||
        previous_command_.forward <= 0.05) {
      return;
    }
    if (delta_range_m <= -pinger_doppler_approach_delta_m_) {
      pinger_doppler_approach_count_ = std::min(
          pinger_doppler_approach_count_ + 1, pinger_doppler_approach_samples_);
      pinger_doppler_recede_count_ = 0;
      if (pinger_doppler_approach_count_ >= pinger_doppler_approach_samples_) {
        pinger_doppler_approach_confirmed_ = true;
      }
      return;
    }
    if (pinger_doppler_approach_confirmed_ &&
        delta_range_m >= pinger_doppler_recede_delta_m_) {
      const auto fitted_range = dynamic_pinger_range_m();
      const bool near_closest_point =
          (acoustic_range_fresh() &&
           *acoustic_range_m_ <= pinger_doppler_reversal_max_range_m_) ||
          (fitted_range &&
           *fitted_range <= pinger_doppler_reversal_max_range_m_ + 0.5);
      if (!near_closest_point) {
        pinger_doppler_recede_count_ = 0;
        return;
      }
      ++pinger_doppler_recede_count_;
      if (pinger_doppler_recede_count_ >= pinger_doppler_recede_samples_) {
        pinger_doppler_reversal_detected_ = true;
      }
      return;
    }
    if (std::abs(delta_range_m) < pinger_doppler_recede_delta_m_) {
      pinger_doppler_recede_count_ = std::max(0, pinger_doppler_recede_count_ - 1);
    }
  }

  bool pinger_guidance_ready(double phase_age) const {
    if (!require_pinger_position_fit_) return true;
    if (prefer_upstream_hydrophone_direction_ && upstream_direction_fresh()) return true;
    if (dynamic_pinger_position_fit_fresh()) return true;
    if (prefer_internal_hydrophone_direction_ && adaptive_direction_fit_fresh()) return true;
    const double mission_pinger_age = pinger_started_ ? age(*pinger_started_) : phase_age;
    return mission_pinger_age >= pinger_max_probe_s_;
  }

  bool pinger_committed_guidance_fresh() const {
    return dynamic_pinger_position_fit_fresh() ||
           (prefer_internal_hydrophone_direction_ && adaptive_direction_fit_fresh());
  }

  double pinger_search_probe_hold_s() const {
    if (pinger_recovery_probe_) {
      // A fixed source position can safely turn the vehicle back as soon as
      // inertia is cancelled. Direction-only recovery needs a short lateral
      // baseline before another committed homing leg.
      if (dynamic_pinger_position_fit_fresh()) {
        return pinger_doppler_brake_s_ + 0.25;
      }
      return pinger_doppler_brake_s_ + 0.25 + 2.50;
    }
    if (pinger_committed_guidance_fresh()) return 0.20;
    if (pinger_started_ && age(*pinger_started_) >= pinger_max_probe_s_ &&
        upstream_direction_fresh()) {
      return 0.35;
    }
    return pinger_min_probe_s_;
  }

  bool pinger_homing_range_rejected(double phase_age, bool require_progress = true) {
    pinger_range_worsening_ = false;
    if (acoustic_range_fresh()) {
      if (!pinger_leg_start_range_m_) {
        pinger_leg_start_range_m_ = *acoustic_range_m_;
        pinger_leg_best_range_m_ = *acoustic_range_m_;
      } else if (!pinger_leg_best_range_m_ ||
                 *acoustic_range_m_ < *pinger_leg_best_range_m_) {
        pinger_leg_best_range_m_ = *acoustic_range_m_;
      }
    }

    double progress = 0.0;
    double regression = 0.0;
    const bool use_absolute_range =
        pinger_position_fit_geometrically_strong() && acoustic_range_fresh() &&
        pinger_leg_start_range_m_ && pinger_leg_best_range_m_;
    if (use_absolute_range) {
      progress = *pinger_leg_start_range_m_ - *acoustic_range_m_;
      regression = *acoustic_range_m_ - *pinger_leg_best_range_m_;
    } else {
      if (!pinger_leg_phase_range_valid_ &&
          age(delta_range_received_) <= hydrophone_timeout_s_) {
        pinger_leg_phase_range_valid_ = true;
        pinger_leg_phase_range_start_m_ = cumulative_phase_range_change_;
        pinger_leg_best_phase_range_change_m_ = 0.0;
      }
      if (!pinger_leg_phase_range_valid_ ||
          age(delta_range_received_) > hydrophone_timeout_s_) {
        pinger_range_worsening_started_.reset();
        return false;
      }
      const double relative_range_change =
          cumulative_phase_range_change_ - pinger_leg_phase_range_start_m_;
      pinger_leg_best_phase_range_change_m_ = std::min(
          pinger_leg_best_phase_range_change_m_, relative_range_change);
      progress = -relative_range_change;
      regression = relative_range_change - pinger_leg_best_phase_range_change_m_;
    }
    const bool regressed =
        phase_age >= pinger_range_progress_grace_s_ &&
        regression >= pinger_range_regression_margin_m_;
    const bool no_progress =
        require_progress && phase_age >= pinger_range_progress_check_s_ &&
        progress < pinger_range_min_progress_m_;
    if (progress >= pinger_range_min_progress_m_) {
      pinger_recovery_probe_ = false;
    }
    if (!regressed && !no_progress) {
      pinger_range_worsening_started_.reset();
      return false;
    }

    pinger_range_worsening_ = true;
    if (!pinger_range_worsening_started_) {
      pinger_range_worsening_started_ = Clock::now();
      return false;
    }
    if (age(*pinger_range_worsening_started_) < pinger_range_regression_hold_s_) {
      return false;
    }

    RCLCPP_WARN(
        get_logger(),
        "rejecting stale pinger bearing: source=%s progress=%.2fm regression=%.2fm acoustic=%.2fm",
        use_absolute_range ? "absolute_range" : "phase_delta_range",
        progress, regression, acoustic_range_m_ ? *acoustic_range_m_ : -1.0);
    pinger_recovery_probe_ = true;
    return true;
  }

  double pinger_leg_progress_m() const {
    if (!pinger_leg_phase_range_valid_) return 0.0;
    return -(cumulative_phase_range_change_ - pinger_leg_phase_range_start_m_);
  }

  double pinger_leg_regression_m() const {
    if (!pinger_leg_phase_range_valid_) return 0.0;
    return cumulative_phase_range_change_ - pinger_leg_phase_range_start_m_ -
           pinger_leg_best_phase_range_change_m_;
  }

  std::optional<double> dynamic_pinger_range_m() const {
    if (!odom_fresh()) return std::nullopt;
    const Eigen::Vector2d vehicle_xy(
        odom_.pose.pose.position.x, odom_.pose.pose.position.y);
    std::optional<Eigen::Vector2d> source_xy;
    if (use_phase_range_position_fusion_ && phase_range_position_fit_fresh()) {
      source_xy = phase_fitted_pinger_xy_;
    } else if (use_acoustic_position_fusion_ &&
               (pinger_position_fit_geometrically_strong() ||
                pinger_position_fit_fresh())) {
      source_xy = fitted_pinger_xy_;
    }
    if (!source_xy) return std::nullopt;
    const double dz = pinger_acoustic_source_depth_z_ - odom_.pose.pose.position.z;
    return std::sqrt((*source_xy - vehicle_xy).squaredNorm() + dz * dz);
  }

  std::optional<double> pinger_control_range_m() const {
    if (acoustic_range_fresh()) return acoustic_range_m_;
    return dynamic_pinger_range_m();
  }

  double bbox_ratio() const {
    return filter_initialized_ ? filtered_height_ratio_ : 0.0;
  }

  bool class_matches(const hit25_auv_ros2_msg::msg::BuoyObservation &observation) const {
    const auto *accepted_names = is_pinger_phase(phase_)
        ? &pinger_visual_class_names_
        : (is_underwater_buoy_phase(phase_) ? &underwater_visual_class_names_ : nullptr);
    if (accepted_names != nullptr) {
      std::string label = observation.class_label;
      std::transform(label.begin(), label.end(), label.begin(),
                     [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
      return std::find(
                 accepted_names->begin(), accepted_names->end(), label) !=
             accepted_names->end();
    }
    return target_class_name_.empty() || observation.class_label == target_class_name_;
  }

  void update_filtered_observation(
      const hit25_auv_ros2_msg::msg::BuoyObservation &observation) {
    if (!observation.detected || observation.image_height == 0 ||
        !std::isfinite(observation.normalized_error_x) ||
        !std::isfinite(observation.normalized_error_y) ||
        observation.confidence < min_confidence_ || !class_matches(observation)) {
      return;
    }
    const double error_x = clamp(observation.normalized_error_x, -1.0, 1.0);
    const double error_y = clamp(observation.normalized_error_y, -1.0, 1.0);
    const double height_ratio = clamp(
        static_cast<double>(observation.bbox_height) / observation.image_height, 0.0, 1.0);
    if (!filter_initialized_) {
      filtered_error_x_ = error_x;
      filtered_error_y_ = error_y;
      filtered_height_ratio_ = height_ratio;
      observation_filter_time_ = Clock::now();
      filter_initialized_ = true;
      return;
    }
    const auto now = Clock::now();
    const double dt = clamp(
        std::chrono::duration<double>(now - observation_filter_time_).count(), 1.0 / 120.0, 0.5);
    const double alpha = clamp(observation_filter_alpha_, 0.05, 1.0);
    const double previous = filtered_error_x_;
    filtered_error_x_ += alpha * (error_x - filtered_error_x_);
    filtered_error_y_ += alpha * (error_y - filtered_error_y_);
    filtered_height_ratio_ += alpha * (height_ratio - filtered_height_ratio_);
    filtered_error_rate_x_ = clamp((filtered_error_x_ - previous) / dt, -2.0, 2.0);
    observation_filter_time_ = now;
  }

  void update_range_motion_fit(const nav_msgs::msg::Odometry & msg) {
    const auto now_stamp = Clock::now();
    const Eigen::Vector3d position(
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    update_acoustic_position_fit(position);
    if (fit_previous_position_ && fit_previous_odom_time_.time_since_epoch().count() != 0) {
      const double dt = std::chrono::duration<double>(now_stamp - fit_previous_odom_time_).count();
      const Eigen::Vector3d delta_position = position - *fit_previous_position_;
      if (pending_delta_count_ > 0 && dt > 1.0e-3 && dt < 2.0 &&
          delta_position.norm() >= 0.003 && delta_position.norm() <= 1.5) {
        range_motion_samples_.push_back(
            RangeMotionSample{delta_position, pending_delta_range_, dt});
        while (range_motion_samples_.size() > 80) range_motion_samples_.pop_front();
        fit_range_motion_direction();
      }
    }
    fit_previous_position_ = position;
    fit_previous_odom_time_ = now_stamp;
    pending_delta_range_ = 0.0;
    pending_delta_count_ = 0;
  }

  void update_phase_range_position_fit() {
    if (!use_phase_range_position_fusion_ || age(odom_received_) > 1.5) return;
    const auto now_stamp = Clock::now();
    const auto measurement_stamp = now_stamp - std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(std::max(0.0, phase_range_measurement_delay_s_)));
    const auto delayed_position = phase_position_at(measurement_stamp);
    if (!delayed_position) return;
    if (phase_range_origin_time_.time_since_epoch().count() == 0) {
      phase_range_origin_time_ = measurement_stamp;
    }
    const Eigen::Vector3d position = *delayed_position;
    const double elapsed =
        std::chrono::duration<double>(measurement_stamp - phase_range_origin_time_).count();
    const bool position_changed = phase_range_position_samples_.empty() ||
        (position - phase_range_position_samples_.back().vehicle_position).norm() >= 0.008;
    const bool time_changed = phase_range_position_samples_.empty() ||
        elapsed - phase_range_position_samples_.back().elapsed_s >= 0.20;
    if (position_changed || time_changed) {
      phase_range_position_samples_.push_back(
          PhaseRangePositionSample{position, cumulative_phase_range_change_, elapsed});
      while (phase_range_position_samples_.size() > 360) {
        phase_range_position_samples_.pop_front();
      }
    }
    const double sample_duration = phase_range_position_samples_.empty()
        ? 0.0
        : phase_range_position_samples_.back().elapsed_s -
            phase_range_position_samples_.front().elapsed_s;
    if (phase_fitted_pinger_xy_) return;
    if (phase_range_position_samples_.size() >= 30 &&
        sample_duration >= phase_range_min_fit_duration_s_ &&
        age(phase_position_fit_attempted_) >= 0.75) {
      phase_position_fit_attempted_ = now_stamp;
      fit_phase_range_pinger_position();
    }
  }

  void record_phase_odom_position(const nav_msgs::msg::Odometry &msg) {
    phase_odom_history_.push_back({
        Clock::now(),
        Eigen::Vector3d(
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)});
    while (phase_odom_history_.size() > 240) phase_odom_history_.pop_front();
  }

  std::optional<Eigen::Vector3d> phase_position_at(
      const Clock::time_point &target_stamp) const {
    if (phase_odom_history_.size() < 2 ||
        target_stamp < phase_odom_history_.front().stamp ||
        target_stamp > phase_odom_history_.back().stamp) {
      return std::nullopt;
    }
    for (std::size_t index = 1; index < phase_odom_history_.size(); ++index) {
      const auto &before = phase_odom_history_[index - 1];
      const auto &after = phase_odom_history_[index];
      if (target_stamp > after.stamp) continue;
      const double interval = std::chrono::duration<double>(after.stamp - before.stamp).count();
      if (interval <= 1.0e-9) return before.position;
      const double alpha = clamp(
          std::chrono::duration<double>(target_stamp - before.stamp).count() / interval,
          0.0, 1.0);
      return before.position + alpha * (after.position - before.position);
    }
    return std::nullopt;
  }

  void fit_phase_range_pinger_position() {
    if (phase_range_position_samples_.size() < 30) return;
    const auto &reference = phase_range_position_samples_.front();
    const Eigen::Vector3d origin = reference.vehicle_position;
    const double fixed_source_z_local = pinger_acoustic_source_depth_z_ - origin.z();

    Eigen::MatrixXd centered_xy(phase_range_position_samples_.size(), 2);
    Eigen::Vector2d mean_xy = Eigen::Vector2d::Zero();
    for (const auto &sample : phase_range_position_samples_) {
      mean_xy += sample.vehicle_position.head<2>();
    }
    mean_xy /= static_cast<double>(phase_range_position_samples_.size());
    double max_displacement = 0.0;
    Eigen::Index geometry_row = 0;
    for (const auto &sample : phase_range_position_samples_) {
      centered_xy.row(geometry_row++) =
          (sample.vehicle_position.head<2>() - mean_xy).transpose();
      max_displacement = std::max(
          max_displacement, (sample.vehicle_position - origin).norm());
    }
    if (max_displacement < 0.15) return;
    const Eigen::JacobiSVD<Eigen::MatrixXd> geometry_svd(
        centered_xy, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto geometry_singular = geometry_svd.singularValues();
    if (geometry_singular.size() < 2 || geometry_singular(1) < 0.03) return;

    const double q0 = reference.cumulative_range_change;
    const double t0 = reference.elapsed_s;
    constexpr double huber_scale_m = 0.005;
    constexpr double bias_sigma_mps = 0.04;
    auto evaluate = [&](const Eigen::Vector4d &params, Eigen::Matrix4d *normal,
                        Eigen::Vector4d *gradient) {
      if (normal) normal->setZero();
      if (gradient) gradient->setZero();
      double cost = 0.0;
      for (const auto &sample : phase_range_position_samples_) {
        const Eigen::Vector3d displacement = sample.vehicle_position - origin;
        const Eigen::Vector3d relative(
            params(0) - displacement.x(), params(1) - displacement.y(),
            fixed_source_z_local - displacement.z());
        const double predicted_range = std::max(relative.norm(), 1.0e-8);
        const double q = sample.cumulative_range_change - q0;
        const double t = sample.elapsed_s - t0;
        const double residual = predicted_range - params(2) + params(3) * t - q;
        const double magnitude = std::abs(residual);
        const double weight = magnitude > huber_scale_m
            ? huber_scale_m / std::max(magnitude, 1.0e-12)
            : 1.0;
        cost += magnitude <= huber_scale_m
            ? 0.5 * residual * residual
            : huber_scale_m * (magnitude - 0.5 * huber_scale_m);
        if (normal && gradient) {
          Eigen::Vector4d jacobian;
          jacobian << relative.x() / predicted_range, relative.y() / predicted_range,
              -1.0, t;
          *normal += weight * jacobian * jacobian.transpose();
          *gradient += weight * jacobian * residual;
        }
      }
      const double bias_weight = 1.0 / (bias_sigma_mps * bias_sigma_mps);
      cost += 0.5 * params(3) * params(3) * bias_weight;
      if (normal && gradient) {
        (*normal)(3, 3) += bias_weight;
        (*gradient)(3) += params(3) * bias_weight;
      }
      return cost;
    };

    std::vector<Eigen::Vector4d> seeds;
    Eigen::Vector2d seed_direction(
        phase_seed_direction_world_ ? phase_seed_direction_world_->x() : hydrophone_direction_[0],
        phase_seed_direction_world_ ? phase_seed_direction_world_->y() : hydrophone_direction_[1]);
    if (seed_direction.norm() < 1.0e-6) seed_direction = Eigen::Vector2d(1.0, 0.0);
    seed_direction.normalize();
    const double seed_range = phase_initial_acoustic_range_m_
        ? clamp(*phase_initial_acoustic_range_m_, 0.5, 80.0)
        : 15.0;
    const double horizontal_range = std::sqrt(std::max(
        seed_range * seed_range - fixed_source_z_local * fixed_source_z_local, 0.25));
    seeds.emplace_back(
        seed_direction.x() * horizontal_range,
        seed_direction.y() * horizontal_range,
        std::sqrt(horizontal_range * horizontal_range +
            fixed_source_z_local * fixed_source_z_local),
        0.0);

    std::optional<Eigen::Vector4d> best_params;
    double best_cost = std::numeric_limits<double>::infinity();
    for (auto params : seeds) {
      if (!params.allFinite()) continue;
      double damping = 1.0e-3;
      for (int iteration = 0; iteration < 35; ++iteration) {
        Eigen::Matrix4d normal;
        Eigen::Vector4d gradient;
        const double current_cost = evaluate(params, &normal, &gradient);
        const Eigen::Matrix4d damped = normal + damping * Eigen::Matrix4d::Identity();
        const Eigen::Vector4d step = damped.ldlt().solve(-gradient);
        if (!step.allFinite()) break;
        Eigen::Vector4d trial = params + step;
        trial(0) = clamp(trial(0), -100.0 - origin.x(), 100.0 - origin.x());
        trial(1) = clamp(trial(1), -100.0 - origin.y(), 100.0 - origin.y());
        trial(2) = clamp(trial(2), 0.10, 80.0);
        trial(3) = clamp(trial(3), -0.50, 0.50);
        const double trial_cost = evaluate(trial, nullptr, nullptr);
        if (std::isfinite(trial_cost) && trial_cost <= current_cost) {
          params = trial;
          damping = std::max(1.0e-7, damping * 0.4);
          if (step.norm() < 1.0e-5) break;
        } else {
          damping = std::min(1.0e3, damping * 8.0);
        }
      }
      const double cost = evaluate(params, nullptr, nullptr);
      if (params.allFinite() && std::isfinite(cost) && cost < best_cost) {
        best_cost = cost;
        best_params = params;
      }
    }
    if (!best_params) return;

    Eigen::MatrixXd final_jacobian(phase_range_position_samples_.size(), 4);
    double squared_residual = 0.0;
    Eigen::Index row = 0;
    for (const auto &sample : phase_range_position_samples_) {
      const Eigen::Vector3d displacement = sample.vehicle_position - origin;
      const Eigen::Vector3d relative(
          (*best_params)(0) - displacement.x(), (*best_params)(1) - displacement.y(),
          fixed_source_z_local - displacement.z());
      const double predicted_range = std::max(relative.norm(), 1.0e-8);
      const double q = sample.cumulative_range_change - q0;
      const double t = sample.elapsed_s - t0;
      const double residual =
          predicted_range - (*best_params)(2) + (*best_params)(3) * t - q;
      squared_residual += residual * residual;
      final_jacobian.row(row++) << relative.x() / predicted_range,
          relative.y() / predicted_range, -1.0, t;
    }
    const double rms_residual = std::sqrt(
        squared_residual / static_cast<double>(phase_range_position_samples_.size()));
    const Eigen::JacobiSVD<Eigen::MatrixXd> fit_svd(
        final_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto singular = fit_svd.singularValues();
    if (singular.size() < 4 || singular(3) <= 1.0e-9) return;
    const double condition = singular(0) / singular(3);
    const Eigen::Vector2d estimate = origin.head<2>() + best_params->head<2>();
    const Eigen::Vector2d initial_ray = estimate - origin.head<2>();
    const double alignment = initial_ray.norm() > 1.0e-6
        ? initial_ray.normalized().dot(seed_direction)
        : -1.0;
    if (!estimate.allFinite() || estimate.norm() > 100.0 ||
        !std::isfinite(condition) || condition > 2.0e5 ||
        !std::isfinite(rms_residual) || rms_residual > 0.20 || alignment < 0.82) {
      return;
    }
    phase_fitted_pinger_xy_ = estimate;
    phase_fitted_initial_range_m_ = (*best_params)(2);
    phase_fitted_bias_range_rate_mps_ = (*best_params)(3);
    phase_fitted_position_condition_ = condition;
    phase_fitted_position_residual_ = rms_residual;
    phase_fitted_position_received_ = Clock::now();
  }

  void update_acoustic_position_fit(const Eigen::Vector3d &vehicle_position) {
    if (!latest_raw_acoustic_range_m_ || age(raw_acoustic_range_received_) > 0.35) return;
    if (fitted_pinger_xy_) {
      const double dz = pinger_acoustic_source_depth_z_ - vehicle_position.z();
      const double fitted_range = std::sqrt(
          (*fitted_pinger_xy_ - vehicle_position.head<2>()).squaredNorm() + dz * dz);
      const bool inside_locked_near_field =
          pinger_acoustic_position_lock_range_m_ > 0.0 &&
          fitted_range <= pinger_acoustic_position_lock_range_m_ &&
          (pinger_position_fit_geometrically_strong() ||
           pinger_position_fit_direction_consistent());
      const bool amplitude_range_saturated =
          pinger_acoustic_position_min_range_m_ > 0.0 &&
          *latest_raw_acoustic_range_m_ <= pinger_acoustic_position_min_range_m_;
      if (inside_locked_near_field || amplitude_range_saturated) {
        // IQ amplitude is range-like only outside the receiver's saturation
        // floor. Preserve the last geometrically observable source estimate
        // while vision performs the final rake approach.
        fitted_pinger_position_received_ = Clock::now();
        return;
      }
    }
    const Eigen::Vector2d xy = vehicle_position.head<2>();
    if (!acoustic_position_samples_.empty() &&
        (xy - acoustic_position_samples_.back().vehicle_xy).norm() < 0.05) {
      return;
    }
    acoustic_position_samples_.push_back({xy, vehicle_position.z(), *latest_raw_acoustic_range_m_});
    while (acoustic_position_samples_.size() > 120) acoustic_position_samples_.pop_front();
    fit_acoustic_pinger_position();
  }

  void fit_acoustic_pinger_position() {
    if (acoustic_position_samples_.size() < 20) return;
    const auto &reference = acoustic_position_samples_.front();
    const double reference_dz = pinger_acoustic_source_depth_z_ - reference.vehicle_z;
    const double reference_horizontal_sq =
        reference.slant_range * reference.slant_range - reference_dz * reference_dz;
    if (reference_horizontal_sq <= 0.01) return;
    Eigen::MatrixXd design(acoustic_position_samples_.size() - 1, 2);
    Eigen::VectorXd measured(acoustic_position_samples_.size() - 1);
    Eigen::Index row = 0;
    for (std::size_t index = 1; index < acoustic_position_samples_.size(); ++index) {
      const auto &sample = acoustic_position_samples_[index];
      const double sample_dz = pinger_acoustic_source_depth_z_ - sample.vehicle_z;
      const double horizontal_sq =
          sample.slant_range * sample.slant_range - sample_dz * sample_dz;
      if (horizontal_sq <= 0.01) continue;
      const Eigen::Vector2d delta = sample.vehicle_xy - reference.vehicle_xy;
      design.row(row) = 2.0 * delta.transpose();
      measured(row) = sample.vehicle_xy.squaredNorm() - reference.vehicle_xy.squaredNorm() -
          horizontal_sq + reference_horizontal_sq;
      ++row;
    }
    if (row < 16) return;
    design.conservativeResize(row, Eigen::NoChange);
    measured.conservativeResize(row);
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        design, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto singular = svd.singularValues();
    if (singular.size() < 2 || singular(1) < 0.12) return;
    if (!std::isfinite(singular(0)) || !std::isfinite(singular(1))) return;
    Eigen::Vector2d estimate = svd.solve(measured);
    if (!estimate.allFinite() || estimate.norm() > 100.0) return;

    // Absolute IQ ranges contain burst interference and are not well served by
    // the squared-range linear solution alone. Refine it with a Huber-weighted
    // fixed-depth range model before the estimate is allowed to steer.
    constexpr double huber_scale_m = 0.20;
    constexpr double damping = 1.0e-3;
    for (int iteration = 0; iteration < 25; ++iteration) {
      Eigen::Matrix2d normal = Eigen::Matrix2d::Zero();
      Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
      for (const auto &sample : acoustic_position_samples_) {
        const double dz = pinger_acoustic_source_depth_z_ - sample.vehicle_z;
        const Eigen::Vector2d horizontal = estimate - sample.vehicle_xy;
        const double predicted = std::max(
            std::sqrt(horizontal.squaredNorm() + dz * dz), 1.0e-8);
        const double residual = predicted - sample.slant_range;
        const double weight = std::abs(residual) > huber_scale_m
            ? huber_scale_m / std::max(std::abs(residual), 1.0e-12)
            : 1.0;
        const Eigen::Vector2d jacobian = horizontal / predicted;
        normal += weight * jacobian * jacobian.transpose();
        gradient += weight * jacobian * residual;
      }
      const Eigen::Vector2d step =
          (normal + damping * Eigen::Matrix2d::Identity()).ldlt().solve(-gradient);
      if (!step.allFinite()) return;
      estimate += step;
      if (step.norm() < 1.0e-5) break;
    }
    if (!estimate.allFinite() || estimate.norm() > 100.0) return;

    std::vector<double> residuals;
    residuals.reserve(acoustic_position_samples_.size());
    Eigen::MatrixXd jacobian(acoustic_position_samples_.size(), 2);
    double squared_residual = 0.0;
    Eigen::Index jacobian_row = 0;
    for (const auto &sample : acoustic_position_samples_) {
      const double dz = pinger_acoustic_source_depth_z_ - sample.vehicle_z;
      const Eigen::Vector2d horizontal = estimate - sample.vehicle_xy;
      const double predicted = std::sqrt(
          horizontal.squaredNorm() + dz * dz);
      const double residual = predicted - sample.slant_range;
      residuals.push_back(std::abs(residual));
      squared_residual += residual * residual;
      jacobian.row(jacobian_row++) =
          (horizontal / std::max(predicted, 1.0e-8)).transpose();
    }
    const Eigen::JacobiSVD<Eigen::MatrixXd> refined_svd(
        jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto refined_singular = refined_svd.singularValues();
    if (refined_singular.size() < 2 || refined_singular(1) <= 1.0e-6) return;
    const double condition = refined_singular(0) / refined_singular(1);
    std::nth_element(
        residuals.begin(), residuals.begin() + residuals.size() / 2, residuals.end());
    const double median_residual = residuals[residuals.size() / 2];
    const double rms_residual = std::sqrt(
        squared_residual / static_cast<double>(acoustic_position_samples_.size()));
    acoustic_candidate_condition_ = condition;
    acoustic_candidate_residual_ = median_residual;
    if (!std::isfinite(condition) || condition > 100.0 ||
        !std::isfinite(median_residual) || median_residual > 0.35 ||
        !std::isfinite(rms_residual) || rms_residual > 0.70) {
      acoustic_candidate_streak_ = 0;
      acoustic_candidate_xy_.reset();
      return;
    }

    if (!acoustic_candidate_xy_ ||
        (estimate - *acoustic_candidate_xy_).norm() > 0.75) {
      acoustic_candidate_xy_ = estimate;
      acoustic_candidate_streak_ = 1;
      return;
    }
    *acoustic_candidate_xy_ = 0.60 * *acoustic_candidate_xy_ + 0.40 * estimate;
    ++acoustic_candidate_streak_;
    if (acoustic_candidate_streak_ < 3) return;

    fitted_pinger_position_condition_ = condition;
    fitted_pinger_position_residual_ = median_residual;
    fitted_pinger_xy_ = fitted_pinger_xy_
        ? 0.65 * *fitted_pinger_xy_ + 0.35 * *acoustic_candidate_xy_
        : *acoustic_candidate_xy_;
    fitted_pinger_position_received_ = Clock::now();
    if (!pinger_acoustic_position_promoted_ &&
        acoustic_position_samples_.size() >= 30 &&
        acoustic_candidate_streak_ >= 5 &&
        condition <= 20.0 && median_residual <= 0.15) {
      pinger_acoustic_position_promoted_ = true;
      RCLCPP_INFO(
          get_logger(),
          "promoting acoustic pinger position: xy=(%.2f, %.2f) condition=%.2f residual=%.3fm samples=%zu",
          fitted_pinger_xy_->x(), fitted_pinger_xy_->y(), condition,
          median_residual, acoustic_position_samples_.size());
    }
  }

  void fit_range_motion_direction() {
    if (range_motion_samples_.size() < 12) return;
    Eigen::MatrixXd design(range_motion_samples_.size(), 4);
    Eigen::VectorXd measured(range_motion_samples_.size());
    for (std::size_t index = 0; index < range_motion_samples_.size(); ++index) {
      const auto & sample = range_motion_samples_[index];
      design.row(index) << -sample.delta_position.x(), -sample.delta_position.y(),
          -sample.delta_position.z(), sample.delta_time;
      measured(static_cast<Eigen::Index>(index)) = sample.delta_range;
    }
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        design, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto singular = svd.singularValues();
    if (singular.size() < 4 || singular(3) < 1.0e-5) return;
    const double condition = singular(0) / singular(3);
    if (!std::isfinite(condition) || condition > 2.0e5) return;
    const Eigen::Vector4d solution = svd.solve(measured);
    Eigen::Vector3d direction = solution.head<3>();
    const double norm = direction.norm();
    if (!direction.allFinite() || norm < 0.20 || norm > 3.0) return;
    direction /= norm;
    const double residual = (design * solution - measured).norm() /
        std::sqrt(static_cast<double>(range_motion_samples_.size()));
    if (!std::isfinite(residual) || residual > 0.05) return;
    if (!fitted_direction_world_) {
      fitted_direction_world_ = direction;
    } else {
      Eigen::Vector3d filtered = 0.75 * *fitted_direction_world_ + 0.25 * direction;
      if (filtered.norm() > 1.0e-6) fitted_direction_world_ = filtered.normalized();
    }
    fitted_direction_received_ = Clock::now();
    fitted_direction_condition_ = condition;
    fitted_direction_residual_ = residual;
  }

  std::array<double, 3> selected_hydrophone_direction() const {
    // A well-conditioned single-sensor range fit is dynamic in the odometry
    // frame, while the upstream far-field EKF can lag after the vehicle passes
    // its original bearing line. Only promote the range fit after it has enough
    // spatial baseline and a tight residual; otherwise preserve the upstream
    // estimator as the primary direction source.
    if (use_acoustic_position_fusion_ && pinger_position_fit_geometrically_strong() &&
        age(odom_received_) <= 1.5) {
      Eigen::Vector3d direction(
          fitted_pinger_xy_->x() - odom_.pose.pose.position.x,
          fitted_pinger_xy_->y() - odom_.pose.pose.position.y,
          pinger_acoustic_source_depth_z_ - odom_.pose.pose.position.z);
      if (direction.norm() > 1.0e-6) {
        direction.normalize();
        return {{direction.x(), direction.y(), direction.z()}};
      }
    }
    if (prefer_upstream_hydrophone_direction_ && upstream_direction_fresh()) {
      return hydrophone_direction_;
    }
    // The phase-range fit locks the stationary source in the odometry frame, then
    // recomputes its relative bearing on every control tick. IQ amplitude is not
    // used by this fit because coherent magnitude is motion/Doppler sensitive.
    if (use_acoustic_position_fusion_ && pinger_position_fit_fresh() &&
        age(odom_received_) <= 1.5) {
      Eigen::Vector3d direction(
          fitted_pinger_xy_->x() - odom_.pose.pose.position.x,
          fitted_pinger_xy_->y() - odom_.pose.pose.position.y,
          pinger_acoustic_source_depth_z_ - odom_.pose.pose.position.z);
      if (direction.norm() > 1.0e-6) {
        direction.normalize();
        return {{direction.x(), direction.y(), direction.z()}};
      }
    }
    if (use_phase_range_position_fusion_ && phase_range_position_fit_fresh() &&
        age(odom_received_) <= 1.5) {
      Eigen::Vector3d direction(
          phase_fitted_pinger_xy_->x() - odom_.pose.pose.position.x,
          phase_fitted_pinger_xy_->y() - odom_.pose.pose.position.y,
          pinger_acoustic_source_depth_z_ - odom_.pose.pose.position.z);
      if (direction.norm() > 1.0e-6) {
        direction.normalize();
        return {{direction.x(), direction.y(), direction.z()}};
      }
    }
    if (prefer_internal_hydrophone_direction_ && adaptive_direction_fit_fresh()) {
      return {{
          fitted_direction_world_->x(),
          fitted_direction_world_->y(),
          fitted_direction_world_->z()}};
    }
    if (upstream_direction_fresh()) return hydrophone_direction_;
    if (allow_internal_hydrophone_direction_fallback_ && fitted_direction_world_ &&
        age(fitted_direction_received_) <= hydrophone_timeout_s_) {
      return {{
          fitted_direction_world_->x(),
          fitted_direction_world_->y(),
          fitted_direction_world_->z()}};
    }
    return hydrophone_direction_;
  }

  const char *selected_hydrophone_source() const {
    if (use_acoustic_position_fusion_ && pinger_position_fit_geometrically_strong()) {
      return "acoustic_position_fusion";
    }
    if (prefer_upstream_hydrophone_direction_ && upstream_direction_fresh()) {
      return "upstream_ekf";
    }
    if (use_acoustic_position_fusion_ && pinger_position_fit_fresh()) {
      return "acoustic_position_fusion";
    }
    if (use_phase_range_position_fusion_ && phase_range_position_fit_fresh()) {
      return "phase_range_position_fusion";
    }
    if (prefer_internal_hydrophone_direction_ && adaptive_direction_fit_fresh()) {
      return "adaptive_range_motion_fit";
    }
    if (upstream_direction_fresh()) return "upstream_ekf";
    if (!allow_internal_hydrophone_direction_fallback_) return "unavailable";
    if (pinger_position_fit_fresh()) return "acoustic_multilateration_fallback";
    if (fitted_direction_fresh()) return "range_motion_fit_fallback";
    return "unavailable";
  }

  bool fitted_direction_fresh() const {
    return dynamic_pinger_position_fit_fresh() ||
           adaptive_direction_fit_fresh();
  }

  bool adaptive_direction_fit_fresh() const {
    return fitted_direction_world_.has_value() &&
           age(fitted_direction_received_) <= hydrophone_timeout_s_ &&
           fitted_direction_condition_ <= 2.0e5 &&
           fitted_direction_residual_ <= 0.05;
  }

  bool phase_range_position_fit_fresh() const {
    return phase_fitted_pinger_xy_.has_value() &&
           age(phase_fitted_position_received_) <= phase_range_position_timeout_s_ &&
           phase_fitted_position_condition_ <= 2.0e5 &&
           phase_fitted_position_residual_ <= 0.20;
  }

  bool dynamic_pinger_position_fit_fresh() const {
    return (use_phase_range_position_fusion_ && phase_range_position_fit_fresh()) ||
           (use_acoustic_position_fusion_ &&
            (pinger_position_fit_geometrically_strong() || pinger_position_fit_fresh()));
  }

  bool pinger_position_fit_fresh() const {
    return fitted_pinger_xy_.has_value() &&
           age(fitted_pinger_position_received_) <= pinger_position_fit_timeout_s_ &&
           fitted_pinger_position_condition_ <= 50.0 &&
           fitted_pinger_position_residual_ <= 0.35 &&
           pinger_position_fit_direction_consistent();
  }

  bool pinger_position_fit_geometrically_strong() const {
    if (!fitted_pinger_xy_ ||
        age(fitted_pinger_position_received_) > pinger_position_fit_timeout_s_) {
      return false;
    }
    const bool strict_quality =
        acoustic_position_samples_.size() >= 30 &&
        acoustic_candidate_streak_ >= 5 &&
        fitted_pinger_position_condition_ <= 20.0 &&
        fitted_pinger_position_residual_ <= 0.15;
    const bool promoted_quality =
        pinger_acoustic_position_promoted_ &&
        fitted_pinger_position_condition_ <= 50.0 &&
        fitted_pinger_position_residual_ <= 0.30;
    return strict_quality || promoted_quality;
  }

  bool pinger_position_fit_direction_consistent() const {
    if (!fitted_pinger_xy_ || !upstream_direction_fresh() || !odom_fresh()) return true;
    if (hydrophone_direction_frame_ == "body") return true;
    Eigen::Vector3d fitted_direction(
        fitted_pinger_xy_->x() - odom_.pose.pose.position.x,
        fitted_pinger_xy_->y() - odom_.pose.pose.position.y,
        pinger_acoustic_source_depth_z_ - odom_.pose.pose.position.z);
    if (fitted_direction.norm() <= 1.0e-6) return false;
    fitted_direction.normalize();
    const Eigen::Vector3d upstream(
        hydrophone_direction_[0], hydrophone_direction_[1], hydrophone_direction_[2]);
    const double tolerance = clamp(pinger_position_fit_bearing_tolerance_rad_, 0.0, 3.14159);
    return fitted_direction.dot(upstream) >= std::cos(tolerance);
  }

  bool new_collector_event() const {
    return collector_sequence_ > phase_collector_sequence_ && age(collector_received_) < 1.5;
  }

  bool collector_target_is_pinger() const {
    std::string target = collector_.target_id;
    std::transform(target.begin(), target.end(), target.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    return target.find("pinger") != std::string::npos;
  }

  bool remember_detached_target() {
    const std::string id = collector_.target_id.empty()
        ? "detached_event_" + std::to_string(collector_sequence_)
        : collector_.target_id;
    const bool inserted = detached_target_ids_.insert(id).second;
    if (inserted) remember_detach_contact();
    detached_count_ = static_cast<int>(detached_target_ids_.size());
    return inserted;
  }

  void remember_detach_contact() {
    if (!odom_fresh()) return;
    const Eigen::Vector2d contact(
        odom_.pose.pose.position.x, odom_.pose.pose.position.y);
    const bool duplicate = std::any_of(
        detached_contact_xy_.begin(), detached_contact_xy_.end(),
        [&contact](const Eigen::Vector2d &existing) {
          return (existing - contact).norm() < 0.25;
        });
    if (!duplicate) detached_contact_xy_.push_back(contact);
  }

  bool remember_netted_target() {
    const std::string id = collector_.target_id.empty()
        ? "netted_event_" + std::to_string(collector_sequence_)
        : collector_.target_id;
    const bool inserted = netted_target_ids_.insert(id).second;
    netted_count_ = static_cast<int>(netted_target_ids_.size());
    return inserted;
  }

  bool remember_released_target() {
    const std::string id = collector_.target_id.empty()
        ? "released_event_" + std::to_string(collector_sequence_)
        : collector_.target_id;
    const bool inserted = released_target_ids_.insert(id).second;
    released_count_ = static_cast<int>(released_target_ids_.size());
    return inserted;
  }

  bool connection_grace_active() const {
    return vehicle_disconnect_grace_s_ > 0.0 && !vehicle_state_.connected &&
           age(last_vehicle_connected_) <= vehicle_disconnect_grace_s_;
  }

  bool vehicle_connected_effective() const {
    return vehicle_state_.connected || connection_grace_active();
  }

  bool vehicle_armed_effective() const {
    if (vehicle_state_.connected) return vehicle_state_.armed;
    return connection_grace_active() &&
           age(last_vehicle_armed_) <= vehicle_disconnect_grace_s_;
  }

  bool vehicle_ready() const {
    return age(state_received_) < state_timeout_s_ && vehicle_connected_effective();
  }

  bool odom_fresh() const { return age(odom_received_) < odom_timeout_s_; }

  void fail(const std::string &reason) {
    failure_reason_ = reason;
    transition(Phase::FAILED);
  }

  Command visual_command(
      double forward, double target_error_x = 0.0, double target_error_y = 0.0) const {
    const auto guidance = kmu26_mission_fsm::vision_buoy::alignment_command(
        filtered_error_x_,
        filtered_error_y_,
        filtered_error_rate_x_,
        forward,
        target_error_x,
        target_error_y,
        center_tolerance_,
        yaw_kp_,
        yaw_kd_,
        heave_kp_,
        max_visual_yaw_,
        max_visual_heave_);
    Command command;
    command.forward = guidance.forward;
    command.yaw = guidance.yaw;
    command.heave = guidance.heave;
    return command;
  }

  double underwater_heave_command(double visual_error_y = 0.0) const {
    if (!odom_fresh()) return 0.0;
    return clamp(
        underwater_depth_kp_ *
                (odom_.pose.pose.position.z - underwater_target_depth_z_) +
            underwater_visual_heave_gain_ * visual_error_y,
        -pinger_heave_limit_, pinger_heave_limit_);
  }

  double pinger_depth_hold_heave(double target_depth_z) const {
    if (!odom_fresh()) return 0.0;
    return pinger_depth_kp_ * (odom_.pose.pose.position.z - target_depth_z);
  }

  double pinger_acoustic_vertical_heave(
      const std::array<double, 3> &direction_body) const {
    const double z = clamp(direction_body[2], -1.0, 1.0);
    const double deadband = clamp(pinger_vertical_direction_deadband_, 0.0, 0.95);
    if (!std::isfinite(z) || std::abs(z) <= deadband) return 0.0;
    const double magnitude = (std::abs(z) - deadband) / (1.0 - deadband);
    // ENU/body z is positive upward, while positive normalized heave commands
    // the ArduSub/MuJoCo vehicle downward. A source below (z < 0) must descend.
    return -std::copysign(
        pinger_heave_kp_ * magnitude * pinger_acoustic_vertical_scale(), z);
  }

  double pinger_acoustic_vertical_scale() const {
    const auto range = pinger_control_range_m();
    if (!range) return 1.0;
    const double zero_range = std::max(0.0, pinger_acoustic_vertical_zero_range_m_);
    const double full_range = std::max(
        zero_range + 1.0e-3, pinger_acoustic_vertical_full_range_m_);
    return clamp((*range - zero_range) / (full_range - zero_range), 0.0, 1.0);
  }

  double pinger_vertical_heave_command(
      const std::array<double, 3> &direction_body,
      double target_depth_z,
      double visual_error_y = 0.0) const {
    return clamp(
        pinger_depth_hold_heave(target_depth_z) +
            pinger_acoustic_vertical_heave(direction_body) +
            0.10 * visual_error_y,
        -pinger_heave_limit_, pinger_heave_limit_);
  }

  bool pinger_vertical_alignment_required(
      const std::array<double, 3> &direction_body) const {
    const double bearing = std::atan2(direction_body[1], direction_body[0]);
    const bool heading_aligned =
        direction_body[0] > 0.05 &&
        std::abs(bearing) <= pinger_acoustic_crawl_bearing_rad_;
    return std::isfinite(direction_body[2]) &&
           heading_aligned &&
           pinger_acoustic_vertical_scale() > 0.05 &&
           std::abs(direction_body[2]) > pinger_vertical_alignment_tolerance_;
  }

  bool acoustic_position_near_field_locked() const {
    const auto range = pinger_control_range_m();
    return use_acoustic_position_fusion_ &&
           (pinger_position_fit_geometrically_strong() || pinger_position_fit_fresh()) &&
           range &&
           pinger_acoustic_position_lock_range_m_ > 0.0 &&
           *range <= pinger_acoustic_position_lock_range_m_;
  }

  double pinger_effective_capture_aim_offset_x() const {
    double blend = clamp(
        bbox_ratio() / std::max(capture_bbox_ratio_, 1.0e-3), 0.0, 1.0);
    if (const auto range = pinger_control_range_m()) {
      const double full_range = std::max(0.0, pinger_rake_lane_full_range_m_);
      const double start_range = std::max(
          full_range + 1.0e-3, pinger_rake_lane_blend_start_m_);
      const double range_blend = clamp(
          (start_range - *range) / (start_range - full_range), 0.0, 1.0);
      blend = std::max(blend, range_blend);
    }
    return capture_aim_offset_x_ * blend;
  }

  Command pinger_probe_command(double phase_age) const {
    Command command;
    const double turn_sign = (pinger_search_reentry_count_ % 2 == 0) ? 1.0 : -1.0;
    if (pinger_recovery_probe_) {
      // Cancel the rejected leg, then collect a new two-axis baseline while
      // moving on a smooth arc. The alternating turn avoids accumulating a
      // permanent orbit around the source.
      if (phase_age < pinger_doppler_brake_s_) {
        command.forward = pinger_doppler_brake_reverse_;
      } else if (phase_age < pinger_doppler_brake_s_ + 0.25) {
        command.forward = 0.0;
      } else {
        command.forward = std::min(pinger_probe_forward_, 0.20);
        command.yaw = turn_sign * pinger_probe_yaw_;
      }
      if (odom_fresh()) {
        command.heave = clamp(
            pinger_depth_kp_ * (odom_.pose.pose.position.z - pinger_command_depth_z()),
            -pinger_heave_limit_, pinger_heave_limit_);
      }
      return command;
    }
    command.forward = pinger_probe_forward_;
    command.yaw = turn_sign * pinger_probe_yaw_;
    if (odom_fresh()) {
      command.heave = clamp(
          pinger_depth_kp_ * (odom_.pose.pose.position.z - pinger_command_depth_z()),
          -pinger_heave_limit_, pinger_heave_limit_);
    }
    return command;
  }

  double current_yaw_rad() const {
    const auto &q = odom_.pose.pose.orientation;
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void reset_pinger_spin_watchdog() {
    pinger_spin_accumulated_yaw_rad_ = 0.0;
    pinger_spin_watch_initialized_ = odom_fresh();
    if (!pinger_spin_watch_initialized_) return;
    pinger_spin_previous_yaw_rad_ = current_yaw_rad();
    pinger_spin_origin_xy_ = Eigen::Vector2d(
        odom_.pose.pose.position.x, odom_.pose.pose.position.y);
  }

  bool pinger_spin_rehome_required() {
    if (!odom_fresh() || pinger_spin_rehome_yaw_rad_ <= 0.0) return false;
    const double yaw = current_yaw_rad();
    const Eigen::Vector2d xy(odom_.pose.pose.position.x, odom_.pose.pose.position.y);
    if (!pinger_spin_watch_initialized_) {
      pinger_spin_watch_initialized_ = true;
      pinger_spin_previous_yaw_rad_ = yaw;
      pinger_spin_origin_xy_ = xy;
      pinger_spin_accumulated_yaw_rad_ = 0.0;
      return false;
    }
    pinger_spin_accumulated_yaw_rad_ += std::abs(
        wrap_pi(yaw - pinger_spin_previous_yaw_rad_));
    pinger_spin_previous_yaw_rad_ = yaw;
    if ((xy - pinger_spin_origin_xy_).norm() > pinger_spin_rehome_max_translation_m_) {
      pinger_spin_origin_xy_ = xy;
      pinger_spin_accumulated_yaw_rad_ = 0.0;
      return false;
    }
    return pinger_spin_accumulated_yaw_rad_ >= pinger_spin_rehome_yaw_rad_;
  }

  void clear_pinger_position_guidance() {
    fitted_pinger_xy_.reset();
    acoustic_candidate_xy_.reset();
    acoustic_candidate_streak_ = 0;
    pinger_acoustic_position_promoted_ = false;
    acoustic_position_samples_.clear();
    phase_fitted_pinger_xy_.reset();
    phase_range_position_samples_.clear();
    phase_seed_direction_world_.reset();
    fitted_direction_world_.reset();
    range_motion_samples_.clear();
    fit_previous_position_.reset();
    fitted_pinger_position_received_ = {};
    phase_fitted_position_received_ = {};
    fitted_direction_received_ = {};
  }

  Command head_on_capture_command(double forward, double pitch) const {
    Command command;
    command.forward = forward;
    command.pitch = pitch;
    if (!capture_heading_valid_ || !odom_fresh()) return command;
    const double yaw_error = wrap_pi(capture_heading_yaw_rad_ - current_yaw_rad());
    command.yaw = clamp(
        capture_heading_kp_ * yaw_error -
            capture_heading_kd_ * odom_.twist.twist.angular.z,
        -capture_heading_yaw_limit_, capture_heading_yaw_limit_);
    command.heave = clamp(
        underwater_depth_kp_ * (odom_.pose.pose.position.z - capture_depth_z_),
        -pinger_heave_limit_, pinger_heave_limit_);
    return command;
  }

  const char *pinger_motion_stage() const {
    switch (phase_) {
      case Phase::PINGER_SEARCH:
        return pinger_recovery_probe_ ? "RECOVERY_CURVE_PROBE" : "CURVE_PROBE";
      case Phase::PINGER_HOMING:
        return pinger_homing_drive_active() ? "DIRECT_COMMIT" : "MEASUREMENT_PAUSE";
      case Phase::PINGER_FINE_ALIGN:
        if (hydrophone_fresh() &&
            pinger_vertical_alignment_required(selected_hydrophone_direction_body())) {
          return "VERTICAL_ALIGN";
        }
        return "HEAD_ON_ALIGN";
      case Phase::PINGER_CAPTURE: return "HEAD_ON_INSERT";
      case Phase::PINGER_VERIFY: return "VERIFY";
      case Phase::PINGER_BACKOFF: return "BACKOFF";
      default: return "INACTIVE";
    }
  }

  std::array<double, 3> selected_hydrophone_direction_body() const {
    auto direction = selected_hydrophone_direction();
    if (hydrophone_direction_frame_ == "body" || age(odom_received_) > 1.5) return direction;
    const double yaw = current_yaw_rad();
    const double world_x = direction[0];
    const double world_y = direction[1];
    direction[0] = std::cos(yaw) * world_x + std::sin(yaw) * world_y;
    direction[1] = -std::sin(yaw) * world_x + std::cos(yaw) * world_y;
    return direction;
  }

  bool pinger_homing_drive_active() const {
    if (pinger_homing_drive_s_ <= 0.0 || pinger_homing_pause_s_ <= 0.0) {
      return true;
    }
    const double period = pinger_homing_drive_s_ + pinger_homing_pause_s_;
    const double cycle = std::fmod(std::max(age(phase_started_), 0.0), period);
    return cycle < pinger_homing_drive_s_;
  }

  Command pinger_homing_command() {
    Command command;
    if (!hydrophone_fresh()) return command;
    const auto selected_direction = selected_hydrophone_direction_body();
    double x = selected_direction[0];
    double y = selected_direction[1];
    publish_hydrophone_direction_body(selected_direction);
    const double bearing = wrap_pi(std::atan2(y, x));
    const double yaw_rate = odom_.twist.twist.angular.z;
    command.yaw = clamp(
        pinger_yaw_kp_ * bearing - pinger_yaw_kd_ * yaw_rate,
        -pinger_yaw_limit_, pinger_yaw_limit_);
    command.heave = pinger_vertical_heave_command(
        selected_direction, pinger_command_depth_z());
    double aligned_forward = pinger_forward_fast_;
    if (const auto range = pinger_control_range_m()) {
      if (*range <= pinger_final_slow_range_m_) {
        aligned_forward = pinger_final_forward_;
      } else if (*range <= pinger_near_slow_range_m_) {
        aligned_forward = pinger_near_forward_;
      }
    }
    const bool heading_aligned =
        x > 0.05 && std::abs(bearing) <= pinger_heading_drive_tolerance_rad_;
    command.forward = heading_aligned ? aligned_forward : pinger_forward_turn_;
    pinger_heading_drive_blocked_ = !heading_aligned && pinger_forward_turn_ <= 0.0;
    if (heading_aligned && !pinger_visual_acquired_ &&
        pinger_homing_yaw_dither_period_s_ > 0.1) {
      const double dither_phase = 6.283185307179586 * age(phase_started_) /
                                  pinger_homing_yaw_dither_period_s_;
      command.yaw = clamp(
          command.yaw + pinger_homing_yaw_dither_amplitude_ * std::sin(dither_phase),
          -pinger_yaw_limit_, pinger_yaw_limit_);
    }
    if (!pinger_visual_acquired_ && pinger_homing_sway_period_s_ > 0.1) {
      const double phase = 6.283185307179586 * age(phase_started_) /
                           pinger_homing_sway_period_s_;
      double sway_scale = 1.0;
      if (dynamic_pinger_position_fit_fresh()) {
        sway_scale = 0.0;
      } else if (prefer_internal_hydrophone_direction_ && adaptive_direction_fit_fresh()) {
        sway_scale = 0.35;
      }
      command.sway = sway_scale * pinger_homing_sway_amplitude_ * std::sin(phase);
    }
    if (!pinger_homing_drive_active()) {
      command.forward = 0.0;
      command.sway = 0.0;
    }
    return command;
  }

  void publish_hydrophone_direction_body(const std::array<double, 3> &direction) {
    geometry_msgs::msg::Vector3Stamped body_direction;
    body_direction.header.stamp = now();
    body_direction.header.frame_id = "base_link";
    body_direction.vector.x = direction[0];
    body_direction.vector.y = direction[1];
    body_direction.vector.z = direction[2];
    hydrophone_body_pub_->publish(body_direction);
  }

  double pinger_command_depth_z() const {
    const bool near_from_iq =
        acoustic_range_m_ && age(iq_magnitude_received_) <= hydrophone_timeout_s_ &&
        *acoustic_range_m_ <= pinger_depth_transition_range_m_;
    const auto fitted_range = dynamic_pinger_range_m();
    const bool near_from_fit = !acoustic_range_fresh() &&
        fitted_range && *fitted_range <= pinger_depth_transition_range_m_;
    if (near_from_iq || near_from_fit) {
      return pinger_depth_z_;
    }
    return pinger_transit_depth_z_;
  }

  Command zone_command() const {
    Command command;
    if (!odom_fresh()) return command;
    const auto &position = odom_.pose.pose.position;
    const auto &q = odom_.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double dx = score_x_ - position.x;
    const double dy = score_y_ - position.y;
    const double error = wrap_pi(std::atan2(dy, dx) - yaw);
    command.yaw = clamp(1.1 * error, -0.72, 0.72);
    command.forward = std::abs(error) < 0.7 ? 0.72 : 0.16;
    return command;
  }

  bool pinger_phase() const {
    return is_pinger_phase(phase_);
  }

  bool surface_phase() const {
    return phase_ == Phase::SURFACE_SEARCH || phase_ == Phase::SURFACE_COLLECT ||
           phase_ == Phase::RETURN_ZONE || phase_ == Phase::SCORE_ZONE_IN ||
           phase_ == Phase::RELEASE;
  }

  const char *mission_state_name() const {
    switch (phase_) {
      case Phase::IDLE: return "IDLE";
      case Phase::WAIT_VEHICLE: return "WAIT_VEHICLE";
      case Phase::WAIT_ARM: return "ARM";
      case Phase::PINGER_SEARCH:
      case Phase::PINGER_HOMING: return "PINGER_HYDROPHONE_HOMING";
      case Phase::PINGER_FINE_ALIGN: return "PINGER_YOLO_FINE_ALIGN";
      case Phase::PINGER_CAPTURE:
      case Phase::PINGER_VERIFY: return "PINGER_RAKE_RELEASE";
      case Phase::PINGER_BACKOFF:
      case Phase::TURN_TO_OWN_ZONE: return "TURN_TO_OWN_ZONE";
      case Phase::VISION_CONTROL:
        if (vision_state_ == "APPROACH_BUOY") return "BUOY_APPROACH";
        if (vision_state_ == "ALIGN_STICK") return "STICK_FINE_ALIGN";
        if (vision_state_ == "INSERT_FORK" || vision_state_ == "DETACH" ||
            vision_state_ == "BACKOFF" || vision_state_ == "VERIFY_RELEASE") {
          return "RAKE_RELEASE";
        }
        if (vision_state_ == "ASCEND" || vision_state_ == "COMPLETE") {
          return "SURFACE_ASCEND";
        }
        return "BUOY_SEARCH";
      case Phase::SEARCH: return "BUOY_SEARCH";
      case Phase::APPROACH:
        return age(phase_started_) < 0.15 ? "BUOY_DETECTED" : "BUOY_APPROACH";
      case Phase::FINE_ALIGN: return "STICK_FINE_ALIGN";
      case Phase::CAPTURE:
      case Phase::VERIFY_CAPTURE: return "RAKE_RELEASE";
      case Phase::BACKOFF:
        return detached_count_ >= expected_detach_count_ ? "SURFACE_ASCEND" : "BUOY_SEARCH";
      case Phase::SURFACE_SEARCH:
        return surface_ready() ? "RED_BUOY_SEARCH" : "SURFACE_ASCEND";
      case Phase::SURFACE_COLLECT: return "NET_CAPTURE";
      case Phase::RETURN_ZONE: return "SCORE_ZONE_TRANSIT";
      case Phase::SCORE_ZONE_IN: return "SCORE_ZONE_IN";
      case Phase::RELEASE: return "SCORE_RELEASE";
      case Phase::COMPLETE: return "COMPLETE";
      case Phase::FAILED: return "FAILED";
    }
    return "UNKNOWN";
  }

  bool vehicle_in_own_course() const {
    if (!odom_fresh()) return false;
    if (own_course_ == "all") return true;
    if (own_course_ != "a" && own_course_ != "b") return false;
    const double x = odom_.pose.pose.position.x;
    const double standoff = std::max(0.0, course_boundary_standoff_);
    return own_course_ == "a"
        ? x <= course_boundary_x_ - standoff
        : x >= course_boundary_x_ + standoff;
  }

  bool target_in_own_course(bool reject_detached = true) const {
    if (!observation_fresh() || !odom_fresh()) {
      return true;
    }
    if (reject_detached && vehicle_near_detached_contact()) return false;
    double range = 0.0;
    if (observation_.range_valid && std::isfinite(observation_.range_m) &&
        observation_.range_m > 0.05) {
      range = observation_.range_m;
    } else {
      const double ratio = std::max(0.015, bbox_ratio());
      range = target_height_m_ /
          (2.0 * std::tan(0.5 * camera_vertical_fov_rad_) * ratio);
      range = clamp(range, 0.2, 20.0);
    }
    const auto &q = odom_.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double target_x = odom_.pose.pose.position.x +
        range * std::cos(yaw + static_cast<double>(observation_.bearing_rad));
    const double target_y = odom_.pose.pose.position.y +
        range * std::sin(yaw + static_cast<double>(observation_.bearing_rad));
    const Eigen::Vector2d estimated_target(target_x, target_y);
    if (reject_detached && std::any_of(
            detached_contact_xy_.begin(), detached_contact_xy_.end(),
            [this, &estimated_target](const Eigen::Vector2d &contact) {
              return (estimated_target - contact).norm() <= detached_exclusion_radius_m_;
            })) {
      return false;
    }
    if (own_course_ != "a" && own_course_ != "b") return true;
    const double tolerance = std::max(0.0, course_boundary_standoff_ * 0.5);
    return own_course_ == "a"
        ? target_x <= course_boundary_x_ + tolerance
        : target_x >= course_boundary_x_ - tolerance;
  }

  bool vehicle_near_detached_contact() const {
    if (!odom_fresh()) return false;
    const Eigen::Vector2d vehicle_xy(
        odom_.pose.pose.position.x, odom_.pose.pose.position.y);
    return std::any_of(
        detached_contact_xy_.begin(), detached_contact_xy_.end(),
        [this, &vehicle_xy](const Eigen::Vector2d &contact) {
          return (vehicle_xy - contact).norm() <= detached_exclusion_radius_m_;
        });
  }

  void build_search_waypoints() {
    search_waypoints_.clear();
    const double half_width = 0.5 * std::max(1.0, search_area_width_m_);
    const double half_height = 0.5 * std::max(1.0, search_area_height_m_);
    double min_x = search_area_center_x_ - half_width;
    double max_x = search_area_center_x_ + half_width;
    if (own_course_ == "a") {
      max_x = std::min(max_x, course_boundary_x_ - course_boundary_standoff_);
    } else if (own_course_ == "b") {
      min_x = std::max(min_x, course_boundary_x_ + course_boundary_standoff_);
    }
    const double min_y = search_area_center_y_ - half_height;
    const double max_y = search_area_center_y_ + half_height;
    const double spacing = std::max(0.5, search_lane_spacing_m_);
    const int lanes = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / spacing)));
    for (int lane = 0; lane <= lanes; ++lane) {
      const double y = std::min(max_y, min_y + lane * spacing);
      if (lane % 2 == 0) {
        search_waypoints_.emplace_back(min_x, y);
        search_waypoints_.emplace_back(max_x, y);
      } else {
        search_waypoints_.emplace_back(max_x, y);
        search_waypoints_.emplace_back(min_x, y);
      }
    }
  }

  Command area_search_command() {
    Command command;
    command.heave = underwater_heave_command();
    if (!odom_fresh() || search_waypoints_.empty()) {
      command.yaw = search_yaw_ * (own_course_ == "b" ? -1.0 : 1.0);
      return command;
    }
    const Eigen::Vector2d vehicle_xy(
        odom_.pose.pose.position.x, odom_.pose.pose.position.y);
    const auto &q = odom_.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    if (vehicle_near_detached_contact()) {
      const auto nearest = std::min_element(
          detached_contact_xy_.begin(), detached_contact_xy_.end(),
          [&vehicle_xy](const Eigen::Vector2d &left, const Eigen::Vector2d &right) {
            return (left - vehicle_xy).squaredNorm() <
                   (right - vehicle_xy).squaredNorm();
          });
      Eigen::Vector2d away = vehicle_xy - *nearest;
      if (away.norm() < 0.10) {
        away = Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
      }
      const double escape_error = wrap_pi(std::atan2(away.y(), away.x()) - yaw);
      command.yaw = clamp(
          1.2 * escape_error, -search_turn_yaw_limit_, search_turn_yaw_limit_);
      command.forward = std::abs(escape_error) < search_heading_tolerance_rad_
          ? search_escape_forward_
          : 0.0;
      return command;
    }
    if (!search_waypoint_initialized_) {
      double best_distance = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < search_waypoints_.size(); ++index) {
        const double distance = (search_waypoints_[index] - vehicle_xy).norm();
        if (distance < best_distance) {
          best_distance = distance;
          search_waypoint_index_ = index;
        }
      }
      search_waypoint_initialized_ = true;
    }
    auto target = search_waypoints_[search_waypoint_index_];
    if ((target - vehicle_xy).norm() <= search_waypoint_tolerance_m_) {
      search_waypoint_index_ = (search_waypoint_index_ + 1) % search_waypoints_.size();
      target = search_waypoints_[search_waypoint_index_];
    }
    const double heading_error = wrap_pi(
        std::atan2(target.y() - vehicle_xy.y(), target.x() - vehicle_xy.x()) - yaw);
    command.yaw = clamp(
        heading_error, -search_turn_yaw_limit_, search_turn_yaw_limit_);
    command.forward = std::abs(heading_error) < search_heading_tolerance_rad_
        ? search_forward_
        : 0.0;
    return command;
  }

  Command boundary_guard(const Command &input) const {
    if (pinger_phase() || !odom_fresh() ||
        (own_course_ != "a" && own_course_ != "b")) {
      return input;
    }
    Command output = input;
    const auto &q = odom_.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double opponent_sign = own_course_ == "a" ? 1.0 : -1.0;
    const double stop_x = course_boundary_x_ -
        opponent_sign * std::max(0.0, course_boundary_standoff_);
    const double distance_to_stop = opponent_sign *
        (stop_x - odom_.pose.pose.position.x);
    const double world_x_command = c * output.forward - s * output.sway;
    const double opponent_command = opponent_sign * world_x_command;
    if (distance_to_stop >= course_boundary_margin_ || opponent_command <= 0.0) {
      return output;
    }
    const double scale = clamp(
        distance_to_stop / std::max(course_boundary_margin_, 1.0e-3), 0.0, 1.0);
    const double allowed_world_x = world_x_command * scale;
    const double correction = allowed_world_x - world_x_command;
    output.forward = clamp(output.forward + correction * c, -1.0, 1.0);
    output.sway = clamp(output.sway - correction * s, -1.0, 1.0);
    if (distance_to_stop <= 0.0) {
      output.forward = clamp(output.forward - opponent_sign * 0.14 * c, -1.0, 1.0);
      output.sway = clamp(output.sway + opponent_sign * 0.14 * s, -1.0, 1.0);
    }
    return output;
  }

  bool in_score_zone() const {
    if (!odom_fresh()) return false;
    const double dx = score_x_ - odom_.pose.pose.position.x;
    const double dy = score_y_ - odom_.pose.pose.position.y;
    return std::hypot(dx, dy) <= score_radius_;
  }

  bool surface_ready() const {
    return odom_fresh() &&
        std::abs(odom_.pose.pose.position.z - surface_collect_depth_z_) <=
            surface_collect_depth_tolerance_m_;
  }

  double surface_heave_command() const {
    if (!odom_fresh()) return 0.0;
    return clamp(
        surface_depth_kp_ * (odom_.pose.pose.position.z - surface_collect_depth_z_),
        -0.55, 0.55);
  }

  Command surface_visual_command() const {
    Command command;
    command.heave = surface_heave_command();
    if (age(observation_received_) > surface_steer_timeout_s_) {
      return command;
    }
    const auto guidance = kmu26_mission_fsm::vision_buoy::surface_command(
        filtered_error_x_,
        filtered_error_rate_x_,
        command.heave,
        surface_yaw_kp_,
        yaw_kd_,
        surface_yaw_limit_,
        surface_center_tolerance_,
        surface_forward_,
        surface_turn_forward_);
    command.forward = guidance.forward;
    command.yaw = guidance.yaw;
    command.heave = guidance.heave;
    return command;
  }

  void tick() {
    Command command;
    bool active_control = false;
    pinger_heading_drive_blocked_ = false;
    if (!enabled_) {
      transition(Phase::IDLE);
    } else if (!vehicle_ready()) {
      transition(Phase::WAIT_VEHICLE);
    } else if (require_armed_ && !vehicle_armed_effective()) {
      transition(Phase::WAIT_ARM);
    } else {
      if (phase_ == Phase::WAIT_VEHICLE || phase_ == Phase::WAIT_ARM || phase_ == Phase::IDLE) {
        transition(start_surface_phase_
                       ? Phase::SURFACE_SEARCH
                       : (use_pinger_first_ && detached_target_ids_.empty()
                              ? Phase::PINGER_SEARCH
                              : underwater_entry_phase()));
      }
      command = boundary_guard(step_active());
      active_control = phase_ != Phase::FAILED && phase_ != Phase::COMPLETE;
    }
    publish_vision_enable(
        delegate_vision_control_ && enabled_ && phase_ == Phase::VISION_CONTROL &&
        vehicle_ready());
    command = active_control ? slew_command(command) : Command{};
    publish_rc(command);
    publish_status(command);
  }

  Command step_active() {
    Command command;
    const double phase_age = age(phase_started_);
    const bool in_pinger_phase = is_pinger_phase(phase_);
    if (in_pinger_phase && non_pinger_detached_during_pinger_) {
      fail("non-pinger buoy detached during pinger homing");
      return command;
    }
    if (in_pinger_phase && pinger_detached_observed_ &&
        phase_ != Phase::PINGER_BACKOFF) {
      acknowledged_detach_count_ = detached_count_;
      transition(Phase::PINGER_BACKOFF);
      return command;
    }
    if ((phase_ == Phase::PINGER_HOMING || phase_ == Phase::PINGER_FINE_ALIGN) &&
        pinger_doppler_reversal_detected_) {
      RCLCPP_WARN(
          get_logger(),
          "pinger closest-point reversal detected: braking immediately (approach=%d recede=%d)",
          pinger_doppler_approach_count_, pinger_doppler_recede_count_);
      pinger_range_worsening_ = true;
      pinger_recovery_probe_ = true;
      transition(Phase::PINGER_SEARCH);
      return command;
    }
    if ((phase_ == Phase::PINGER_HOMING || phase_ == Phase::PINGER_FINE_ALIGN) &&
        pinger_spin_rehome_required()) {
      RCLCPP_WARN(
          get_logger(),
          "pinger full-turn watchdog: stopping and reacquiring (yaw=%.2frad translation<=%.2fm)",
          pinger_spin_accumulated_yaw_rad_, pinger_spin_rehome_max_translation_m_);
      pinger_recovery_probe_ = true;
      pinger_spin_rehome_active_ = true;
      ++pinger_spin_rehome_count_;
      clear_pinger_position_guidance();
      transition(Phase::PINGER_SEARCH);
      return command;
    }
    const bool in_underwater_buoy_phase = is_underwater_buoy_phase(phase_);
    if (in_underwater_buoy_phase && detached_count_ > acknowledged_detach_count_) {
      acknowledged_detach_count_ = detached_count_;
      if (phase_ != Phase::BACKOFF) transition(Phase::BACKOFF);
      return command;
    }
    switch (phase_) {
      case Phase::PINGER_SEARCH:
        command = pinger_spin_rehome_active_ && phase_age < pinger_spin_rehome_stop_s_
            ? Command{}
            : pinger_probe_command(
                  pinger_spin_rehome_active_ ? phase_age - pinger_spin_rehome_stop_s_ : phase_age);
        {
        const double stop_delay = pinger_spin_rehome_active_ ? pinger_spin_rehome_stop_s_ : 0.0;
        const bool probe_complete =
            phase_age >= stop_delay + pinger_search_probe_hold_s();
        if ((!pinger_recovery_probe_ || probe_complete) && pinger_yolo_fresh()) {
          pinger_visual_acquired_ = true;
          pinger_recovery_probe_ = false;
          transition(Phase::PINGER_FINE_ALIGN);
          break;
        }
        if (hydrophone_fresh()) {
          const auto direction_body = selected_hydrophone_direction_body();
          publish_hydrophone_direction_body(direction_body);
          if (probe_complete && pinger_guidance_ready(phase_age)) {
            command = pinger_homing_command();
          }
        }
          if (probe_complete && hydrophone_fresh() &&
              pinger_guidance_ready(phase_age)) {
            pinger_recovery_probe_ = false;
            transition(Phase::PINGER_HOMING);
          }
        }
        break;
      case Phase::PINGER_HOMING:
        if (!hydrophone_fresh()) {
          transition(Phase::PINGER_SEARCH);
        } else if (pinger_yolo_fresh()) {
          pinger_visual_acquired_ = true;
          pinger_recovery_probe_ = false;
          transition(Phase::PINGER_FINE_ALIGN);
        } else if (acoustic_pinger_near()) {
          transition(Phase::PINGER_FINE_ALIGN);
        } else if (pinger_homing_range_rejected(phase_age)) {
          transition(Phase::PINGER_SEARCH);
        } else {
          command = pinger_homing_command();
        }
        break;
      case Phase::PINGER_FINE_ALIGN:
        {
          bool acoustic_capture_aligned = false;
          bool vertical_alignment_required = false;
          std::optional<std::array<double, 3>> direction_body;
        if (pinger_visual_acquired_ &&
            age(pinger_visual_last_seen_) > pinger_visual_reacquire_timeout_s_) {
          pinger_visual_acquired_ = false;
          alignment_started_.reset();
          transition(Phase::PINGER_SEARCH);
          break;
        }
        if (pinger_homing_range_rejected(phase_age, false)) {
          pinger_visual_acquired_ = false;
          alignment_started_.reset();
          transition(Phase::PINGER_SEARCH);
          break;
        }
        if (hydrophone_fresh()) {
          direction_body = selected_hydrophone_direction_body();
          publish_hydrophone_direction_body(*direction_body);
          vertical_alignment_required = pinger_vertical_alignment_required(*direction_body);
          if (!pinger_visual_fresh()) {
            // Keep steering on the live acoustic bearing while moving slowly.
            // The shallow curved path also preserves observability for one sensor.
            command = pinger_homing_command();
            command.forward = std::min(
                command.forward, pinger_acoustic_crawl_forward_);
            const double bearing = wrap_pi(std::atan2((*direction_body)[1], (*direction_body)[0]));
            if ((*direction_body)[0] <= 0.05 ||
                std::abs(bearing) > pinger_acoustic_crawl_bearing_rad_) {
              command.forward = std::min(command.forward, pinger_forward_turn_);
            }
            if (command.forward <= 0.0) {
              pinger_heading_drive_blocked_ = true;
            }
            // Hydrophone homing has already reached the camera handoff radius.
            // Do not crawl through the emitting buoy while the slower YOLO
            // pipeline produces its first fresh frame. Keep yaw/depth control
            // active so the target remains in the forward camera view.
            command.forward = 0.0;
            pinger_heading_drive_blocked_ = true;
            const auto fitted_range = dynamic_pinger_range_m();
            acoustic_capture_aligned = !require_pinger_yolo_for_capture_ && fitted_range &&
                *fitted_range <= pinger_acoustic_capture_range_m_ &&
                (*direction_body)[0] > 0.0 &&
                std::abs(bearing) <= pinger_acoustic_capture_bearing_rad_;
          }
        }
        if (!pinger_visual_fresh()) {
          if (!hydrophone_fresh()) {
            command.forward = 0.0;
            command.yaw = pinger_scan_yaw_;
          }
          command.heave = direction_body
              ? pinger_vertical_heave_command(*direction_body, pinger_depth_z_)
              : clamp(pinger_depth_hold_heave(pinger_depth_z_),
                      -pinger_heave_limit_, pinger_heave_limit_);
          if (vertical_alignment_required) {
            command.forward = std::min(command.forward, pinger_vertical_forward_limit_);
            command.sway = 0.0;
            command.yaw = clamp(
                -pinger_yaw_kd_ * odom_.twist.twist.angular.z,
                -pinger_vertical_yaw_limit_, pinger_vertical_yaw_limit_);
            acoustic_capture_aligned = false;
          }
          if (acoustic_capture_aligned) {
            if (!alignment_started_) alignment_started_ = Clock::now();
            if (age(*alignment_started_) >= alignment_hold_s_) {
              transition(Phase::PINGER_CAPTURE);
            }
          } else {
            alignment_started_.reset();
          }
        } else {
          pinger_visual_acquired_ = true;
          const double pinger_aim_offset_x = pinger_effective_capture_aim_offset_x();
          command = visual_command(
              0.18, pinger_aim_offset_x, capture_aim_offset_y_);
          const double visual_error_y = filtered_error_y_ - capture_aim_offset_y_;
          command.heave = direction_body
              ? pinger_vertical_heave_command(
                    *direction_body, pinger_depth_z_, visual_error_y)
              : clamp(
                    pinger_depth_hold_heave(pinger_depth_z_) + 0.10 * visual_error_y,
                    -pinger_heave_limit_, pinger_heave_limit_);
          if (vertical_alignment_required) {
            command.forward = std::min(command.forward, pinger_vertical_forward_limit_);
            command.sway = 0.0;
            command.yaw = clamp(
                command.yaw, -pinger_vertical_yaw_limit_, pinger_vertical_yaw_limit_);
          }
          const double capture_error_x = filtered_error_x_ - pinger_aim_offset_x;
          const double capture_error_y = filtered_error_y_ - capture_aim_offset_y_;
          const auto pinger_range = pinger_control_range_m();
          const bool acoustic_commit_range = pinger_range &&
              *pinger_range <= pinger_capture_commit_range_m_;
          const bool capture_aligned =
              !vertical_alignment_required &&
              bbox_ratio() >= pinger_capture_min_bbox_ratio_ &&
              (bbox_ratio() >= capture_bbox_ratio_ || acoustic_commit_range) &&
              std::abs(capture_error_x) <= capture_center_tolerance_x_ &&
              std::abs(capture_error_y) <= capture_center_tolerance_y_;
          if (capture_aligned) {
            if (!alignment_started_) alignment_started_ = Clock::now();
            if (age(*alignment_started_) >= capture_alignment_hold_s_) {
              transition(Phase::PINGER_CAPTURE);
            }
          } else {
            alignment_started_.reset();
          }
        }
        }
        break;
      case Phase::PINGER_CAPTURE:
        command = head_on_capture_command(capture_insert_forward_, 0.0);
        if (phase_age >= capture_drive_s_) transition(Phase::PINGER_VERIFY);
        break;
      case Phase::PINGER_VERIFY:
        if (new_collector_event() && collector_target_is_pinger() &&
            (collector_.detached || collector_.captured || collector_.netted)) {
          remember_detached_target();
          acknowledged_detach_count_ = detached_count_;
          transition(Phase::PINGER_BACKOFF);
        } else if (phase_age > collector_timeout_s_) {
          transition(pinger_visual_acquired_ ? Phase::PINGER_FINE_ALIGN : Phase::PINGER_SEARCH);
        }
        break;
      case Phase::PINGER_BACKOFF:
        command = head_on_capture_command(capture_backoff_forward_, 0.0);
        if (phase_age >= capture_backoff_s_) {
          transition(Phase::TURN_TO_OWN_ZONE);
        }
        break;
      case Phase::TURN_TO_OWN_ZONE:
        if (!odom_fresh()) {
          fail("odometry timeout while turning to own zone");
        } else if (vehicle_in_own_course()) {
          transition(detached_count_ >= expected_detach_count_
                         ? Phase::SURFACE_SEARCH
                         : underwater_entry_phase());
        } else {
          command = area_search_command();
        }
        break;
      case Phase::VISION_CONTROL:
        // The zetex1001 visual state machine owns yaw/heave/forward through
        // /control/vision/rc_override while this node remains the full-mission
        // orchestrator. Boundary and completion decisions stay here because
        // they require odometry and collector contracts outside that package.
        if (!odom_fresh()) {
          fail("odometry timeout during delegated vision control");
        } else if (!vehicle_in_own_course() ||
                   (observation_fresh() && !target_in_own_course())) {
          transition(Phase::TURN_TO_OWN_ZONE);
        } else if (!vision_state_fresh()) {
          if (phase_age > vision_state_timeout_s_) {
            fail("delegated vision controller state timeout");
          }
        } else if (vision_state_ == "FAILSAFE") {
          fail("delegated vision controller entered FAILSAFE");
        } else if (detached_count_ >= expected_detach_count_) {
          transition(Phase::SURFACE_SEARCH);
        } else if (vision_state_ == "ASCEND" || vision_state_ == "COMPLETE") {
          if (vision_complete_requires_detach_count_) {
            fail("vision controller completed before required detach count");
          } else {
            transition(Phase::SURFACE_SEARCH);
          }
        } else if ((vision_state_ == "SEARCH" || vision_state_ == "AREA_VERIFY") &&
                   phase_age >= vision_search_handoff_s_ && !observation_fresh()) {
          // One target interaction ended or the candidate disappeared. Return
          // motion ownership to the odometry search instead of spinning at a
          // fixed point in the upstream SEARCH state.
          transition(Phase::SEARCH);
        }
        break;
      case Phase::SEARCH:
        command = area_search_command();
        if (observation_fresh() && bbox_ratio() >= search_accept_bbox_ratio_ &&
            target_in_own_course()) {
          transition(delegate_vision_control_ ? Phase::VISION_CONTROL : Phase::APPROACH);
        }
        break;
      case Phase::APPROACH:
        if (!observation_fresh() || !target_in_own_course()) transition(Phase::SEARCH);
        else {
          command = visual_command(0.68);
          command.heave = underwater_heave_command(filtered_error_y_);
          if (bbox_ratio() >= fine_bbox_ratio_) transition(Phase::FINE_ALIGN);
        }
        break;
      case Phase::FINE_ALIGN:
        if (!observation_fresh() || !target_in_own_course()) transition(Phase::SEARCH);
        else {
          command = visual_command(
              0.18, capture_aim_offset_x_, capture_aim_offset_y_);
          command.heave = underwater_heave_command(
              filtered_error_y_ - capture_aim_offset_y_);
          const double capture_error_x = filtered_error_x_ - capture_aim_offset_x_;
          const double capture_error_y = filtered_error_y_ - capture_aim_offset_y_;
          const bool capture_aligned =
              bbox_ratio() >= capture_bbox_ratio_ &&
              std::abs(capture_error_x) <= capture_center_tolerance_x_ &&
              std::abs(capture_error_y) <= capture_center_tolerance_y_;
          if (capture_aligned) {
            if (!alignment_started_) alignment_started_ = Clock::now();
            if (age(*alignment_started_) >= capture_alignment_hold_s_) {
              transition(Phase::CAPTURE);
            }
          } else {
            alignment_started_.reset();
          }
        }
        break;
      case Phase::CAPTURE:
        command = head_on_capture_command(capture_insert_forward_, 0.0);
        if (phase_age >= capture_drive_s_) transition(Phase::VERIFY_CAPTURE);
        break;
      case Phase::VERIFY_CAPTURE:
        if (detached_count_ > acknowledged_detach_count_) {
          acknowledged_detach_count_ = detached_count_;
          transition(Phase::BACKOFF);
        } else if (new_collector_event() && collector_.detached && collector_.target_id.empty()) {
          remember_detached_target();
          acknowledged_detach_count_ = detached_count_;
          transition(Phase::BACKOFF);
        } else if (phase_age > collector_timeout_s_) {
          transition(observation_fresh() && target_in_own_course()
                         ? Phase::FINE_ALIGN
                         : Phase::SEARCH);
        }
        break;
      case Phase::BACKOFF:
        command = head_on_capture_command(capture_backoff_forward_, 0.0);
        if (phase_age >= capture_backoff_s_) {
          transition(detached_count_ >= expected_detach_count_
                         ? Phase::SURFACE_SEARCH
                         : Phase::SEARCH);
        }
        break;
      case Phase::SURFACE_SEARCH:
        command.heave = surface_heave_command();
        command.yaw = 0.28;
        if (surface_ready() && observation_fresh() && target_in_own_course(false)) {
          transition(Phase::SURFACE_COLLECT);
        }
        break;
      case Phase::SURFACE_COLLECT:
        if (!surface_ready() || !observation_fresh() || !target_in_own_course(false)) {
          transition(Phase::SURFACE_SEARCH);
        }
        else {
          command = surface_visual_command();
          if (netted_count_ > acknowledged_netted_count_) {
            acknowledged_netted_count_ = netted_count_;
            transition(netted_count_ >= expected_net_count_ ? Phase::RETURN_ZONE : Phase::SURFACE_SEARCH);
          } else if (new_collector_event() && collector_.netted && collector_.target_id.empty()) {
            remember_netted_target();
            acknowledged_netted_count_ = netted_count_;
            transition(netted_count_ >= expected_net_count_ ? Phase::RETURN_ZONE : Phase::SURFACE_SEARCH);
          }
        }
        break;
      case Phase::RETURN_ZONE:
        if (!odom_fresh()) fail("odometry timeout while returning to score zone");
        else if (in_score_zone()) transition(Phase::SCORE_ZONE_IN);
        else command = zone_command();
        break;
      case Phase::SCORE_ZONE_IN:
        if (!odom_fresh()) {
          fail("odometry timeout while confirming score zone");
        } else if (!in_score_zone()) {
          transition(Phase::RETURN_ZONE);
        } else if (phase_age >= 0.25) {
          transition(Phase::RELEASE);
        }
        break;
      case Phase::RELEASE:
        command.pitch = 0.60;
        command.forward = 0.15;
        if (released_count_ >= expected_net_count_ && in_score_zone()) {
          transition(Phase::COMPLETE);
        } else if (new_collector_event() && collector_.released &&
                   collector_.target_id.empty() && in_score_zone()) {
          remember_released_target();
          if (released_count_ >= expected_net_count_) transition(Phase::COMPLETE);
        } else if (phase_age > release_timeout_s_) {
          fail("not all collected buoys were released inside score zone");
        }
        break;
      case Phase::COMPLETE:
      case Phase::FAILED:
      case Phase::IDLE:
      case Phase::WAIT_VEHICLE:
      case Phase::WAIT_ARM:
        break;
    }
    return command;
  }

  uint16_t pwm(double value, bool invert = false) const {
    const double axis = invert ? -value : value;
    return static_cast<uint16_t>(std::lround(kNeutral + clamp(axis, -1.0, 1.0) * rc_span_));
  }

  Command slew_command(const Command &desired) {
    const auto now = Clock::now();
    const double dt = clamp(
        std::chrono::duration<double>(now - previous_command_time_).count(), 0.0, 0.10);
    const double limit = std::max(0.01, command_slew_per_s_) * dt;
    auto limited = [limit](double current, double target) {
      return current + clamp(target - current, -limit, limit);
    };
    Command output;
    output.forward = limited(previous_command_.forward, desired.forward);
    output.sway = limited(previous_command_.sway, desired.sway);
    output.heave = limited(previous_command_.heave, desired.heave);
    output.yaw = limited(previous_command_.yaw, desired.yaw);
    output.pitch = limited(previous_command_.pitch, desired.pitch);
    if (phase_ == Phase::PINGER_HOMING && !pinger_homing_drive_active()) {
      // A benchmark pause means neutral propulsion now, not a pause that is
      // consumed by the generic slew limiter. Yaw and depth loops stay live.
      output.forward = 0.0;
      output.sway = 0.0;
    }
    if (is_pinger_phase(phase_) && pinger_heading_drive_blocked_) {
      // Differential thrust can rotate the UUV in place. Remove residual
      // forward slew immediately so yaw correction does not become a wide arc.
      output.forward = 0.0;
      output.sway = 0.0;
    }
    previous_command_ = output;
    previous_command_time_ = now;
    return output;
  }

  void publish_rc(const Command &command) {
    mavros_msgs::msg::OverrideRCIn msg;
    msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    if (dry_run_ || phase_ == Phase::IDLE || phase_ == Phase::WAIT_VEHICLE ||
        phase_ == Phase::WAIT_ARM || phase_ == Phase::FAILED || phase_ == Phase::COMPLETE) {
      msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    } else if (delegate_vision_control_ && phase_ == Phase::VISION_CONTROL) {
      // A release-only frame is ignored by rc_override_mux, allowing the
      // lower-priority vision source to become the sole active owner.
      msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    } else {
      for (std::size_t index = 0; index < 8; ++index) msg.channels[index] = kNeutral;
      msg.channels[kPitch] = pwm(command.pitch);
      msg.channels[kHeave] = pwm(command.heave, invert_heave_);
      msg.channels[kYaw] = pwm(command.yaw, invert_yaw_);
      msg.channels[kForward] = pwm(command.forward);
      msg.channels[kSway] = pwm(command.sway);
    }
    rc_pub_->publish(msg);
  }

  void publish_status(const Command &command) {
    std_msgs::msg::String msg;
    std::ostringstream out;
    const auto selected_direction = selected_hydrophone_direction();
    const auto selected_direction_body = selected_hydrophone_direction_body();
    const bool vertical_alignment_required = hydrophone_fresh() &&
        pinger_vertical_alignment_required(selected_direction_body);
    out << "{\"enabled\":" << (enabled_ ? "true" : "false")
        << ",\"dry_run\":" << (dry_run_ ? "true" : "false")
        << ",\"mission_elapsed_s\":"
        << (enabled_ ? age(mission_started_) : 0.0)
        << ",\"state\":\"" << mission_state_name() << "\""
        << ",\"internal_state\":\"" << phase_name(phase_) << "\""
        << ",\"vision_delegated\":" << (delegate_vision_control_ ? "true" : "false")
        << ",\"vision_state\":\"" << vision_state_ << "\""
        << ",\"vision_state_fresh\":" << (vision_state_fresh() ? "true" : "false")
        << ",\"connected\":" << (vehicle_connected_effective() ? "true" : "false")
        << ",\"armed\":" << (vehicle_armed_effective() ? "true" : "false")
        << ",\"raw_connected\":" << (vehicle_state_.connected ? "true" : "false")
        << ",\"raw_armed\":" << (vehicle_state_.armed ? "true" : "false")
        << ",\"connection_grace_active\":"
        << (connection_grace_active() ? "true" : "false")
        << ",\"mode\":\"" << vehicle_state_.mode << "\""
        << ",\"robot_state\":\""
        << (!vehicle_connected_effective() ? "DISCONNECTED" :
            (vehicle_armed_effective() ? "ARMED" : "DISARMED")) << "\""
        << ",\"robot_state_label\":\""
        << (!vehicle_connected_effective() ? "vehicle disconnected" :
            (vehicle_armed_effective() ? "vehicle armed" : "waiting for arm")) << "\""
        << ",\"hydrophone_fresh\":" << (hydrophone_fresh() ? "true" : "false")
        << ",\"hydrophone_age_s\":" << age(hydrophone_received_)
        << ",\"hydrophone_source\":\"" << selected_hydrophone_source() << "\""
        << ",\"hydrophone_upstream_fresh\":"
        << (upstream_direction_fresh() ? "true" : "false")
        << ",\"hydrophone_internal_fallback_enabled\":"
        << (allow_internal_hydrophone_direction_fallback_ ? "true" : "false")
        << ",\"hydrophone_upstream_preferred\":"
        << (prefer_upstream_hydrophone_direction_ ? "true" : "false")
        << ",\"hydrophone_acoustic_position_fusion_enabled\":"
        << (use_acoustic_position_fusion_ ? "true" : "false")
        << ",\"hydrophone_phase_range_position_fusion_enabled\":"
        << (use_phase_range_position_fusion_ ? "true" : "false")
        << ",\"hydrophone_upstream_direction\":[" << hydrophone_direction_[0] << ","
        << hydrophone_direction_[1] << "," << hydrophone_direction_[2] << "]"
        << ",\"hydrophone_direction\":[" << selected_direction[0] << ","
        << selected_direction[1] << "," << selected_direction[2] << "]"
        << ",\"hydrophone_direction_body\":[" << selected_direction_body[0] << ","
        << selected_direction_body[1] << "," << selected_direction_body[2] << "]"
        << ",\"hydrophone_fit_samples\":" << range_motion_samples_.size()
        << ",\"hydrophone_fit_condition\":" << fitted_direction_condition_
        << ",\"hydrophone_fit_residual_m\":" << fitted_direction_residual_
        << ",\"hydrophone_fit_direction\":["
        << (fitted_direction_world_ ? fitted_direction_world_->x() : 0.0) << ","
        << (fitted_direction_world_ ? fitted_direction_world_->y() : 0.0) << ","
        << (fitted_direction_world_ ? fitted_direction_world_->z() : 0.0) << "]"
        << ",\"pinger_position_fit_xy\":["
        << (fitted_pinger_xy_ ? fitted_pinger_xy_->x() : 0.0) << ","
        << (fitted_pinger_xy_ ? fitted_pinger_xy_->y() : 0.0) << "]"
        << ",\"pinger_position_fit_condition\":" << fitted_pinger_position_condition_
        << ",\"pinger_position_fit_residual_m\":" << fitted_pinger_position_residual_
        << ",\"pinger_position_fit_samples\":" << acoustic_position_samples_.size()
        << ",\"pinger_position_fit_strong\":"
        << (pinger_position_fit_geometrically_strong() ? "true" : "false")
        << ",\"pinger_position_fit_promoted\":"
        << (pinger_acoustic_position_promoted_ ? "true" : "false")
        << ",\"pinger_position_near_field_locked\":"
        << (acoustic_position_near_field_locked() ? "true" : "false")
        << ",\"pinger_position_candidate_xy\":["
        << (acoustic_candidate_xy_ ? acoustic_candidate_xy_->x() : 0.0) << ","
        << (acoustic_candidate_xy_ ? acoustic_candidate_xy_->y() : 0.0) << "]"
        << ",\"pinger_position_candidate_streak\":" << acoustic_candidate_streak_
        << ",\"pinger_position_candidate_condition\":" << acoustic_candidate_condition_
        << ",\"pinger_position_candidate_residual_m\":" << acoustic_candidate_residual_
        << ",\"phase_range_position_fit_xy\":["
        << (phase_fitted_pinger_xy_ ? phase_fitted_pinger_xy_->x() : 0.0) << ","
        << (phase_fitted_pinger_xy_ ? phase_fitted_pinger_xy_->y() : 0.0) << "]"
        << ",\"phase_range_position_fit_initial_range_m\":"
        << phase_fitted_initial_range_m_
        << ",\"phase_range_position_fit_bias_mps\":"
        << phase_fitted_bias_range_rate_mps_
        << ",\"phase_range_position_fit_condition\":"
        << phase_fitted_position_condition_
        << ",\"phase_range_position_fit_residual_m\":"
        << phase_fitted_position_residual_
        << ",\"phase_range_position_fit_samples\":"
        << phase_range_position_samples_.size()
        << ",\"phase_range_measurement_delay_s\":"
        << phase_range_measurement_delay_s_
        << ",\"phase_range_seed_direction\":["
        << (phase_seed_direction_world_ ? phase_seed_direction_world_->x() : 0.0) << ","
        << (phase_seed_direction_world_ ? phase_seed_direction_world_->y() : 0.0) << ","
        << (phase_seed_direction_world_ ? phase_seed_direction_world_->z() : 0.0) << "]"
        << ",\"phase_range_position_fit_age_s\":"
        << age(phase_fitted_position_received_)
        << ",\"pinger_elapsed_s\":"
        << (pinger_started_ ? age(*pinger_started_) : 0.0)
        << ",\"pinger_completed_elapsed_s\":" << pinger_completed_elapsed_s_
        << ",\"pinger_motion_stage\":\"" << pinger_motion_stage() << "\""
        << ",\"pinger_vertical_alignment_required\":"
        << (vertical_alignment_required ? "true" : "false")
        << ",\"pinger_depth_hold_heave\":"
        << pinger_depth_hold_heave(pinger_command_depth_z())
        << ",\"pinger_acoustic_vertical_heave\":"
        << (hydrophone_fresh()
                ? pinger_acoustic_vertical_heave(selected_direction_body)
                : 0.0)
        << ",\"pinger_acoustic_vertical_scale\":"
        << pinger_acoustic_vertical_scale()
        << ",\"pinger_effective_capture_aim_offset_x\":"
        << pinger_effective_capture_aim_offset_x()
        << ",\"pinger_probe_forward\":" << pinger_probe_forward_
        << ",\"pinger_probe_yaw\":" << pinger_probe_yaw_
        << ",\"pinger_homing_drive_s\":" << pinger_homing_drive_s_
        << ",\"pinger_homing_pause_s\":" << pinger_homing_pause_s_
        << ",\"pinger_homing_drive_active\":"
        << (pinger_homing_drive_active() ? "true" : "false")
        << ",\"pinger_heading_drive_blocked\":"
        << (pinger_heading_drive_blocked_ ? "true" : "false")
        << ",\"pinger_path_length_m\":" << pinger_path_length_m_
        << ",\"pinger_search_reentry_count\":" << pinger_search_reentry_count_
        << ",\"pinger_recovery_count\":" << pinger_recovery_count_
        << ",\"pinger_spin_accumulated_yaw_rad\":"
        << pinger_spin_accumulated_yaw_rad_
        << ",\"pinger_spin_rehome_active\":"
        << (pinger_spin_rehome_active_ ? "true" : "false")
        << ",\"pinger_spin_rehome_count\":" << pinger_spin_rehome_count_
        << ",\"acoustic_range_m\":" << (acoustic_range_m_ ? *acoustic_range_m_ : -1.0)
        << ",\"pinger_leg_start_range_m\":"
        << (pinger_leg_start_range_m_ ? *pinger_leg_start_range_m_ : -1.0)
        << ",\"pinger_leg_best_range_m\":"
        << (pinger_leg_best_range_m_ ? *pinger_leg_best_range_m_ : -1.0)
        << ",\"pinger_leg_progress_source\":\"phase_delta_range\""
        << ",\"pinger_leg_progress_m\":" << pinger_leg_progress_m()
        << ",\"pinger_leg_regression_m\":" << pinger_leg_regression_m()
        << ",\"pinger_range_worsening\":"
        << (pinger_range_worsening_ ? "true" : "false")
        << ",\"pinger_doppler_approach_confirmed\":"
        << (pinger_doppler_approach_confirmed_ ? "true" : "false")
        << ",\"pinger_doppler_approach_count\":" << pinger_doppler_approach_count_
        << ",\"pinger_doppler_recede_count\":" << pinger_doppler_recede_count_
        << ",\"pinger_doppler_reversal\":"
        << (pinger_doppler_reversal_detected_ ? "true" : "false")
        << ",\"pinger_recovery_probe\":"
        << (pinger_recovery_probe_ ? "true" : "false")
        << ",\"pinger_guidance_range_m\":"
        << (dynamic_pinger_range_m() ? *dynamic_pinger_range_m() : -1.0)
        << ",\"pinger_control_range_m\":"
        << (pinger_control_range_m() ? *pinger_control_range_m() : -1.0)
        << ",\"pinger_yolo_capture_required\":"
        << (require_pinger_yolo_for_capture_ ? "true" : "false")
        << ",\"pinger_visual_acquired\":"
        << (pinger_visual_acquired_ ? "true" : "false")
        << ",\"pinger_visual_age_s\":" << age(pinger_visual_last_seen_)
        << ",\"observation_fresh\":" << (observation_fresh() ? "true" : "false")
        << ",\"observation_age_s\":" << age(observation_received_)
        << ",\"odom_age_s\":" << age(odom_received_)
        << ",\"robot_xyz\":[" << odom_.pose.pose.position.x << ","
        << odom_.pose.pose.position.y << "," << odom_.pose.pose.position.z << "]"
        << ",\"target_label\":\"" << observation_.class_label << "\""
        << ",\"target_confidence\":" << observation_.confidence
        << ",\"error_norm\":[" << filtered_error_x_ << "," << filtered_error_y_ << "]"
        << ",\"bbox_height_ratio\":" << bbox_ratio()
        << ",\"alignment_hold_s\":"
        << (alignment_started_ ? age(*alignment_started_) : 0.0)
        << ",\"capture_heading_locked\":"
        << (capture_heading_valid_ ? "true" : "false")
        << ",\"capture_aim_offset\":[" << capture_aim_offset_x_ << ","
        << capture_aim_offset_y_ << "]"
        << ",\"collector_fresh\":" << (age(collector_received_) < 1.5 ? "true" : "false")
        << ",\"collector_target_id\":\"" << collector_.target_id << "\""
        << ",\"collector_eq_active\":"
        << (collector_.collector_eq_active ? "true" : "false")
        << ",\"capture_state\":\"" << collector_.capture_state << "\""
        << ",\"target_class\":\"" << observation_.class_label << "\""
        << ",\"target_id\":\"" << collector_.target_id << "\""
        << ",\"detached_count\":" << detached_count_
        << ",\"expected_detach_count\":" << expected_detach_count_
        << ",\"netted_count\":" << netted_count_
        << ",\"expected_net_count\":" << expected_net_count_
        << ",\"released_count\":" << released_count_
        << ",\"remaining_attached\":"
        << std::max(0, expected_detach_count_ - detached_count_)
        << ",\"processed_count\":" << detached_count_
        << ",\"collected_count\":" << netted_count_
        << ",\"scored_count\":" << released_count_
        << ",\"failed_count\":0"
        << ",\"capture_flag\":"
        << (collector_.collector_eq_active ? "true" : "false")
        << ",\"own_course\":\"" << own_course_ << "\""
        << ",\"course_boundary_x\":" << course_boundary_x_
        << ",\"score_zone\":{\"xyz\":[" << score_x_ << "," << score_y_
        << "," << surface_collect_depth_z_ << "],\"radius\":" << score_radius_
        << ",\"radius_m\":" << score_radius_ << "}"
        << ",\"mission_policy\":{\"own_course\":\"" << own_course_
        << "\",\"course_boundary_x_m\":" << course_boundary_x_
        << ",\"course_boundary_margin_m\":" << course_boundary_margin_
        << ",\"course_boundary_standoff_m\":" << course_boundary_standoff_ << "}"
        << ",\"robot\":{\"x\":" << odom_.pose.pose.position.x
        << ",\"y\":" << odom_.pose.pose.position.y
        << ",\"z\":" << odom_.pose.pose.position.z
        << ",\"depth_m\":" << -odom_.pose.pose.position.z
        << ",\"yaw_rad\":" << current_yaw_rad() << "}"
        << ",\"detection\":{\"buoy_id\":\"" << collector_.target_id
        << "\",\"class_name\":\"" << observation_.class_label
        << "\",\"confidence\":" << observation_.confidence
        << ",\"bearing_rad\":" << observation_.bearing_rad
        << ",\"distance_m\":"
        << (observation_.range_valid ? observation_.range_m : -1.0) << "}"
        << ",\"search_waypoint_index\":" << search_waypoint_index_
        << ",\"search_waypoint_count\":" << search_waypoints_.size()
        << ",\"target_in_own_course\":"
        << (target_in_own_course(!surface_phase()) ? "true" : "false")
        << ",\"failure\":\"" << failure_reason_ << "\""
        << ",\"command\":{\"forward\":" << command.forward
        << ",\"sway\":" << command.sway << ",\"heave\":" << command.heave
        << ",\"yaw\":" << command.yaw << ",\"pitch\":" << command.pitch << "}"
        << ",\"buoys\":[";
    bool first_buoy = true;
    const auto append_buoy = [&out, &first_buoy](
        const std::string &id, const char *class_name, const char *state) {
      if (!first_buoy) out << ',';
      first_buoy = false;
      out << "{\"id\":\"" << id << "\",\"class_name\":\"" << class_name
          << "\",\"state\":\"" << state << "\",\"processed\":true}";
    };
    for (const auto &id : detached_target_ids_) {
      if (netted_target_ids_.count(id) == 0 && released_target_ids_.count(id) == 0) {
        append_buoy(id, "buoy", "DETACHED");
      }
    }
    for (const auto &id : netted_target_ids_) {
      append_buoy(id, "red_buoy", released_target_ids_.count(id) ? "RELEASED" : "NETTED");
    }
    for (const auto &id : released_target_ids_) {
      if (netted_target_ids_.count(id) == 0) append_buoy(id, "red_buoy", "RELEASED");
    }
    out << "]}";
    msg.data = out.str();
    status_pub_->publish(msg);
    write_status_json(msg.data);
  }

  void write_status_json(const std::string &payload) {
    if (status_json_path_.empty() ||
        age(status_json_written_) < std::max(0.02, status_json_period_s_)) {
      return;
    }
    status_json_written_ = Clock::now();
    try {
      const std::filesystem::path path(status_json_path_);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }
      const std::filesystem::path temporary = path.string() + ".tmp";
      {
        std::ofstream stream(temporary, std::ios::out | std::ios::trunc);
        if (!stream) throw std::runtime_error("cannot open temporary status file");
        stream << payload << '\n';
        if (!stream) throw std::runtime_error("cannot write temporary status file");
      }
      std::error_code error;
      std::filesystem::rename(temporary, path, error);
      if (error) {
        std::filesystem::remove(path, error);
        error.clear();
        std::filesystem::rename(temporary, path, error);
      }
      if (error) throw std::runtime_error(error.message());
    } catch (const std::exception &error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "status JSON write failed path=%s: %s",
          status_json_path_.c_str(), error.what());
    }
  }

  std::string observation_topic_, collector_topic_, odom_topic_, state_topic_, output_topic_, status_topic_;
  std::string vision_enable_topic_, vision_state_topic_, vision_state_;
  std::string status_json_path_;
  std::string hydrophone_direction_topic_, hydrophone_direction_frame_, hydrophone_body_topic_;
  std::string delta_range_topic_, iq_magnitude_topic_;
  std::string target_class_name_, own_course_;
  std::vector<std::string> pinger_visual_class_names_;
  std::vector<std::string> underwater_visual_class_names_;
  bool enabled_{false}, dry_run_{true}, require_armed_{true}, invert_heave_{true}, invert_yaw_{true};
  bool delegate_vision_control_{true};
  bool vision_complete_requires_detach_count_{true};
  bool use_pinger_first_{true};
  bool require_pinger_yolo_for_capture_{true};
  bool start_surface_phase_{false};
  bool require_pinger_position_fit_{true};
  bool use_acoustic_position_fusion_{false};
  double pinger_acoustic_position_lock_range_m_{1.4};
  double pinger_acoustic_position_min_range_m_{0.0};
  bool use_phase_range_position_fusion_{false};
  bool allow_internal_hydrophone_direction_fallback_{false};
  bool prefer_upstream_hydrophone_direction_{true};
  bool prefer_internal_hydrophone_direction_{true};
  bool pinger_visual_acquired_{false};
  bool pinger_detached_observed_{false};
  bool non_pinger_detached_during_pinger_{false};
  int expected_detach_count_{1}, expected_net_count_{1}, detached_count_{0}, netted_count_{0};
  int released_count_{0};
  int acknowledged_detach_count_{0}, acknowledged_netted_count_{0};
  double hydrophone_timeout_s_{3.0}, pinger_position_fit_timeout_s_{8.0};
  double pinger_min_probe_s_{6.0};
  double phase_range_position_timeout_s_{120.0};
  double phase_range_min_fit_duration_s_{11.5};
  double phase_range_measurement_delay_s_{0.128};
  double pinger_max_probe_s_{45.0};
  double pinger_forward_fast_{0.55}, pinger_forward_turn_{0.10};
  double pinger_heading_drive_tolerance_rad_{0.22};
  double pinger_near_slow_range_m_{8.0}, pinger_near_forward_{0.30};
  double pinger_final_slow_range_m_{2.5}, pinger_final_forward_{0.18};
  double pinger_probe_forward_{0.24}, pinger_probe_yaw_{0.30};
  double pinger_homing_leg_s_{10.0};
  double pinger_homing_sway_amplitude_{0.0};
  double pinger_homing_sway_period_s_{6.0};
  double pinger_homing_yaw_dither_amplitude_{0.06};
  double pinger_homing_yaw_dither_period_s_{5.0};
  double pinger_homing_drive_s_{0.0};
  double pinger_homing_pause_s_{0.0};
  double pinger_spin_rehome_yaw_rad_{5.75};
  double pinger_spin_rehome_max_translation_m_{0.75};
  double pinger_spin_rehome_stop_s_{1.0};
  double pinger_position_fit_bearing_tolerance_rad_{0.45};
  double pinger_range_regression_margin_m_{0.40};
  double pinger_range_regression_hold_s_{0.65};
  double pinger_range_progress_grace_s_{0.80};
  double pinger_range_progress_check_s_{2.8};
  double pinger_range_min_progress_m_{0.12};
  double pinger_doppler_approach_delta_m_{0.003};
  double pinger_doppler_recede_delta_m_{0.003};
  double pinger_doppler_reversal_max_range_m_{4.0};
  int pinger_doppler_approach_samples_{3};
  int pinger_doppler_recede_samples_{2};
  double pinger_doppler_brake_s_{0.70};
  double pinger_doppler_brake_reverse_{-0.22};
  double pinger_depth_z_{-8.5}, pinger_transit_depth_z_{-7.7};
  double pinger_acoustic_source_depth_z_{-8.865};
  double pinger_depth_transition_range_m_{3.0}, pinger_depth_kp_{0.18};
  double pinger_yaw_kp_{0.85}, pinger_yaw_kd_{0.16}, pinger_yaw_limit_{0.44};
  double pinger_scan_yaw_{0.07};
  double pinger_scan_yaw_gain_{0.80}, pinger_scan_yaw_limit_{0.30};
  double pinger_acoustic_crawl_forward_{0.20};
  double pinger_acoustic_crawl_bearing_rad_{0.45};
  double pinger_acoustic_capture_range_m_{0.78};
  double pinger_acoustic_capture_bearing_rad_{0.16};
  double pinger_heave_kp_{0.42}, pinger_heave_limit_{0.34};
  double pinger_vertical_direction_deadband_{0.08};
  double pinger_vertical_alignment_tolerance_{0.22};
  double pinger_vertical_forward_limit_{0.0}, pinger_vertical_yaw_limit_{0.08};
  double pinger_acoustic_vertical_zero_range_m_{0.80};
  double pinger_acoustic_vertical_full_range_m_{2.0};
  double pinger_yolo_min_bbox_ratio_{0.06};
  double underwater_target_depth_z_{-8.5}, underwater_depth_kp_{0.30};
  double underwater_visual_heave_gain_{0.05};
  double pinger_yolo_acoustic_range_m_{2.5}, pinger_iq_range_reference_{0.325};
  double pinger_visual_bearing_tolerance_rad_{0.35};
  double pinger_visual_reacquire_timeout_s_{1.0};
  double pinger_camera_hfov_rad_{1.211};
  double pinger_capture_commit_range_m_{0.55};
  double pinger_capture_min_bbox_ratio_{0.08};
  double pinger_rake_lane_blend_start_m_{0.90}, pinger_rake_lane_full_range_m_{0.45};
  double observation_timeout_s_{1.5}, odom_timeout_s_{0.8}, state_timeout_s_{8.0};
  double vision_state_timeout_s_{1.5}, vision_search_handoff_s_{0.75};
  double vehicle_disconnect_grace_s_{0.0};
  double collector_timeout_s_{6.0}, capture_drive_s_{1.4};
  double capture_insert_forward_{0.28};
  double capture_backoff_s_{0.55}, capture_backoff_forward_{-0.16};
  double capture_heading_kp_{0.90}, capture_heading_kd_{0.15};
  double capture_heading_yaw_limit_{0.18};
  double capture_center_tolerance_x_{0.08}, capture_center_tolerance_y_{0.12};
  double capture_alignment_hold_s_{0.35};
  double capture_aim_offset_x_{0.0}, capture_aim_offset_y_{0.0};
  double alignment_hold_s_{0.15}, fine_bbox_ratio_{0.18}, capture_bbox_ratio_{0.32}, center_tolerance_{0.16};
  double min_confidence_{0.35}, observation_filter_alpha_{0.45};
  double yaw_kp_{0.30}, yaw_kd_{0.02}, heave_kp_{0.22};
  double max_visual_yaw_{0.18}, max_visual_heave_{0.18}, search_yaw_{0.20};
  double command_slew_per_s_{0.8};
  double course_boundary_x_{0.0}, course_boundary_margin_{0.8}, course_boundary_standoff_{0.7};
  double detached_exclusion_radius_m_{1.8};
  double search_area_center_x_{-8.5}, search_area_center_y_{0.0};
  double search_area_width_m_{9.0}, search_area_height_m_{16.0};
  double search_lane_spacing_m_{3.0}, search_waypoint_tolerance_m_{0.9};
  double search_forward_{0.48}, search_turn_yaw_limit_{0.65};
  double search_heading_tolerance_rad_{0.70}, search_escape_forward_{0.55};
  double search_accept_bbox_ratio_{0.065};
  double target_height_m_{0.55}, camera_vertical_fov_rad_{0.75};
  double score_x_{0.0}, score_y_{0.0}, score_radius_{0.8}, release_timeout_s_{8.0};
  double surface_collect_depth_z_{-0.31}, surface_collect_depth_tolerance_m_{0.12};
  double surface_depth_kp_{0.55}, surface_yaw_kp_{0.55}, surface_yaw_limit_{0.35};
  double surface_forward_{0.45}, surface_turn_forward_{0.0};
  double surface_center_tolerance_{0.10}, surface_steer_timeout_s_{0.65};
  double status_json_period_s_{0.10};
  double rc_span_{400.0};
  Phase phase_{Phase::IDLE};
  Clock::time_point phase_started_{}, mission_started_{}, observation_received_{}, collector_received_{}, odom_received_{}, state_received_{};
  Clock::time_point vision_state_received_{};
  Clock::time_point last_vehicle_connected_{}, last_vehicle_armed_{};
  Clock::time_point hydrophone_received_{};
  Clock::time_point pinger_visual_last_seen_{};
  Clock::time_point iq_magnitude_received_{};
  Clock::time_point raw_acoustic_range_received_{}, fitted_pinger_position_received_{};
  Clock::time_point phase_range_origin_time_{}, phase_position_fit_attempted_{};
  Clock::time_point phase_fitted_position_received_{};
  Clock::time_point delta_range_received_{}, fitted_direction_received_{}, fit_previous_odom_time_{};
  Clock::time_point observation_filter_time_{}, previous_command_time_{};
  Clock::time_point status_json_written_{};
  std::optional<Clock::time_point> alignment_started_;
  std::optional<Clock::time_point> pinger_started_;
  std::optional<Clock::time_point> pinger_range_worsening_started_;
  bool pinger_range_worsening_{false};
  bool pinger_recovery_probe_{false};
  bool pinger_heading_drive_blocked_{false};
  bool pinger_spin_rehome_active_{false};
  bool pinger_spin_watch_initialized_{false};
  double pinger_spin_accumulated_yaw_rad_{0.0};
  double pinger_spin_previous_yaw_rad_{0.0};
  Eigen::Vector2d pinger_spin_origin_xy_{0.0, 0.0};
  int pinger_spin_rehome_count_{0};
  int pinger_doppler_approach_count_{0};
  int pinger_doppler_recede_count_{0};
  bool pinger_doppler_approach_confirmed_{false};
  bool pinger_doppler_reversal_detected_{false};
  bool pinger_leg_phase_range_valid_{false};
  double pinger_leg_phase_range_start_m_{0.0};
  double pinger_leg_best_phase_range_change_m_{0.0};
  bool filter_initialized_{false};
  bool capture_heading_valid_{false};
  double capture_heading_yaw_rad_{0.0}, capture_depth_z_{0.0};
  double filtered_error_x_{0.0}, filtered_error_y_{0.0}, filtered_height_ratio_{0.0};
  double filtered_error_rate_x_{0.0};
  Command previous_command_{};
  std::uint64_t collector_sequence_{0}, phase_collector_sequence_{0};
  std::unordered_set<std::string> detached_target_ids_;
  std::unordered_set<std::string> netted_target_ids_;
  std::unordered_set<std::string> released_target_ids_;
  std::vector<Eigen::Vector2d> detached_contact_xy_;
  std::vector<Eigen::Vector2d> search_waypoints_;
  std::size_t search_waypoint_index_{0};
  bool search_waypoint_initialized_{false};
  std::array<double, 3> hydrophone_direction_{{1.0, 0.0, 0.0}};
  std::optional<Eigen::Vector3d> fit_previous_position_;
  std::optional<Eigen::Vector3d> pinger_path_previous_position_;
  std::optional<Eigen::Vector3d> fitted_direction_world_;
  std::optional<Eigen::Vector2d> fitted_pinger_xy_;
  std::optional<Eigen::Vector2d> acoustic_candidate_xy_;
  std::optional<Eigen::Vector2d> phase_fitted_pinger_xy_;
  std::optional<Eigen::Vector3d> phase_seed_direction_world_;
  std::optional<double> acoustic_range_m_;
  std::optional<double> latest_raw_acoustic_range_m_;
  std::optional<double> phase_initial_acoustic_range_m_;
  std::optional<double> pinger_leg_start_range_m_;
  std::optional<double> pinger_leg_best_range_m_;
  std::deque<RangeMotionSample> range_motion_samples_;
  std::deque<AcousticPositionSample> acoustic_position_samples_;
  std::deque<PhaseRangePositionSample> phase_range_position_samples_;
  std::deque<TimedPhasePosition> phase_odom_history_;
  std::deque<double> acoustic_range_window_;
  std::deque<double> raw_delta_range_window_;
  double pending_delta_range_{0.0};
  double cumulative_phase_range_change_{0.0};
  int pending_delta_count_{0};
  double fitted_direction_condition_{0.0}, fitted_direction_residual_{0.0};
  double fitted_pinger_position_condition_{0.0}, fitted_pinger_position_residual_{0.0};
  double pinger_path_length_m_{0.0};
  double pinger_completed_elapsed_s_{0.0};
  int pinger_search_reentry_count_{0}, pinger_recovery_count_{0};
  int acoustic_candidate_streak_{0};
  bool pinger_acoustic_position_promoted_{false};
  double acoustic_candidate_condition_{0.0}, acoustic_candidate_residual_{0.0};
  double phase_fitted_initial_range_m_{0.0};
  double phase_fitted_bias_range_rate_mps_{0.0};
  double phase_fitted_position_condition_{0.0}, phase_fitted_position_residual_{0.0};
  std::string failure_reason_;
  hit25_auv_ros2_msg::msg::BuoyObservation observation_{};
  hit25_auv_ros2_msg::msg::CollectorState collector_{};
  nav_msgs::msg::Odometry odom_{};
  mavros_msgs::msg::State vehicle_state_{};
  rclcpp::Subscription<hit25_auv_ros2_msg::msg::BuoyObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vision_state_sub_;
  rclcpp::Subscription<hit25_auv_ros2_msg::msg::CollectorState>::SharedPtr collector_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr hydrophone_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr delta_range_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr iq_magnitude_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr hydrophone_body_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_enable_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObservationMissionFsm>());
  rclcpp::shutdown();
  return 0;
}
