#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

class MissionStateMachineNode : public rclcpp::Node
{
public:
  MissionStateMachineNode()
  : Node("mission_state_machine_node")
  {
    bbox_topic_ = declare_parameter<std::string>("bbox_topic", "/vision/buoy_bbox");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/auv/depth");
    depth_pose_topic_ = declare_parameter<std::string>("depth_pose_topic", "/depth/pose");
    depth_pose_scale_ = declare_parameter<double>("depth_pose_scale", 1.0);
    depth_pose_offset_m_ = declare_parameter<double>("depth_pose_offset_m", 0.0);
    depth_odom_topic_ = declare_parameter<std::string>(
      "depth_odom_topic", "/odometry/filtered");
    depth_odom_scale_ = declare_parameter<double>("depth_odom_scale", -1.0);
    depth_odom_offset_m_ = declare_parameter<double>("depth_odom_offset_m", 0.0);
    enable_topic_ = declare_parameter<std::string>("enable_topic", "/control/vision/enable");
    state_topic_ = declare_parameter<std::string>("state_topic", "/control/vision/state");
    status_topic_ = declare_parameter<std::string>("status_topic", "/control/vision/status");
    vehicle_state_topic_ =
      declare_parameter<std::string>("vehicle_state_topic", "/mavros/state");
    rc_override_topic_ =
      declare_parameter<std::string>("rc_override_topic", "/control/vision/rc_override");
    dry_run_ = declare_parameter<bool>("dry_run", false);
    require_armed_ = declare_parameter<bool>("require_armed", true);
    vehicle_state_timeout_sec_ = declare_parameter<double>("vehicle_state_timeout_sec", 8.0);

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    detection_timeout_sec_ = declare_parameter<double>("detection_timeout_sec", 0.7);
    depth_timeout_sec_ = declare_parameter<double>("depth_timeout_sec", 1.0);
    work_depth_m_ = declare_parameter<double>("work_depth_m", 9.5);
    surface_depth_m_ = declare_parameter<double>("surface_depth_m", 0.4);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 10.5);
    depth_tolerance_m_ = declare_parameter<double>("depth_tolerance_m", 0.2);
    depth_stable_sec_ = declare_parameter<double>("depth_stable_sec", 2.0);
    depth_kp_pwm_per_m_ = declare_parameter<double>("depth_kp_pwm_per_m", 120.0);
    max_depth_delta_pwm_ = declare_parameter<int>("max_depth_delta_pwm", 160);

    buoy_class_id_ = declare_parameter<int>("buoy_class_id", 0);
    stick_class_id_ = declare_parameter<int>("stick_class_id", 1);
    min_detection_hits_ = declare_parameter<int>("min_detection_hits", 3);
    approach_area_ratio_ = declare_parameter<double>("approach_area_ratio", 0.12);
    search_timeout_sec_ = declare_parameter<double>("search_timeout_sec", 20.0);
    area_verify_sec_ = declare_parameter<double>("area_verify_sec", 12.0);

    fork_target_x_ = declare_parameter<double>("fork_target_x", 0.5);
    fork_target_y_ = declare_parameter<double>("fork_target_y", 0.5);
    stick_deadband_x_ = declare_parameter<double>("stick_deadband_x", 0.06);
    stick_deadband_y_ = declare_parameter<double>("stick_deadband_y", 0.08);
    align_stable_sec_ = declare_parameter<double>("align_stable_sec", 0.7);

    insert_pwm_ = declare_parameter<int>("insert_pwm", 1560);
    insert_duration_sec_ = declare_parameter<double>("insert_duration_sec", 0.8);
    detach_pwm_ = declare_parameter<int>("detach_pwm", 1620);
    detach_duration_sec_ = declare_parameter<double>("detach_duration_sec", 0.3);
    backoff_pwm_ = declare_parameter<int>("backoff_pwm", 1420);
    backoff_duration_sec_ = declare_parameter<double>("backoff_duration_sec", 0.5);
    verify_clear_sec_ = declare_parameter<double>("verify_clear_sec", 1.0);
    verify_timeout_sec_ = declare_parameter<double>("verify_timeout_sec", 3.0);
    max_target_retries_ = declare_parameter<int>("max_target_retries", 2);
    expected_target_count_ = declare_parameter<int>("expected_target_count", 0);

    throttle_channel_ = declare_parameter<int>("throttle_channel", 3);
    yaw_channel_ = declare_parameter<int>("yaw_channel", 4);
    forward_channel_ = declare_parameter<int>("forward_channel", 5);
    neutral_pwm_ = declare_parameter<int>("neutral_pwm", 1500);
    min_pwm_ = declare_parameter<int>("min_pwm", 1300);
    max_pwm_ = declare_parameter<int>("max_pwm", 1700);
    max_yaw_delta_ = declare_parameter<int>("max_yaw_delta", 180);
    max_tracking_depth_delta_ = declare_parameter<int>("max_tracking_depth_delta", 100);
    approach_forward_pwm_ = declare_parameter<int>("approach_forward_pwm", 1560);
    search_yaw_pwm_ = declare_parameter<int>("search_yaw_pwm", 1560);
    yaw_invert_ = declare_parameter<bool>("yaw_invert", false);
    vertical_positive_is_up_ = declare_parameter<bool>("vertical_positive_is_up", true);

    validate_parameters();

    bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      bbox_topic_, 10, std::bind(&MissionStateMachineNode::on_bbox, this, std::placeholders::_1));
    if (!depth_odom_topic_.empty()) {
      depth_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        depth_odom_topic_, 10,
        std::bind(&MissionStateMachineNode::on_depth_odom, this, std::placeholders::_1));
    } else if (!depth_topic_.empty()) {
      depth_sub_ = create_subscription<std_msgs::msg::Float64>(
        depth_topic_, 10,
        std::bind(&MissionStateMachineNode::on_depth, this, std::placeholders::_1));
    } else if (!depth_pose_topic_.empty()) {
      depth_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        depth_pose_topic_, 10,
        std::bind(&MissionStateMachineNode::on_depth_pose, this, std::placeholders::_1));
    }
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, 10, std::bind(&MissionStateMachineNode::on_enable, this, std::placeholders::_1));
    vehicle_state_sub_ = create_subscription<mavros_msgs::msg::State>(
      vehicle_state_topic_, 10,
      std::bind(&MissionStateMachineNode::on_vehicle_state, this, std::placeholders::_1));
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_override_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10));

    const double period_sec = 1.0 / std::max(1.0, control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_sec),
      std::bind(&MissionStateMachineNode::on_timer, this));

    state_entered_at_ = now();
    publish_state();
    RCLCPP_INFO(
      get_logger(),
      "Vision mission controller ready; enable=%s depth=%s depth_odom=%s depth_pose=%s bbox=%s state=%s rc=%s dry_run=%s",
      enable_topic_.c_str(), depth_topic_.c_str(), depth_odom_topic_.c_str(),
      depth_pose_topic_.c_str(), bbox_topic_.c_str(),
      state_topic_.c_str(), rc_override_topic_.c_str(), dry_run_ ? "true" : "false");
    RCLCPP_WARN(
      get_logger(),
      "Control starts disabled. Publish std_msgs/Bool true to %s after pre-flight checks.",
      enable_topic_.c_str());
  }

  void publish_release_once()
  {
    auto channels = nochange_channels();
    release_controlled_channels(channels);
    publish_channels(channels);
  }

private:
  enum class State
  {
    IDLE,
    DIVE,
    SEARCH,
    APPROACH_BUOY,
    ALIGN_STICK,
    INSERT_FORK,
    DETACH,
    BACKOFF,
    VERIFY_RELEASE,
    AREA_VERIFY,
    ASCEND,
    COMPLETE,
    FAILSAFE
  };

  struct Detection
  {
    float confidence{0.0F};
    float center_x{0.0F};
    float center_y{0.0F};
    float width{0.0F};
    float height{0.0F};
    float image_width{0.0F};
    float image_height{0.0F};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    int consecutive_hits{0};
  };

  void validate_parameters()
  {
    if (
      min_pwm_ < 1300 || max_pwm_ > 1700 || min_pwm_ >= max_pwm_ ||
      neutral_pwm_ < min_pwm_ || neutral_pwm_ > max_pwm_)
    {
      throw std::invalid_argument(
              "Thruster PWM range must stay within 1300..1700 and include neutral_pwm");
    }
    for (const int channel : {throttle_channel_, yaw_channel_, forward_channel_}) {
      if (channel < 1 || channel > 18) {
        throw std::invalid_argument("RC channel numbers must be in [1, 18]");
      }
    }
    if (
      throttle_channel_ == yaw_channel_ || throttle_channel_ == forward_channel_ ||
      yaw_channel_ == forward_channel_)
    {
      throw std::invalid_argument("Controlled RC channels must be unique");
    }
    if (work_depth_m_ <= surface_depth_m_ || max_depth_m_ <= work_depth_m_) {
      throw std::invalid_argument("Depths must satisfy surface_depth < work_depth < max_depth");
    }
    if (buoy_class_id_ == stick_class_id_) {
      throw std::invalid_argument("buoy_class_id and stick_class_id must differ");
    }
  }

  void on_enable(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool was_enabled = enabled_;
    enabled_ = msg->data;
    if (!enabled_) {
      transition_to(State::IDLE, "control disabled");
    } else if (!was_enabled) {
      target_success_count_ = 0;
      target_retries_ = 0;
      buoy_.reset();
      stick_.reset();
      transition_to(State::IDLE, "control enabled; waiting for vehicle and depth");
    }
  }

  void on_vehicle_state(const mavros_msgs::msg::State::SharedPtr msg)
  {
    vehicle_state_ = *msg;
    vehicle_state_received_at_ = now();
    vehicle_state_seen_ = true;
  }

  void on_depth(const std_msgs::msg::Float64::SharedPtr msg)
  {
    accept_depth(msg->data, "float64");
  }

  void on_depth_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    accept_depth(
      depth_pose_scale_ * msg->pose.pose.position.z + depth_pose_offset_m_, "pose");
  }

  void on_depth_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    accept_depth(
      depth_odom_scale_ * msg->pose.pose.position.z + depth_odom_offset_m_, "odometry");
  }

  void accept_depth(double depth_m, const char * source)
  {
    if (!std::isfinite(depth_m) || depth_m < 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid depth value");
      return;
    }
    depth_m_ = depth_m;
    depth_source_ = source;
    depth_received_at_ = now();
  }

  void on_bbox(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 10) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring bbox with fewer than 10 values");
      return;
    }
    for (size_t index = 0; index < 10; ++index) {
      if (!std::isfinite(msg->data[index])) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring bbox containing NaN/Inf");
        return;
      }
    }
    if (msg->data[1] < 0.5F || msg->data[8] <= 0.0F || msg->data[9] <= 0.0F) {
      return;
    }

    const int class_id = static_cast<int>(std::lround(msg->data[2]));
    if (class_id == buoy_class_id_) {
      update_detection(buoy_, msg);
    } else if (class_id == stick_class_id_) {
      update_detection(stick_, msg);
    }
  }

  void update_detection(
    std::optional<Detection> & slot,
    const std_msgs::msg::Float32MultiArray::SharedPtr & msg)
  {
    const auto received_at = now();
    int hits = 1;
    if (slot && (received_at - slot->received_at).seconds() <= detection_timeout_sec_) {
      hits = slot->consecutive_hits + 1;
    }
    slot = Detection{
      msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7],
      msg->data[8], msg->data[9], received_at, hits};
  }

  void on_timer()
  {
    auto channels = nochange_channels();

    if (!enabled_) {
      if (state_ != State::IDLE) {
        transition_to(State::IDLE, "control disabled");
      }
      release_controlled_channels(channels);
      publish_channels(channels);
      publish_status();
      return;
    }

    if (!vehicle_ready()) {
      if (state_ != State::IDLE) {
        transition_to(State::IDLE, "vehicle disconnected, stale, or not armed");
      }
      release_controlled_channels(channels);
      publish_channels(channels);
      publish_status();
      return;
    }

    if (state_ == State::IDLE) {
      if (!has_recent_depth()) {
        release_controlled_channels(channels);
        publish_channels(channels);
        publish_status();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for a recent positive-down depth value on %s", depth_topic_.c_str());
        return;
      }
      target_retries_ = 0;
      transition_to(State::DIVE, "enabled with valid depth");
    }

    if (mission_state_requires_depth() && !has_recent_depth()) {
      transition_to(State::FAILSAFE, "depth input stale");
    }
    if (depth_m_ && *depth_m_ > max_depth_m_) {
      transition_to(State::FAILSAFE, "maximum depth exceeded");
    }

    switch (state_) {
      case State::IDLE:
        release_controlled_channels(channels);
        break;
      case State::DIVE:
        run_dive(channels);
        break;
      case State::SEARCH:
        run_search(channels);
        break;
      case State::APPROACH_BUOY:
        run_approach(channels);
        break;
      case State::ALIGN_STICK:
        run_align_stick(channels);
        break;
      case State::INSERT_FORK:
        set_neutral_control(channels);
        if (state_age_sec() >= insert_duration_sec_) {
          transition_to(State::DETACH, "fork insertion pulse complete");
          set_channel(channels, forward_channel_, detach_pwm_);
        } else {
          set_channel(channels, forward_channel_, insert_pwm_);
        }
        break;
      case State::DETACH:
        set_neutral_control(channels);
        if (state_age_sec() >= detach_duration_sec_) {
          transition_to(State::BACKOFF, "detach pulse complete");
          set_channel(channels, forward_channel_, backoff_pwm_);
        } else {
          set_channel(channels, forward_channel_, detach_pwm_);
        }
        break;
      case State::BACKOFF:
        set_neutral_control(channels);
        if (state_age_sec() >= backoff_duration_sec_) {
          transition_to(State::VERIFY_RELEASE, "backoff complete");
        } else {
          set_channel(channels, forward_channel_, backoff_pwm_);
        }
        break;
      case State::VERIFY_RELEASE:
        run_verify_release(channels);
        break;
      case State::AREA_VERIFY:
        run_area_verify(channels);
        break;
      case State::ASCEND:
        run_ascend(channels);
        break;
      case State::COMPLETE:
      case State::FAILSAFE:
        release_controlled_channels(channels);
        break;
    }

    publish_channels(channels);
    publish_status();
  }

  void run_dive(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    set_channel(channels, throttle_channel_, depth_control_pwm(work_depth_m_));
    if (std::abs(*depth_m_ - work_depth_m_) <= depth_tolerance_m_) {
      if (!condition_started_at_) {
        condition_started_at_ = now();
      } else if ((now() - *condition_started_at_).seconds() >= depth_stable_sec_) {
        transition_to(State::SEARCH, "work depth stable");
      }
    } else {
      condition_started_at_.reset();
    }
  }

  void run_search(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    set_channel(channels, throttle_channel_, depth_control_pwm(work_depth_m_));
    set_channel(channels, yaw_channel_, search_yaw_pwm_);
    if (confirmed_buoy()) {
      transition_to(State::APPROACH_BUOY, "confirmed buoy selected");
    } else if (state_age_sec() >= search_timeout_sec_) {
      transition_to(State::AREA_VERIFY, "initial search exhausted");
    }
  }

  void run_approach(std::array<uint16_t, 18> & channels)
  {
    if (!recent(buoy_)) {
      set_neutral_control(channels);
      transition_to(State::SEARCH, "buoy lost during approach");
      return;
    }
    apply_visual_tracking(channels, *buoy_, 0.5, 0.5, approach_forward_pwm_);
    if (detection_area_ratio(*buoy_) >= approach_area_ratio_ && recent(stick_)) {
      transition_to(State::ALIGN_STICK, "close buoy and stick visible");
    }
  }

  void run_align_stick(std::array<uint16_t, 18> & channels)
  {
    if (!recent(stick_)) {
      set_neutral_control(channels);
      transition_to(
        recent(buoy_) ? State::APPROACH_BUOY : State::SEARCH,
        "stick lost during fine alignment");
      return;
    }
    apply_visual_tracking(channels, *stick_, fork_target_x_, fork_target_y_, neutral_pwm_);
    const auto [error_x, error_y] = normalized_error(*stick_, fork_target_x_, fork_target_y_);
    if (std::abs(error_x) <= stick_deadband_x_ && std::abs(error_y) <= stick_deadband_y_) {
      if (!condition_started_at_) {
        condition_started_at_ = now();
      } else if ((now() - *condition_started_at_).seconds() >= align_stable_sec_) {
        transition_to(State::INSERT_FORK, "stick alignment stable");
      }
    } else {
      condition_started_at_.reset();
    }
  }

  void run_verify_release(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    if (!recent(buoy_) && state_age_sec() >= verify_clear_sec_) {
      target_retries_ = 0;
      ++target_success_count_;
      buoy_.reset();
      stick_.reset();
      if (expected_target_count_ > 0 && target_success_count_ >= expected_target_count_) {
        transition_to(State::ASCEND, "configured vision target count completed");
      } else {
        transition_to(State::SEARCH, "target absent after backoff; visual release confirmed");
      }
      return;
    }
    if (state_age_sec() >= verify_timeout_sec_) {
      if (target_retries_ < max_target_retries_) {
        ++target_retries_;
        transition_to(
          recent(stick_) ? State::ALIGN_STICK : State::APPROACH_BUOY,
          "target remains; retrying detach");
      } else {
        RCLCPP_ERROR(get_logger(), "Target retry limit reached; abandoning this target");
        target_retries_ = 0;
        buoy_.reset();
        stick_.reset();
        transition_to(State::SEARCH, "target retry limit reached");
      }
    }
  }

  void run_area_verify(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    set_channel(channels, throttle_channel_, depth_control_pwm(work_depth_m_));
    set_channel(channels, yaw_channel_, search_yaw_pwm_);
    if (confirmed_buoy()) {
      transition_to(State::APPROACH_BUOY, "buoy found during area verification");
    } else if (state_age_sec() >= area_verify_sec_) {
      if (expected_target_count_ > 0 && target_success_count_ < expected_target_count_) {
        transition_to(State::SEARCH, "area scan incomplete for configured target count");
      } else {
        transition_to(State::ASCEND, "area verification complete with no targets");
      }
    }
  }

  void run_ascend(std::array<uint16_t, 18> & channels)
  {
    set_neutral_control(channels);
    set_channel(channels, throttle_channel_, depth_control_pwm(surface_depth_m_));
    if (*depth_m_ <= surface_depth_m_ + depth_tolerance_m_) {
      transition_to(State::COMPLETE, "surface depth reached");
    }
  }

  void apply_visual_tracking(
    std::array<uint16_t, 18> & channels, const Detection & detection,
    double target_x, double target_y, int forward_pwm)
  {
    set_neutral_control(channels);
    const auto [error_x, error_y] = normalized_error(detection, target_x, target_y);
    const double yaw_sign = yaw_invert_ ? -1.0 : 1.0;
    const double vertical_sign = vertical_positive_is_up_ ? -1.0 : 1.0;
    set_channel(
      channels, yaw_channel_,
      neutral_pwm_ + static_cast<int>(yaw_sign * error_x * max_yaw_delta_));
    set_channel(
      channels, throttle_channel_,
      neutral_pwm_ + static_cast<int>(vertical_sign * error_y * max_tracking_depth_delta_));
    set_channel(channels, forward_channel_, forward_pwm);
  }

  std::pair<double, double> normalized_error(
    const Detection & detection, double target_x, double target_y) const
  {
    const double x = detection.center_x / detection.image_width;
    const double y = detection.center_y / detection.image_height;
    return {
      std::clamp((x - target_x) * 2.0, -1.0, 1.0),
      std::clamp((y - target_y) * 2.0, -1.0, 1.0)};
  }

  double detection_area_ratio(const Detection & detection) const
  {
    return static_cast<double>(detection.width * detection.height) /
           static_cast<double>(detection.image_width * detection.image_height);
  }

  int depth_control_pwm(double target_depth_m) const
  {
    const double error_m = target_depth_m - *depth_m_;
    const int delta = std::clamp(
      static_cast<int>(error_m * depth_kp_pwm_per_m_),
      -max_depth_delta_pwm_, max_depth_delta_pwm_);
    // Depth is positive-down. With positive throttle meaning up, descending needs a negative PWM delta.
    const int sign = vertical_positive_is_up_ ? -1 : 1;
    return neutral_pwm_ + sign * delta;
  }

  bool recent(const std::optional<Detection> & detection) const
  {
    return detection && (now() - detection->received_at).seconds() <= detection_timeout_sec_;
  }

  bool confirmed_buoy() const
  {
    return recent(buoy_) && buoy_->consecutive_hits >= min_detection_hits_;
  }

  bool has_recent_depth() const
  {
    return depth_m_ && (now() - depth_received_at_).seconds() <= depth_timeout_sec_;
  }

  bool vehicle_ready() const
  {
    if (!vehicle_state_seen_) {
      return false;
    }
    if ((now() - vehicle_state_received_at_).seconds() > vehicle_state_timeout_sec_) {
      return false;
    }
    return vehicle_state_.connected && (!require_armed_ || vehicle_state_.armed);
  }

  bool mission_state_requires_depth() const
  {
    return state_ != State::IDLE && state_ != State::COMPLETE && state_ != State::FAILSAFE;
  }

  double state_age_sec() const
  {
    return (now() - state_entered_at_).seconds();
  }

  void transition_to(State next, const std::string & reason)
  {
    if (state_ == next) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "State %s -> %s: %s", state_name(state_), state_name(next), reason.c_str());
    state_ = next;
    state_entered_at_ = now();
    condition_started_at_.reset();
    publish_state();
  }

  void publish_state()
  {
    if (!state_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = state_name(state_);
    state_pub_->publish(msg);
  }

  void publish_status()
  {
    // Republish the compact upstream state at the control rate so an external
    // mission orchestrator can apply a deterministic freshness timeout.
    publish_state();
    if (!status_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = std::string("{\"state\":\"") + state_name(state_) +
      "\",\"enabled\":" + (enabled_ ? "true" : "false") +
      ",\"dry_run\":" + (dry_run_ ? "true" : "false") +
      ",\"vehicle_ready\":" + (vehicle_ready() ? "true" : "false") +
      ",\"depth_source\":\"" + depth_source_ + "\"" +
      ",\"depth_m\":" + (depth_m_ ? std::to_string(*depth_m_) : "null") +
      ",\"target_success_count\":" + std::to_string(target_success_count_) +
      ",\"expected_target_count\":" + std::to_string(expected_target_count_) + "}";
    status_pub_->publish(msg);
  }

  static const char * state_name(State state)
  {
    switch (state) {
      case State::IDLE: return "IDLE";
      case State::DIVE: return "DIVE";
      case State::SEARCH: return "SEARCH";
      case State::APPROACH_BUOY: return "APPROACH_BUOY";
      case State::ALIGN_STICK: return "ALIGN_STICK";
      case State::INSERT_FORK: return "INSERT_FORK";
      case State::DETACH: return "DETACH";
      case State::BACKOFF: return "BACKOFF";
      case State::VERIFY_RELEASE: return "VERIFY_RELEASE";
      case State::AREA_VERIFY: return "AREA_VERIFY";
      case State::ASCEND: return "ASCEND";
      case State::COMPLETE: return "COMPLETE";
      case State::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  std::array<uint16_t, 18> nochange_channels() const
  {
    std::array<uint16_t, 18> channels{};
    channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
    return channels;
  }

  void set_neutral_control(std::array<uint16_t, 18> & channels)
  {
    set_channel(channels, throttle_channel_, neutral_pwm_);
    set_channel(channels, yaw_channel_, neutral_pwm_);
    set_channel(channels, forward_channel_, neutral_pwm_);
  }

  void release_controlled_channels(std::array<uint16_t, 18> & channels)
  {
    set_channel(channels, throttle_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, yaw_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    set_channel(channels, forward_channel_, mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
  }

  void set_channel(std::array<uint16_t, 18> & channels, int channel, int pwm) const
  {
    if (
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE &&
      pwm != mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE)
    {
      pwm = std::clamp(pwm, min_pwm_, max_pwm_);
    }
    channels[static_cast<size_t>(channel - 1)] = static_cast<uint16_t>(pwm);
  }

  void publish_channels(const std::array<uint16_t, 18> & channels)
  {
    mavros_msgs::msg::OverrideRCIn msg;
    if (dry_run_) {
      msg.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_RELEASE);
    } else {
      msg.channels = channels;
    }
    rc_pub_->publish(msg);
  }

  std::string bbox_topic_;
  std::string depth_topic_;
  std::string depth_pose_topic_;
  double depth_pose_scale_{1.0};
  double depth_pose_offset_m_{0.0};
  std::string depth_odom_topic_;
  double depth_odom_scale_{-1.0};
  double depth_odom_offset_m_{0.0};
  std::string depth_source_;
  std::string enable_topic_;
  std::string state_topic_;
  std::string status_topic_;
  std::string vehicle_state_topic_;
  std::string rc_override_topic_;
  bool dry_run_{false};
  bool require_armed_{true};
  double vehicle_state_timeout_sec_{8.0};
  double control_rate_hz_{20.0};
  double detection_timeout_sec_{0.7};
  double depth_timeout_sec_{1.0};
  double work_depth_m_{9.5};
  double surface_depth_m_{0.4};
  double max_depth_m_{10.5};
  double depth_tolerance_m_{0.2};
  double depth_stable_sec_{2.0};
  double depth_kp_pwm_per_m_{120.0};
  int max_depth_delta_pwm_{160};
  int buoy_class_id_{0};
  int stick_class_id_{1};
  int min_detection_hits_{3};
  double approach_area_ratio_{0.12};
  double search_timeout_sec_{20.0};
  double area_verify_sec_{12.0};
  double fork_target_x_{0.5};
  double fork_target_y_{0.5};
  double stick_deadband_x_{0.06};
  double stick_deadband_y_{0.08};
  double align_stable_sec_{0.7};
  int insert_pwm_{1560};
  double insert_duration_sec_{0.8};
  int detach_pwm_{1620};
  double detach_duration_sec_{0.3};
  int backoff_pwm_{1420};
  double backoff_duration_sec_{0.5};
  double verify_clear_sec_{1.0};
  double verify_timeout_sec_{3.0};
  int max_target_retries_{2};
  int expected_target_count_{0};
  int throttle_channel_{3};
  int yaw_channel_{4};
  int forward_channel_{5};
  int neutral_pwm_{1500};
  int min_pwm_{1300};
  int max_pwm_{1700};
  int max_yaw_delta_{180};
  int max_tracking_depth_delta_{100};
  int approach_forward_pwm_{1560};
  int search_yaw_pwm_{1560};
  bool yaw_invert_{false};
  bool vertical_positive_is_up_{true};

  bool enabled_{false};
  bool vehicle_state_seen_{false};
  State state_{State::IDLE};
  rclcpp::Time state_entered_at_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> condition_started_at_;
  std::optional<double> depth_m_;
  rclcpp::Time depth_received_at_{0, 0, RCL_ROS_TIME};
  std::optional<Detection> buoy_;
  std::optional<Detection> stick_;
  int target_retries_{0};
  int target_success_count_{0};
  mavros_msgs::msg::State vehicle_state_{};
  rclcpp::Time vehicle_state_received_at_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr depth_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr vehicle_state_sub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionStateMachineNode>();
  rclcpp::spin(node);
  node->publish_release_once();
  rclcpp::shutdown();
  return 0;
}
