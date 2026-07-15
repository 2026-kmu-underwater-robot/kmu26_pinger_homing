#pragma once

#include <algorithm>
#include <cmath>

namespace kmu26_mission_fsm::vision_buoy {

struct GuidanceCommand {
  double forward{0.0};
  double heave{0.0};
  double yaw{0.0};
};

inline double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

inline GuidanceCommand alignment_command(
    double filtered_error_x,
    double filtered_error_y,
    double filtered_error_rate_x,
    double requested_forward,
    double target_error_x,
    double target_error_y,
    double center_tolerance,
    double yaw_kp,
    double yaw_kd,
    double heave_kp,
    double max_yaw,
    double max_heave) {
  GuidanceCommand command;
  const double ex = filtered_error_x - target_error_x;
  const double ey = filtered_error_y - target_error_y;
  const bool centered = std::abs(ex) <= center_tolerance;
  const double alignment_scale = clamp(1.0 - std::abs(ex), 0.15, 1.0);
  command.forward = centered
      ? requested_forward
      : std::min(requested_forward * alignment_scale, 0.14);
  command.yaw = clamp(
      -(yaw_kp * ex + yaw_kd * filtered_error_rate_x), -max_yaw, max_yaw);
  command.heave = clamp(heave_kp * ey, -max_heave, max_heave);
  return command;
}

inline GuidanceCommand surface_command(
    double filtered_error_x,
    double filtered_error_rate_x,
    double requested_heave,
    double yaw_kp,
    double yaw_kd,
    double yaw_limit,
    double center_tolerance,
    double forward,
    double turn_forward) {
  GuidanceCommand command;
  command.heave = requested_heave;
  command.yaw = clamp(
      -(yaw_kp * filtered_error_x + yaw_kd * filtered_error_rate_x),
      -yaw_limit,
      yaw_limit);
  command.forward = std::abs(filtered_error_x) <= center_tolerance
      ? forward : turn_forward;
  return command;
}

}  // namespace kmu26_mission_fsm::vision_buoy
