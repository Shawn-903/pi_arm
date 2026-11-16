#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include "pi_arm_hardware/backend.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"

// Private policies shared by the real and deterministic mock backends.
namespace pi_arm_hardware::detail
{

inline constexpr std::size_t kAxisCount = 6U;
inline constexpr int64_t kWriteRejectLogPeriodMs = 1000;
inline constexpr std::array<const char *, kAxisCount> kExpectedJointNames{
  "joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"};

inline rclcpp::Logger backend_logger()
{
  return rclcpp::get_logger("PiArmBackend");
}

inline rclcpp::Clock & steady_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

template<typename ... Args>
void log_write_reject(const char * format, Args && ... args) noexcept
{
  try {
    RCLCPP_ERROR_THROTTLE(
      backend_logger(), steady_clock(), kWriteRejectLogPeriodMs,
      format, std::forward<Args>(args)...);
  } catch (...) {
  }
}

template<typename ... Args>
void log_write_warn(const char * format, Args && ... args) noexcept
{
  try {
    RCLCPP_WARN_THROTTLE(
      backend_logger(), steady_clock(), kWriteRejectLogPeriodMs,
      format, std::forward<Args>(args)...);
  } catch (...) {
  }
}

inline double clamp_joint_velocity(
  const double velocity, const double velocity_limit, const std::size_t index) noexcept
{
  if (!(std::abs(velocity) > velocity_limit + 1e-6)) {
    return velocity;
  }
  log_write_warn(
    "write velocity clamped at joint[%zu]: vel=%.17g -> lim=%.17g",
    index, velocity, velocity_limit);
  return std::copysign(velocity_limit, velocity);
}

inline void validate_backend_config(const BackendConfig & config)
{
  if (!std::isfinite(config.moving_speed_threshold_rad_s) ||
    config.moving_speed_threshold_rad_s < 0.0)
  {
    throw std::invalid_argument("moving speed threshold must be finite and nonnegative");
  }
  if (config.actuators.size() != kAxisCount) {
    throw std::invalid_argument("backend requires exactly six actuators");
  }
  if (config.cable_kinematics_file.empty()) {
    throw std::invalid_argument("cable_kinematics_file must not be empty");
  }
  if (config.joint_lower_limits_rad.size() != kAxisCount ||
    config.joint_upper_limits_rad.size() != kAxisCount ||
    config.joint_velocity_limits_rad_s.size() != kAxisCount)
  {
    throw std::invalid_argument("backend requires six joint position and velocity limits");
  }

  std::unordered_set<std::uint8_t> motor_ids;
  for (std::size_t index = 0; index < config.actuators.size(); ++index) {
    validate_actuator_config(config.actuators[index]);
    if (config.actuators[index].joint_name != kExpectedJointNames[index]) {
      throw std::invalid_argument(
              "actuator order must be joint_01, joint_02, joint_04, joint_05, joint_06, joint_09");
    }
    if (!motor_ids.insert(config.actuators[index].motor_id).second) {
      throw std::invalid_argument("actuator motor IDs must be unique");
    }
    if (!std::isfinite(config.joint_lower_limits_rad[index]) ||
      !std::isfinite(config.joint_feedback_tolerances_rad[index]) ||
      config.joint_feedback_tolerances_rad[index] < 0.0 ||
      !std::isfinite(config.joint_upper_limits_rad[index]) ||
      config.joint_lower_limits_rad[index] > config.joint_upper_limits_rad[index] ||
      !std::isfinite(config.joint_velocity_limits_rad_s[index]) ||
      config.joint_velocity_limits_rad_s[index] <= 0.0)
    {
      throw std::invalid_argument("backend joint limits must be finite and valid");
    }
  }
}

inline bool joint_depends_on_motor(
  const std::size_t joint_index, const std::size_t motor_index, const bool swap_wrist_channels) noexcept
{
  if (joint_index == 0U) {
    return motor_index == 0U;
  }
  if (joint_index == 1U) {
    return motor_index == 1U;
  }
  const auto own_motor = swap_wrist_channels && (joint_index == 3U || joint_index == 4U) ?
    7U - joint_index : joint_index;
  return motor_index == 1U || motor_index == own_motor;
}

template<typename T>
std::vector<double> to_vector(const T & values)
{
  return std::vector<double>(values.begin(), values.end());
}

}  // namespace pi_arm_hardware::detail
