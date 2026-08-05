#include "pi_arm_hardware/actuator_coordinates.hpp"

#include <cmath>
#include <stdexcept>

namespace pi_arm_hardware
{
namespace
{

constexpr double kDegreesPerRadian = 57.2957795130823208768;

}  // namespace

void validate_actuator_config(const ActuatorConfig & config)
{
  if (config.joint_name.empty()) {
    throw std::invalid_argument("actuator joint_name must not be empty");
  }
  if (config.motor_id == 0U) {
    throw std::invalid_argument("actuator motor_id must be positive");
  }
  if (!std::isfinite(config.protocol_angle_ratio) || config.protocol_angle_ratio <= 0.0) {
    throw std::invalid_argument("protocol_angle_ratio must be finite and positive");
  }
  if (config.motor_direction != -1 && config.motor_direction != 1) {
    throw std::invalid_argument("motor_direction must be -1 or 1");
  }
}

double output_position_to_protocol_deg(
  const double output_position_rad, const ActuatorConfig & config) noexcept
{
  return output_position_rad * kDegreesPerRadian *
         static_cast<double>(config.motor_direction) * config.protocol_angle_ratio;
}

double protocol_position_to_output_rad(
  const double protocol_position_deg, const ActuatorConfig & config) noexcept
{
  return protocol_position_deg /
         (kDegreesPerRadian * static_cast<double>(config.motor_direction) *
         config.protocol_angle_ratio);
}

double output_velocity_to_protocol_dps(
  const double output_velocity_rad_s, const ActuatorConfig & config) noexcept
{
  return output_position_to_protocol_deg(output_velocity_rad_s, config);
}

double protocol_velocity_to_output_rad_s(
  const double protocol_velocity_dps, const ActuatorConfig & config) noexcept
{
  return protocol_position_to_output_rad(protocol_velocity_dps, config);
}

}  // namespace pi_arm_hardware
