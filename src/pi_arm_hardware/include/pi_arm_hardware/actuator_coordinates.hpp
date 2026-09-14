#pragma once

#include <cstdint>
#include <string>

namespace pi_arm_hardware
{

struct ActuatorConfig
{
  std::string joint_name;
  std::uint8_t motor_id{0U};
  double protocol_angle_ratio{0.0};
  int motor_direction{1};
};

void validate_actuator_config(const ActuatorConfig & config);

double output_position_to_protocol_deg(
  double output_position_rad, const ActuatorConfig & config) noexcept;
double protocol_position_to_output_rad(
  double protocol_position_deg, const ActuatorConfig & config) noexcept;
double model_output_position_to_protocol_deg(
  double model_output_position_rad, double software_output_offset_rad,
  const ActuatorConfig & config) noexcept;
double protocol_position_to_model_output_rad(
  double protocol_position_deg, double software_output_offset_rad,
  const ActuatorConfig & config) noexcept;
double output_velocity_to_protocol_dps(
  double output_velocity_rad_s, const ActuatorConfig & config) noexcept;
double protocol_velocity_to_output_rad_s(
  double protocol_velocity_dps, const ActuatorConfig & config) noexcept;

}  // namespace pi_arm_hardware
