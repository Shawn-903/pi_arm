#include "pi_arm_hardware/hardware_state_message.hpp"

#include "pi_arm_hardware/backend.hpp"

namespace pi_arm_hardware
{

pi_arm_interfaces::msg::HardwareState make_hardware_state_message(
  const HardwareSample & sample, const std::vector<std::string> & joint_names,
  const BackendConfig & config, const builtin_interfaces::msg::Time & stamp)
{
  pi_arm_interfaces::msg::HardwareState message;
  message.stamp = stamp;
  message.connected = sample.connected;
  message.driver_mode = sample.driver_mode;
  message.management_operation = sample.management_operation;
  message.joint_names = joint_names;
  message.positions = sample.positions_rad;
  message.velocities = sample.velocities_rad_s;
  message.enabled = sample.enabled;
  message.errors = sample.has_error;
  message.moving = sample.in_motion;
  message.fresh = sample.fresh;
  message.error_codes = sample.error_codes;
  message.ages_sec = sample.ages_sec;
  for (const auto & actuator : config.actuators) {
    message.motor_ids.push_back(actuator.motor_id);
    message.protocol_angle_ratios.push_back(actuator.protocol_angle_ratio);
    message.motor_directions.push_back(actuator.motor_direction);
  }
  message.motor_enabled = sample.motor_enabled;
  message.motor_errors = sample.motor_has_error;
  message.motor_moving = sample.motor_in_motion;
  message.motor_fresh = sample.motor_fresh;
  message.motor_error_codes = sample.motor_error_codes;
  message.motor_ages_sec = sample.motor_ages_sec;
  message.calibrated = sample.calibrated;
  message.kinematics_valid = sample.kinematics_valid;
  message.teaching_active = sample.teaching_active;
  message.control_permitted = sample.control_permitted;
  message.motion_stop_pending = sample.motion_stop_pending;
  message.motion_stop_failed = sample.motion_stop_failed;
  message.calibration_id = sample.calibration_id;
  message.software_zero_id = sample.software_zero_id;
  message.raw_protocol_positions_deg = sample.raw_protocol_positions_deg;
  message.raw_output_positions_rad = sample.raw_output_positions_rad;
  message.software_output_offsets_rad = sample.software_output_offsets_rad;
  message.model_output_positions_rad = sample.model_output_positions_rad;
  message.kinematics_error_code = sample.kinematics_error_code;
  message.kinematics_error_axis = sample.kinematics_error_axis;
  message.kinematics_error_value = sample.kinematics_error_value;
  message.kinematics_error_message = sample.kinematics_error_message;
  message.diagnostic = sample.error_message;
  message.commissioning = config.commissioning;
  message.can_interface = config.can_interface;
  message.motor_test_permitted = sample.motor_test_permitted;
  message.joint_test_permitted = sample.joint_test_permitted;
  return message;
}

}  // namespace pi_arm_hardware
