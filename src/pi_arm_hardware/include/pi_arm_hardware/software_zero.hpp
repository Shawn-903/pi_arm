#pragma once

#include "pi_arm_hardware/calibration.hpp"

namespace pi_arm_hardware
{

// Throws on an absent, malformed or mismatched reference. No fallback to old offsets.
CalibrationResult load_software_zero_reference(
  const std::string & path, const CalibrationResult & base,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics);

// Validates before atomically replacing the host-side file. Never writes CAN/ROM.
CalibrationResult save_software_zero_reference(
  const std::string & path, const CalibrationResult & base,
  const CalibrationVector6 & reference_joint_rad, const CalibrationVector6 & protocol_deg,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics,
  const std::string & created_at_utc);

CalibrationResult capture_pose_zero(
  const std::string & path, const std::string & model_fingerprint,
  const CalibrationVector6 & reference_joint_rad, const CalibrationVector6 & protocol_deg,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics,
  const std::string & created_at_utc,
  const CalibrationResult * packing = nullptr);

}  // namespace pi_arm_hardware
