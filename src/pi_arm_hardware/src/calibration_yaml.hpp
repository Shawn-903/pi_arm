#pragma once

#include "pi_arm_hardware/calibration.hpp"

#include <yaml-cpp/yaml.h>

namespace pi_arm_hardware
{
CalibrationResult decode_pack_reference(
  const YAML::Node & document, const std::string & expected_fingerprint,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics);
CalibrationResult decode_software_zero(
  const YAML::Node & document, const CalibrationResult & base,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics);
}
