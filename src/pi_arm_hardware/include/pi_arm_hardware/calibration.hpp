#pragma once

#include "pi_arm_hardware/actuator_coordinates.hpp"
#include "pi_arm_transmission/cable_kinematics.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace pi_arm_hardware
{

constexpr std::size_t kCalibrationAxisCount = 6U;
using CalibrationVector6 = std::array<double, kCalibrationAxisCount>;

struct CalibrationData
{
  int schema_version{0};
  std::string robot_id;
  std::string calibration_id;
  bool approved{false};
  int model_version{0};
  std::string created_at_utc;
  std::string configuration_fingerprint;
  std::array<std::string, kCalibrationAxisCount> joint_names{};
  std::array<std::uint8_t, kCalibrationAxisCount> motor_ids{};
  CalibrationVector6 packing_pose_joint_rad{};
  CalibrationVector6 raw_output_at_packing_rad{};
};

struct CalibrationResult
{
  CalibrationData data;
  CalibrationVector6 software_output_offsets_rad{};
  bool valid{false};
  std::string message;
};

std::string calibration_configuration_fingerprint(
  const std::string & cable_kinematics_file,
  const std::vector<ActuatorConfig> & actuators);

CalibrationResult load_pack_reference_calibration(
  const std::string & path,
  const std::string & cable_kinematics_file,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics) noexcept;

double rom_relative_to_model_output_rad(
  double raw_output_rad, double software_output_offset_rad) noexcept;
double model_to_rom_relative_output_rad(
  double model_output_rad, double software_output_offset_rad) noexcept;

}  // namespace pi_arm_hardware