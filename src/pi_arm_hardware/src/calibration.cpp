#include "pi_arm_hardware/calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace pi_arm_hardware
{
namespace
{

constexpr std::array<const char *, kCalibrationAxisCount> kExpectedJointNames{
  "joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"};

void hash_bytes(std::uint64_t & hash, const std::string & bytes)
{
  constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
  for (const unsigned char value : bytes) {
    hash ^= static_cast<std::uint64_t>(value);
    hash *= kFnvPrime;
  }
}

std::string read_file(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot read file: " + path);
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

template<typename T, std::size_t N>
std::array<T, N> required_array(const YAML::Node & node, const char * key)
{
  const auto value = node[key];
  if (!value || !value.IsSequence() || value.size() != N) {
    throw std::invalid_argument(std::string("calibration.") + key + " must contain six values");
  }
  std::array<T, N> result{};
  for (std::size_t index = 0; index < N; ++index) {
    result[index] = value[index].as<T>();
  }
  return result;
}

void reject_unknown_fields(const YAML::Node & node)
{
  static const std::set<std::string> allowed{
    "schema_version", "robot_id", "calibration_id", "approved", "model_version",
    "created_at_utc", "configuration_fingerprint", "joint_names", "motor_ids",
    "packing_pose_joint_rad", "raw_output_at_packing_rad"};
  for (const auto & item : node) {
    const auto key = item.first.as<std::string>();
    if (allowed.count(key) == 0U) {
      throw std::invalid_argument("unknown calibration field: " + key);
    }
  }
}

CalibrationResult invalid_result(const std::string & message)
{
  CalibrationResult result;
  result.software_output_offsets_rad.fill(std::numeric_limits<double>::quiet_NaN());
  result.message = message;
  return result;
}

}  // namespace

std::string calibration_configuration_fingerprint(
  const std::string & cable_kinematics_file,
  const std::vector<ActuatorConfig> & actuators)
{
  std::uint64_t hash = 14695981039346656037ULL;
  hash_bytes(hash, read_file(cable_kinematics_file));
  std::ostringstream mapping;
  mapping << std::setprecision(17);
  for (const auto & actuator : actuators) {
    mapping << actuator.joint_name << '|' << static_cast<unsigned>(actuator.motor_id) << '|'
            << actuator.protocol_angle_ratio << '|' << actuator.motor_direction << '\n';
  }
  hash_bytes(hash, mapping.str());
  std::ostringstream output;
  output << "fnv1a64:" << std::hex << std::setw(16) << std::setfill('0') << hash;
  return output.str();
}

CalibrationResult load_pack_reference_calibration(
  const std::string & path,
  const std::string & cable_kinematics_file,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics) noexcept
{
  if (path.empty()) {
    return invalid_result("calibration_file is empty");
  }
  try {
    const YAML::Node document = YAML::LoadFile(path);
    const YAML::Node node = document["calibration"];
    if (!node || !node.IsMap()) {
      throw std::invalid_argument("top-level calibration map is required");
    }
    reject_unknown_fields(node);

    CalibrationResult result;
    result.data.schema_version = node["schema_version"].as<int>();
    result.data.robot_id = node["robot_id"].as<std::string>();
    result.data.calibration_id = node["calibration_id"].as<std::string>();
    result.data.approved = node["approved"].as<bool>();
    result.data.model_version = node["model_version"].as<int>();
    result.data.created_at_utc = node["created_at_utc"].as<std::string>();
    result.data.configuration_fingerprint =
      node["configuration_fingerprint"].as<std::string>();
    result.data.joint_names =
      required_array<std::string, kCalibrationAxisCount>(node, "joint_names");
    const auto motor_ids = required_array<int, kCalibrationAxisCount>(node, "motor_ids");
    result.data.packing_pose_joint_rad =
      required_array<double, kCalibrationAxisCount>(node, "packing_pose_joint_rad");
    result.data.raw_output_at_packing_rad =
      required_array<double, kCalibrationAxisCount>(node, "raw_output_at_packing_rad");

    if (result.data.schema_version != 1) {
      throw std::invalid_argument("calibration schema_version must be 1");
    }
    if (result.data.model_version != 3) {
      throw std::invalid_argument("calibration model_version must be 3");
    }
    if (result.data.robot_id.empty() || result.data.calibration_id.empty() ||
      result.data.created_at_utc.empty())
    {
      throw std::invalid_argument("robot_id, calibration_id and created_at_utc must not be empty");
    }
    if (actuators.size() != kCalibrationAxisCount) {
      throw std::invalid_argument("calibration requires six actuator mappings");
    }

    for (std::size_t index = 0; index < kCalibrationAxisCount; ++index) {
      if (result.data.joint_names[index] != kExpectedJointNames[index] ||
        actuators[index].joint_name != kExpectedJointNames[index])
      {
        throw std::invalid_argument("calibration joint order does not match the six-axis model");
      }
      if (motor_ids[index] <= 0 || motor_ids[index] > 255 ||
        actuators[index].motor_id != static_cast<std::uint8_t>(motor_ids[index]))
      {
        throw std::invalid_argument("calibration motor_ids do not match the actuator mapping");
      }
      result.data.motor_ids[index] = static_cast<std::uint8_t>(motor_ids[index]);
      if (!std::isfinite(result.data.packing_pose_joint_rad[index]) ||
        !std::isfinite(result.data.raw_output_at_packing_rad[index]))
      {
        throw std::invalid_argument("calibration pose and raw output values must be finite");
      }
    }

    const auto expected_fingerprint =
      calibration_configuration_fingerprint(cable_kinematics_file, actuators);
    if (result.data.configuration_fingerprint != expected_fingerprint) {
      throw std::invalid_argument(
              "calibration configuration_fingerprint mismatch: expected " +
              expected_fingerprint);
    }

    pi_arm_transmission::Vector6 packing_pose{};
    for (std::size_t index = 0; index < kCalibrationAxisCount; ++index) {
      packing_pose[index] = result.data.packing_pose_joint_rad[index];
    }
    pi_arm_transmission::Vector6 model_output_at_packing{};
    const auto status =
      kinematics.joint_to_output_position(packing_pose, model_output_at_packing);
    if (!status) {
      throw std::invalid_argument(
              std::string("packing pose is invalid for cable model: ") +
              pi_arm_transmission::to_string(status.error));
    }
    const auto limit_status = kinematics.check_output_position_limits(model_output_at_packing);
    if (!limit_status) {
      throw std::invalid_argument("packing pose exceeds model output limits");
    }
    for (std::size_t index = 0; index < kCalibrationAxisCount; ++index) {
      result.software_output_offsets_rad[index] =
        model_output_at_packing[index] - result.data.raw_output_at_packing_rad[index];
    }
    if (!result.data.approved) {
      result.message = "calibration exists but approved is false";
      return result;
    }
    result.valid = true;
    result.message = "approved pack-reference calibration loaded";
    return result;
  } catch (const std::exception & error) {
    return invalid_result(error.what());
  }
}

double rom_relative_to_model_output_rad(
  const double raw_output_rad, const double software_output_offset_rad) noexcept
{
  return raw_output_rad + software_output_offset_rad;
}

double model_to_rom_relative_output_rad(
  const double model_output_rad, const double software_output_offset_rad) noexcept
{
  return model_output_rad - software_output_offset_rad;
}

}  // namespace pi_arm_hardware