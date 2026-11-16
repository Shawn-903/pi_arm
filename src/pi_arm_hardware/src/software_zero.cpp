#include "pi_arm_hardware/software_zero.hpp"

#include <yaml-cpp/yaml.h>

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <set>
#include <stdexcept>

namespace pi_arm_hardware
{
namespace
{
CalibrationVector6 vector6(const YAML::Node & node, const char * key)
{
  const auto values = node[key];
  if (!values.IsSequence() || values.size() != 6) {
    throw std::invalid_argument(std::string(key) + " must contain six values");
  }
  CalibrationVector6 result{};
  for (std::size_t i = 0; i < 6; ++i) {
    result[i] = values[i].as<double>();
    if (!std::isfinite(result[i])) {
      throw std::invalid_argument(std::string(key) + " must be finite");
    }
  }
  return result;
}

CalibrationResult decode(
  const YAML::Node & document, const CalibrationResult & base,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics)
{
  if (!base.valid || !base.data.approved || actuators.size() != 6) {
    throw std::invalid_argument("software zero requires an approved base calibration");
  }
  const auto node = document["software_zero"];
  if (!node.IsMap()) {throw std::invalid_argument("software_zero map is required");}
  const std::set<std::string> fields{
    "schema_version", "reference_id", "created_at_utc", "robot_id", "base_calibration_id",
    "configuration_fingerprint", "motor_ids", "reference_joint_positions_rad",
    "raw_protocol_positions_deg"};
  for (const auto & item : node) {
    if (fields.count(item.first.as<std::string>()) == 0) {
      throw std::invalid_argument("unknown software_zero field: " + item.first.as<std::string>());
    }
  }
  if (node["schema_version"].as<int>() != 1 ||
    node["robot_id"].as<std::string>() != base.data.robot_id ||
    node["base_calibration_id"].as<std::string>() != base.data.calibration_id ||
    node["configuration_fingerprint"].as<std::string>() != base.data.configuration_fingerprint)
  {
    throw std::invalid_argument("software zero does not match the base calibration/configuration");
  }
  const auto ids = node["motor_ids"];
  if (!ids.IsSequence() || ids.size() != 6) {
    throw std::invalid_argument("software zero requires six motor IDs");
  }
  for (std::size_t i = 0; i < 6; ++i) {
    if (ids[i].as<int>() != actuators[i].motor_id) {
      throw std::invalid_argument("software zero motor order mismatch");
    }
  }
  auto result = base;
  result.software_zero_id = node["reference_id"].as<std::string>();
  if (result.software_zero_id.empty() || node["created_at_utc"].as<std::string>().empty()) {
    throw std::invalid_argument("software zero ID and timestamp must not be empty");
  }
  const auto reference = vector6(node, "reference_joint_positions_rad");
  const auto protocol = vector6(node, "raw_protocol_positions_deg");
  pi_arm_transmission::Vector6 output{};
  if (!kinematics.joint_to_output_position(reference, output) ||
    !kinematics.check_output_position_limits(output))
  {
    throw std::invalid_argument("software zero reference pose is outside the model limits");
  }
  for (std::size_t i = 0; i < 6; ++i) {
    result.software_output_offsets_rad[i] =
      output[i] - protocol_position_to_output_rad(protocol[i], actuators[i]);
  }
  result.message = "approved base calibration with host-side software zero";
  return result;
}

void atomic_write(const std::filesystem::path & path, const std::string & text)
{
  if (!path.is_absolute() || path.filename().empty()) {
    throw std::invalid_argument("software_zero_file must be an absolute file path");
  }
  std::filesystem::create_directories(path.parent_path());
  std::string temporary = path.string() + ".tmp-XXXXXX";
  int fd = ::mkstemp(temporary.data());
  if (fd < 0) {throw std::runtime_error("cannot create software zero temporary file");}
  try {
    std::size_t written = 0;
    while (written < text.size()) {
      const auto count = ::write(fd, text.data() + written, text.size() - written);
      if (count < 0 && errno == EINTR) {continue;}
      if (count <= 0) {throw std::runtime_error("cannot write software zero file");}
      written += static_cast<std::size_t>(count);
    }
    if (::fsync(fd) != 0) {throw std::runtime_error("cannot sync software zero file");}
    const int close_result = ::close(fd);
    fd = -1;
    if (close_result != 0) {throw std::runtime_error("cannot close software zero file");}
    std::filesystem::rename(temporary, path);
  } catch (...) {
    if (fd >= 0) {::close(fd);}
    ::unlink(temporary.c_str());
    throw;
  }
  // The rename is the commit point. Do not report it as rolled back afterwards.
  const int directory = ::open(path.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
  if (directory >= 0) {::fsync(directory); ::close(directory);}
}
}  // namespace

CalibrationResult load_software_zero_reference(
  const std::string & path, const CalibrationResult & base,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics)
{
  return decode(YAML::LoadFile(path), base, actuators, kinematics);
}

CalibrationResult save_software_zero_reference(
  const std::string & path, const CalibrationResult & base,
  const CalibrationVector6 & reference_joint_rad, const CalibrationVector6 & protocol_deg,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics,
  const std::string & reference_id, const std::string & created_at_utc)
{
  // Refuse to overwrite unrelated files or another robot's accepted reference.
  if (std::filesystem::exists(path)) {
    load_software_zero_reference(path, base, actuators, kinematics);
  }
  YAML::Node document;
  auto node = document["software_zero"];
  node["schema_version"] = 1;
  node["reference_id"] = reference_id;
  node["created_at_utc"] = created_at_utc;
  node["robot_id"] = base.data.robot_id;
  node["base_calibration_id"] = base.data.calibration_id;
  node["configuration_fingerprint"] = base.data.configuration_fingerprint;
  for (std::size_t i = 0; i < 6; ++i) {
    node["motor_ids"].push_back(static_cast<int>(base.data.motor_ids[i]));
    node["reference_joint_positions_rad"].push_back(reference_joint_rad[i]);
    node["raw_protocol_positions_deg"].push_back(protocol_deg[i]);
  }
  auto result = decode(document, base, actuators, kinematics);
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  emitter << document;
  if (!emitter.good()) {throw std::runtime_error("cannot serialize software zero reference");}
  atomic_write(path, std::string(emitter.c_str()) + "\n");
  return result;
}
}  // namespace pi_arm_hardware
