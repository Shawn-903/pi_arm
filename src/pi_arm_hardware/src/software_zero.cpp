#include "pi_arm_hardware/software_zero.hpp"
#include "calibration_yaml.hpp"

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
YAML::Node text_field(const std::string & value)
{
  YAML::Node node(value);
  // Preserve timestamps and numeric-looking IDs as strings in other YAML readers.
  node.SetTag("tag:yaml.org,2002:str");
  return node;
}

YAML::Node pack_reference(const CalibrationData & data)
{
  YAML::Node node;
  node["schema_version"] = data.schema_version;
  node["robot_id"] = text_field(data.robot_id);
  node["calibration_id"] = text_field(data.calibration_id);
  node["approved"] = data.approved;
  node["model_version"] = data.model_version;
  node["created_at_utc"] = text_field(data.created_at_utc);
  node["configuration_fingerprint"] = data.configuration_fingerprint;
  for (std::size_t i = 0; i < kCalibrationAxisCount; ++i) {
    node["joint_names"].push_back(data.joint_names[i]);
    node["motor_ids"].push_back(static_cast<unsigned>(data.motor_ids[i]));
    node["packing_pose_joint_rad"].push_back(data.packing_pose_joint_rad[i]);
    node["raw_output_at_packing_rad"].push_back(data.raw_output_at_packing_rad[i]);
  }
  return node;
}

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

}  // namespace

CalibrationResult decode_software_zero(
  const YAML::Node & document, const CalibrationResult & base,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics)
{
  const auto node = document["software_zero"];
  if (!node.IsMap()) {throw std::invalid_argument("software_zero map is required");}
  const int version = node["schema_version"].as<int>();
  if (version < 1 || version > 4 || actuators.size() != kCalibrationAxisCount) {
    throw std::invalid_argument("invalid software zero format or actuator mapping");
  }
  std::set<std::string> fields{
    "schema_version", "created_at_utc", "motor_ids", "reference_joint_positions_rad",
    "raw_protocol_positions_deg"};
  if (version <= 3) {fields.insert({"reference_id", "configuration_fingerprint"});}
  if (version >= 3) {
    fields.insert("joint_names");
  } else {
    fields.insert({"robot_id", "base_calibration_id"});
  }
  for (const auto & item : node) {
    if (fields.count(item.first.as<std::string>()) == 0) {
      throw std::invalid_argument("unknown software_zero field: " + item.first.as<std::string>());
    }
  }
  if (version <= 3 && (base.data.configuration_fingerprint.empty() ||
    node["configuration_fingerprint"].as<std::string>() != base.data.configuration_fingerprint))
  {
    throw std::invalid_argument("software zero does not match the actuator/cable model");
  }
  if (version < 3 && (!base.valid || !base.data.approved ||
    node["robot_id"].as<std::string>() != base.data.robot_id ||
    node["base_calibration_id"].as<std::string>() != base.data.calibration_id))
  {
    throw std::invalid_argument("legacy software zero does not match the base calibration");
  }
  auto result = base;
  if (version >= 3 && !result.data.approved) {
    result.data.calibration_id = base.data.configuration_fingerprint;
  }
  if (version == 2 || document["calibration"]) {
    const auto embedded = decode_pack_reference(
      document, base.data.configuration_fingerprint, actuators, kinematics);
    if (!embedded.valid) {
      throw std::invalid_argument("invalid calibration in zero file: " + embedded.message);
    }
    if (base.data.approved && (embedded.data.robot_id != base.data.robot_id ||
      embedded.data.calibration_id != base.data.calibration_id ||
      embedded.data.created_at_utc != base.data.created_at_utc ||
      embedded.data.packing_pose_joint_rad != base.data.packing_pose_joint_rad ||
      embedded.data.raw_output_at_packing_rad != base.data.raw_output_at_packing_rad))
    {
      throw std::invalid_argument("zero file and selected base calibration differ");
    }
    result = embedded;
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
  if (version >= 3) {
    const auto names = node["joint_names"];
    if (!names.IsSequence() || names.size() != actuators.size()) {
      throw std::invalid_argument("software zero requires six joint names");
    }
    for (std::size_t i = 0; i < actuators.size(); ++i) {
      if (names[i].as<std::string>() != actuators[i].joint_name) {
        throw std::invalid_argument("software zero joint order mismatch");
      }
    }
  }
  if (node["created_at_utc"].as<std::string>().empty()) {
    throw std::invalid_argument("software zero timestamp must not be empty");
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
  result.valid = true;
  result.message = "software zero loaded";
  return result;
}

namespace
{
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
  if (!base.valid) {throw std::invalid_argument("selected calibration is invalid: " + base.message);}
  return decode_software_zero(YAML::LoadFile(path), base, actuators, kinematics);
}

CalibrationResult save_software_zero_reference(
  const std::string & path, const CalibrationResult & base,
  const CalibrationVector6 & reference_joint_rad, const CalibrationVector6 & protocol_deg,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics,
  const std::string & created_at_utc)
{
  if (!base.valid || !base.data.approved) {
    throw std::invalid_argument("selected packing calibration is not approved");
  }
  return capture_pose_zero(path, base.data.configuration_fingerprint, reference_joint_rad,
    protocol_deg, actuators, kinematics, created_at_utc, &base);
}

CalibrationResult capture_pose_zero(
  const std::string & path, const std::string & model_fingerprint,
  const CalibrationVector6 & reference_joint_rad, const CalibrationVector6 & protocol_deg,
  const std::vector<ActuatorConfig> & actuators,
  const pi_arm_transmission::CableKinematics & kinematics,
  const std::string & created_at_utc,
  const CalibrationResult * packing)
{
  if (actuators.size() != kCalibrationAxisCount || model_fingerprint.empty()) {
    throw std::invalid_argument("software zero requires the six configured actuators and cable model");
  }
  CalibrationResult model;
  model.data.configuration_fingerprint = model_fingerprint;
  YAML::Node document;
  if (packing) {
    if (!packing->valid || !packing->data.approved ||
      packing->data.configuration_fingerprint != model_fingerprint)
    {throw std::invalid_argument("packing preset does not match the current cable model");}
    document["calibration"] = pack_reference(packing->data);
    model = *packing;
  }
  if (std::filesystem::exists(path)) {
    const auto previous = YAML::LoadFile(path);
    const auto old = previous["software_zero"];
    if (!old.IsMap()) {
      throw std::invalid_argument("destination is not a software zero file");
    }
  }
  auto node = document["software_zero"];
  node["schema_version"] = 4;
  node["created_at_utc"] = text_field(created_at_utc);
  for (std::size_t i = 0; i < 6; ++i) {
    node["motor_ids"].push_back(static_cast<unsigned>(actuators[i].motor_id));
    node["joint_names"].push_back(actuators[i].joint_name);
    node["reference_joint_positions_rad"].push_back(reference_joint_rad[i]);
    node["raw_protocol_positions_deg"].push_back(protocol_deg[i]);
  }
  auto result = decode_software_zero(document, model, actuators, kinematics);
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  emitter << document;
  if (!emitter.good()) {throw std::runtime_error("cannot serialize software zero reference");}
  atomic_write(path, std::string(emitter.c_str()) + "\n");
  return result;
}
}  // namespace pi_arm_hardware
