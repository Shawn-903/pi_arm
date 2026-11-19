#include "pi_arm_hardware/backend.hpp"

#include "hardware_interface/component_parser.hpp"

#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace pi_arm_hardware
{
namespace
{

constexpr std::size_t kAxisCount = 6U;

template<typename T>
T parse_value(const std::string & text, const std::string & name)
{
  std::istringstream stream(text);
  T value{};
  stream >> value;
  if (!stream || !(stream >> std::ws).eof()) {
    throw std::invalid_argument("invalid value for '" + name + "': " + text);
  }
  return value;
}

std::string parameter_or(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const std::string & fallback)
{
  const auto found = parameters.find(name);
  return found == parameters.end() ? fallback : found->second;
}

const std::string & required_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & owner, const std::string & name)
{
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    throw std::invalid_argument(owner + " is missing required parameter '" + name + "'");
  }
  return found->second;
}

}  // namespace

BackendConfig backend_config_from_robot_description(const std::string & robot_description)
{
  const auto resources = hardware_interface::parse_control_resources_from_urdf(robot_description);
  if (resources.size() != 1U) {
    throw std::invalid_argument("robot_description must contain exactly one ros2_control system");
  }
  const auto & info = resources.front();
  if (info.joints.size() != kAxisCount) {
    throw std::invalid_argument("monitor requires exactly six active ros2_control joints");
  }

  BackendConfig config;
  config.can_interface = parameter_or(info.hardware_parameters, "can_interface", "can0");
  config.cable_kinematics_file =
    required_parameter(info.hardware_parameters, "ros2_control hardware", "cable_kinematics_file");
  config.calibration_file =
    required_parameter(info.hardware_parameters, "ros2_control hardware", "calibration_file");
  config.software_zero_file = parameter_or(info.hardware_parameters, "software_zero_file", "");
  config.poll_interval = std::chrono::milliseconds{
    parse_value<int>(
      parameter_or(info.hardware_parameters, "state_poll_interval_ms", "20"),
      "state_poll_interval_ms")};
  config.status1_poll_divider = parse_value<std::size_t>(
    parameter_or(info.hardware_parameters, "status1_poll_divider", "5"),
    "status1_poll_divider");
  config.moving_speed_threshold_rad_s = parse_value<double>(
    parameter_or(
      info.hardware_parameters, "moving_speed_threshold_rad_s",
      "0.00017453292519943296"),
    "moving_speed_threshold_rad_s");
  config.state_stale_timeout_sec = parse_value<double>(
    parameter_or(info.hardware_parameters, "state_stale_timeout_sec", "1.0"),
    "state_stale_timeout_sec");
  config.allow_real_control = false;
  config.read_only = true;

  std::unordered_set<int> motor_ids;
  for (const auto & joint : info.joints) {
    const auto limits = info.limits.find(joint.name);
    if (limits == info.limits.end() || !limits->second.has_position_limits ||
      !limits->second.has_velocity_limits || limits->second.max_velocity <= 0.0)
    {
      throw std::invalid_argument("joint '" + joint.name + "' has incomplete URDF limits");
    }
    const int motor_id = parse_value<int>(
      required_parameter(joint.parameters, joint.name, "motor_id"), "motor_id");
    if (motor_id <= 0 || motor_id > 255 || !motor_ids.insert(motor_id).second) {
      throw std::invalid_argument("motor IDs must be unique values in 1..255");
    }
    config.actuators.push_back(
      ActuatorConfig{
        joint.name,
        static_cast<std::uint8_t>(motor_id),
        parse_value<double>(
          required_parameter(joint.parameters, joint.name, "protocol_angle_ratio"),
          "protocol_angle_ratio"),
        parse_value<int>(
          required_parameter(joint.parameters, joint.name, "motor_direction"),
          "motor_direction")});
    config.joint_lower_limits_rad.push_back(limits->second.min_position);
    config.joint_upper_limits_rad.push_back(limits->second.max_position);
    config.joint_feedback_tolerances_rad[config.actuators.size() - 1U] =
      parse_value<double>(parameter_or(
        joint.parameters, "feedback_position_tolerance_rad", "0"),
        "feedback_position_tolerance_rad");
    config.joint_velocity_limits_rad_s.push_back(limits->second.max_velocity);
  }
  return config;
}

}  // namespace pi_arm_hardware
