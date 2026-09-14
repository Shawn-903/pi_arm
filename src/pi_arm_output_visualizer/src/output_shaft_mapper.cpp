#include "pi_arm_output_visualizer/output_shaft_mapper.hpp"

#include "pi_arm_transmission/result.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <sstream>
#include <utility>

namespace pi_arm_output_visualizer
{

namespace
{

const std::array<std::string, 6> kOutputShaftNames{
  "motor_01_output", "motor_02_output", "motor_03_output",
  "motor_04_output", "motor_05_output", "motor_06_output"};

const std::array<std::string, 6> kActiveJointNames{
  "joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"};

std::string format_kinematics_error(const pi_arm_transmission::KinematicsStatus & status)
{
  std::ostringstream stream;
  stream << pi_arm_transmission::to_string(status.error)
         << " at axis " << status.axis << " (value=" << status.value << ")";
  return stream.str();
}

}  // namespace

OutputShaftMapper::OutputShaftMapper(
  pi_arm_transmission::CableKinematicsParameters parameters)
: kinematics_(std::move(parameters))
{
}

const std::array<std::string, 6> & OutputShaftMapper::output_shaft_names()
{
  return kOutputShaftNames;
}

const std::array<std::string, 6> & OutputShaftMapper::active_joint_names()
{
  return kActiveJointNames;
}

bool OutputShaftMapper::map(
  const sensor_msgs::msg::JointState & input,
  sensor_msgs::msg::JointState & output,
  std::string & error) const
{
  error.clear();
  if (input.name.size() != input.position.size()) {
    error = "input name and position arrays must have the same length";
    return false;
  }
  const bool has_velocity = !input.velocity.empty();
  if (has_velocity && input.velocity.size() != input.name.size()) {
    error = "a non-empty velocity array must have the same length as name";
    return false;
  }

  pi_arm_transmission::Vector6 output_position{};
  pi_arm_transmission::Vector6 output_velocity{};
  std::array<bool, 6> found{};

  for (std::size_t input_index = 0; input_index < input.name.size(); ++input_index) {
    const auto expected = std::find(
      kOutputShaftNames.begin(), kOutputShaftNames.end(), input.name[input_index]);
    if (expected == kOutputShaftNames.end()) {
      continue;
    }
    const auto axis = static_cast<std::size_t>(
      std::distance(kOutputShaftNames.begin(), expected));
    if (found[axis]) {
      error = "duplicate output-shaft name: " + input.name[input_index];
      return false;
    }
    if (!std::isfinite(input.position[input_index])) {
      error = "non-finite output-shaft position for " + input.name[input_index];
      return false;
    }
    output_position[axis] = input.position[input_index];
    if (has_velocity) {
      if (!std::isfinite(input.velocity[input_index])) {
        error = "non-finite output-shaft velocity for " + input.name[input_index];
        return false;
      }
      output_velocity[axis] = input.velocity[input_index];
    }
    found[axis] = true;
  }

  for (std::size_t axis = 0; axis < found.size(); ++axis) {
    if (!found[axis]) {
      error = "missing required output-shaft name: " + kOutputShaftNames[axis];
      return false;
    }
  }

  const auto limit_status = kinematics_.check_output_position_limits(output_position);
  if (!limit_status) {
    error = "output-shaft position rejected: " + format_kinematics_error(limit_status);
    return false;
  }

  pi_arm_transmission::Vector6 joint_position{};
  const auto position_status =
    kinematics_.output_to_joint_position(output_position, joint_position);
  if (!position_status) {
    error = "output-to-joint position mapping failed: " +
      format_kinematics_error(position_status);
    return false;
  }

  pi_arm_transmission::Vector6 joint_velocity{};
  if (has_velocity) {
    const auto velocity_status = kinematics_.output_to_joint_velocity(
      joint_position, output_velocity, joint_velocity);
    if (!velocity_status) {
      error = "output-to-joint velocity mapping failed: " +
        format_kinematics_error(velocity_status);
      return false;
    }
  }

  output = sensor_msgs::msg::JointState{};
  output.header = input.header;
  output.name.assign(kActiveJointNames.begin(), kActiveJointNames.end());
  output.position.assign(joint_position.begin(), joint_position.end());
  if (has_velocity) {
    output.velocity.assign(joint_velocity.begin(), joint_velocity.end());
  }
  return true;
}

}  // namespace pi_arm_output_visualizer
