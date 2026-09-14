#pragma once

#include "pi_arm_transmission/cable_kinematics.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <array>
#include <string>

namespace pi_arm_output_visualizer
{

class OutputShaftMapper
{
public:
  explicit OutputShaftMapper(pi_arm_transmission::CableKinematicsParameters parameters);

  bool map(
    const sensor_msgs::msg::JointState & input,
    sensor_msgs::msg::JointState & output,
    std::string & error) const;

  static const std::array<std::string, 6> & output_shaft_names();
  static const std::array<std::string, 6> & active_joint_names();

private:
  pi_arm_transmission::CableKinematics kinematics_;
};

}  // namespace pi_arm_output_visualizer
