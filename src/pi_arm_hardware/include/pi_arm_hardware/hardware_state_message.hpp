#pragma once

#include "pi_arm_interfaces/msg/hardware_state.hpp"

#include <string>
#include <vector>

namespace pi_arm_hardware
{

struct BackendConfig;
struct HardwareSample;

// Shared ROS boundary for the control plugin and the hardware monitor.
// The monitor adds its live axis-test status after converting the sample.
pi_arm_interfaces::msg::HardwareState make_hardware_state_message(
  const HardwareSample & sample, const std::vector<std::string> & joint_names,
  const BackendConfig & config, const builtin_interfaces::msg::Time & stamp);

}  // namespace pi_arm_hardware
