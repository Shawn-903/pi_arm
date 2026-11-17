#include "pi_arm_hardware/pi_arm_system.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <string>
#include <utility>

namespace
{

hardware_interface::HardwareInfo mock_hardware_info()
{
  hardware_interface::HardwareInfo info;
  info.name = "configuration_test";
  info.type = "system";
  info.hardware_parameters = {
    {"backend", "mock"}, {"cable_kinematics_file", PI_ARM_TEST_CABLE_CONFIG}};
  constexpr std::array<const char *, 6> names{
    "joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"};
  for (std::size_t index = 0; index < names.size(); ++index) {
    hardware_interface::ComponentInfo joint;
    joint.name = names[index];
    joint.type = "joint";
    joint.parameters = {
      {"motor_id", std::to_string(index + 1U)},
      {"protocol_angle_ratio", "1"}, {"motor_direction", "1"}};
    for (const auto * name : {"position", "velocity"}) {
      hardware_interface::InterfaceInfo interface;
      interface.name = name;
      joint.command_interfaces.push_back(interface);
      joint.state_interfaces.push_back(interface);
    }
    auto & limits = info.limits[joint.name];
    limits.has_position_limits = true;
    limits.min_position = -1.0;
    limits.max_position = 1.0;
    limits.has_velocity_limits = true;
    limits.max_velocity = 1.0;
    info.joints.push_back(std::move(joint));
  }
  return info;
}

TEST(SystemConfiguration, RejectsInvalidJointCount)
{
  auto missing = mock_hardware_info();
  missing.joints.pop_back();
  auto duplicate = mock_hardware_info();
  duplicate.joints.push_back(duplicate.joints.back());
  auto empty = mock_hardware_info();
  empty.joints.clear();
  for (const auto & info : {missing, duplicate, empty}) {
    SCOPED_TRACE(info.joints.size());
    pi_arm_hardware::PiArmSystem system;
    EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
  }
}

TEST(SystemConfiguration, ValidConfigurationExportsSixJointInterfaces)
{
  pi_arm_hardware::PiArmSystem system;
  ASSERT_EQ(system.on_init(mock_hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.export_state_interfaces().size(), 12U);
  EXPECT_EQ(system.export_command_interfaces().size(), 12U);
}

TEST(SystemConfiguration, InvalidParameterReturnsError)
{
  auto info = mock_hardware_info();
  info.hardware_parameters["state_poll_interval_ms"] = "not-a-number";
  pi_arm_hardware::PiArmSystem system;
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
}

}  // namespace
