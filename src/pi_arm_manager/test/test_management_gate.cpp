#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "pi_arm_manager/management_gate.hpp"

using pi_arm_interfaces::msg::HardwareState;
using pi_arm_interfaces::msg::RobotState;
using pi_arm_manager::MotorPowerOperation;

namespace
{

HardwareState healthy_motors()
{
  HardwareState state;
  state.connected = true;
  state.joint_names = {
    "joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"};
  state.motor_ids = {1, 2, 3, 4, 5, 6};
  state.motor_enabled = {true, true, true, true, true, true};
  state.motor_errors = {false, false, false, false, false, false};
  state.motor_moving = {false, false, false, false, false, false};
  state.motor_fresh = {true, true, true, true, true, true};
  state.motor_error_codes = {0, 0, 0, 0, 0, 0};
  state.motor_ages_sec = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
  return state;
}

}  // namespace

TEST(ManagementGate, AllowsSequentialDisableInDifferentOrders)
{
  const std::vector<std::vector<int>> orders{
    {1, 2, 3, 4, 5, 6},
    {2, 6, 1, 4, 3, 5},
  };
  for (const auto & order : orders) {
    auto hardware = healthy_motors();
    for (const int motor_id : order) {
      const auto decision = pi_arm_manager::decide_motor_power_operation(
        MotorPowerOperation::DISABLE, hardware,
        std::all_of(
          hardware.motor_enabled.begin(), hardware.motor_enabled.end(), [](bool value) {
            return value;
          }) ? RobotState::READY : RobotState::DISABLED,
        false, true, motor_id);
      EXPECT_TRUE(decision.allowed) << motor_id;
      EXPECT_FALSE(decision.already_satisfied) << motor_id;
      hardware.motor_enabled[static_cast<std::size_t>(motor_id - 1)] = false;
    }
  }
}

TEST(ManagementGate, AllowsDisableAllFromPartialState)
{
  auto hardware = healthy_motors();
  hardware.motor_enabled[1] = false;
  const auto decision = pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::DISABLED, false, true, 0);
  EXPECT_TRUE(decision.allowed);
  EXPECT_FALSE(decision.already_satisfied);
}

TEST(ManagementGate, ReportsFreshFeedbackIdempotently)
{
  auto hardware = healthy_motors();
  auto decision = pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::ENABLE, hardware, RobotState::READY, false, true, 0);
  EXPECT_FALSE(decision.allowed);
  EXPECT_TRUE(decision.already_satisfied);
  EXPECT_EQ(decision.code, 0);

  hardware.motor_enabled[3] = false;
  decision = pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::DISABLED, false, true, 4);
  EXPECT_FALSE(decision.allowed);
  EXPECT_TRUE(decision.already_satisfied);
  EXPECT_EQ(decision.code, 0);
}

TEST(ManagementGate, AllowsSequentialEnableFromPartialState)
{
  auto hardware = healthy_motors();
  std::fill(hardware.motor_enabled.begin(), hardware.motor_enabled.end(), false);
  for (const int motor_id : {5, 1, 6, 2, 4, 3}) {
    const auto decision = pi_arm_manager::decide_motor_power_operation(
      MotorPowerOperation::ENABLE, hardware, RobotState::DISABLED, false, true, motor_id);
    EXPECT_TRUE(decision.allowed) << motor_id;
    hardware.motor_enabled[static_cast<std::size_t>(motor_id - 1)] = true;
  }
}

TEST(ManagementGate, RejectsInvalidMotorId)
{
  const auto decision = pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, healthy_motors(), RobotState::READY, false, true, 7);
  EXPECT_FALSE(decision.allowed);
  EXPECT_FALSE(decision.already_satisfied);
  EXPECT_EQ(decision.code, 2004);
}

TEST(ManagementGate, RejectsUnsafeOrUnverifiedStates)
{
  auto hardware = healthy_motors();
  EXPECT_FALSE(pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::RUNNING, false, true, 1).allowed);
  EXPECT_FALSE(pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::READY, true, true, 1).allowed);
  EXPECT_FALSE(pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::READY, false, false, 1).allowed);
  EXPECT_FALSE(pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::ENABLE, hardware, RobotState::FAULT, false, true, 1).allowed);

  hardware.motor_moving[4] = true;
  EXPECT_FALSE(pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::DISABLED, false, true, 1).allowed);
  hardware.motor_moving[4] = false;
  hardware.motor_fresh[2] = false;
  EXPECT_FALSE(pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::DISCONNECTED, false, true, 1).allowed);

  hardware = healthy_motors();
  hardware.motor_enabled.pop_back();
  EXPECT_FALSE(pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::DISCONNECTED, false, true, 1).allowed);
}

TEST(ManagementGate, AllowsControlledDisableDespiteKinematicsOrMotorFault)
{
  auto hardware = healthy_motors();
  hardware.kinematics_valid = false;
  hardware.motor_errors[1] = true;
  EXPECT_TRUE(pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::FAULT, false, true, 0).allowed);
  EXPECT_FALSE(pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::FAULT, true, true, 0).allowed);
  EXPECT_FALSE(pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::FAULT, false, false, 0).allowed);
  hardware.motor_fresh[0] = false;
  EXPECT_FALSE(pi_arm_manager::decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, RobotState::FAULT, false, true, 0).allowed);
}

TEST(ManagementGate, RecoveryAllowsFaultEvenWhenKinematicsAreInvalid)
{
  auto h = healthy_motors();
  h.motor_errors[1] = true;
  h.motor_error_codes[1] = 8;
  h.kinematics_valid = false;
  h.control_permitted = false;
  EXPECT_TRUE(pi_arm_manager::decide_motor_recovery(h, RobotState::FAULT, false, true, 2).allowed);
  h.motor_enabled.assign(6, false);
  EXPECT_TRUE(pi_arm_manager::decide_motor_recovery(h, RobotState::FAULT, false, true, 2).allowed);
  EXPECT_FALSE(pi_arm_manager::decide_motor_recovery(h, RobotState::FAULT, true, true, 2).allowed);
  EXPECT_FALSE(pi_arm_manager::decide_motor_recovery(h, RobotState::FAULT, false, false, 2).allowed);
  EXPECT_FALSE(pi_arm_manager::decide_motor_recovery(h, RobotState::FAULT, false, true, 7).allowed);
  h.motor_fresh[0] = false;
  EXPECT_FALSE(pi_arm_manager::decide_motor_recovery(h, RobotState::FAULT, false, true, 2).allowed);
}
