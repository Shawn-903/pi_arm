#include <gtest/gtest.h>

#include "pi_arm_manager/state_aggregator.hpp"

using pi_arm_interfaces::msg::HardwareState;
using pi_arm_interfaces::msg::RobotState;
using pi_arm_interfaces::msg::TaskState;

static HardwareState healthy_hardware()
{
  HardwareState state;
  state.connected = true;
  state.joint_names = {"joint1", "joint2"};
  state.positions = {0.0, 0.0};
  state.velocities = {0.0, 0.0};
  state.fresh = {true, true};
  state.enabled = {true, true};
  state.errors = {false, false};
  state.moving = {false, false};
  state.error_codes = {0, 0};
  state.ages_sec = {0.01, 0.01};
  state.motor_ids = {1, 2};
  state.motor_enabled = {true, true};
  state.motor_errors = {false, false};
  state.motor_moving = {false, false};
  state.motor_fresh = {true, true};
  state.motor_error_codes = {0, 0};
  state.motor_ages_sec = {0.01, 0.01};
  state.calibrated = true;
  state.kinematics_valid = true;
  state.control_permitted = true;
  state.calibration_id = "test";
  state.raw_protocol_positions_deg = {0.0, 0.0};
  state.raw_output_positions_rad = {0.0, 0.0};
  state.software_output_offsets_rad = {0.0, 0.0};
  state.model_output_positions_rad = {0.0, 0.0};
  return state;
}

TEST(StateAggregator, PhysicalMotionIsRunningWithoutTask)
{
  auto hardware = healthy_hardware();
  TaskState task;
  task.status = TaskState::IDLE;
  hardware.moving[1] = true;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::RUNNING);
}

TEST(StateAggregator, PendingStopDoesNotTurnFinishingTaskIntoHardwareFailure)
{
  auto hardware = healthy_hardware();
  TaskState task;
  task.status = TaskState::RUNNING;
  hardware.control_permitted = false;
  hardware.motion_stop_pending = true;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::RUNNING);
  task.status = TaskState::IDLE;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISABLED);
  hardware.motion_stop_failed = true;
  task.status = TaskState::RUNNING;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::FAULT);
}

TEST(StateAggregator, AppliesStrictPriority)
{
  auto hardware = healthy_hardware();
  TaskState task;
  task.status = TaskState::RUNNING;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::RUNNING);

  hardware.enabled[0] = false;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISABLED);
  hardware.errors[0] = true;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::FAULT);
  hardware.connected = false;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISCONNECTED);
}

TEST(StateAggregator, ReadyRequiresHealthyEnabledHardware)
{
  auto hardware = healthy_hardware();
  TaskState task;
  task.status = TaskState::IDLE;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::READY);
  hardware.fresh[1] = false;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISABLED);
}

TEST(StateAggregator, RejectsIncompleteHardwareFrames)
{
  auto hardware = healthy_hardware();
  TaskState task;
  task.status = TaskState::IDLE;
  hardware.velocities.pop_back();
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISABLED);
}

TEST(StateAggregator, CalibrationAndControlPermissionFailClosed)
{
  auto hardware = healthy_hardware();
  TaskState task;
  task.status = TaskState::IDLE;

  hardware.calibrated = false;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISABLED);
  hardware.calibrated = true;
  hardware.kinematics_valid = false;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISABLED);
  hardware.kinematics_valid = true;
  hardware.control_permitted = false;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISABLED);
}
