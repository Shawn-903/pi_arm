#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "pi_arm_interfaces/msg/hardware_state.hpp"
#include "pi_arm_interfaces/msg/robot_state.hpp"

namespace pi_arm_manager
{

enum class MotorPowerOperation
{
  ENABLE,
  DISABLE,
};

struct MotorPowerDecision
{
  bool allowed{false};
  bool already_satisfied{false};
  int code{2001};
  const char * message{"operation rejected by robot state"};
};

inline bool complete_raw_motor_state(
  const pi_arm_interfaces::msg::HardwareState & hardware)
{
  const auto count = hardware.motor_ids.size();
  return count > 0U && count == hardware.joint_names.size() &&
    hardware.motor_enabled.size() == count &&
    hardware.motor_errors.size() == count &&
    hardware.motor_moving.size() == count &&
    hardware.motor_fresh.size() == count &&
    hardware.motor_error_codes.size() == count &&
    hardware.motor_ages_sec.size() == count;
}

// This is the actual enable/disable service gate used by ManagerNode.  It
// permits power-state changes only with complete, fresh, fault-free and still
// feedback, while allowing READY <-> partially DISABLED transitions.
inline MotorPowerDecision decide_motor_power_operation(
  const MotorPowerOperation operation,
  const pi_arm_interfaces::msg::HardwareState & hardware,
  const uint8_t robot_state,
  const bool task_claimed,
  const bool still,
  const int requested_motor_id)
{
  if (!complete_raw_motor_state(hardware)) {
    return {false, false, 2001, "raw motor state feedback is incomplete"};
  }

  std::vector<std::size_t> selected;
  for (std::size_t index = 0; index < hardware.motor_ids.size(); ++index) {
    if (requested_motor_id == 0 || hardware.motor_ids[index] == requested_motor_id) {
      selected.push_back(index);
    }
  }
  if (selected.empty()) {
    return {false, false, 2004, "motor_id is not configured (use 0 for all motors)"};
  }

  if (robot_state == pi_arm_interfaces::msg::RobotState::DISCONNECTED ||
    !hardware.connected ||
    !std::all_of(hardware.motor_fresh.begin(), hardware.motor_fresh.end(), [](bool value) {
      return value;
    }))
  {
    return {false, false, 2001, "motor state feedback is disconnected or stale"};
  }
  if (robot_state == pi_arm_interfaces::msg::RobotState::FAULT) {
    return {false, false, 2001, "motor power operation rejected while hardware is in FAULT"};
  }
  if (task_claimed || robot_state == pi_arm_interfaces::msg::RobotState::RUNNING ||
    std::any_of(hardware.motor_moving.begin(), hardware.motor_moving.end(), [](bool value) {
      return value;
    }))
  {
    return {false, false, 2001, "motor power operation rejected while motion is active"};
  }
  if (!still) {
    return {false, false, 2001, "motor power operation requires three stationary feedback frames"};
  }
  if (robot_state != pi_arm_interfaces::msg::RobotState::READY &&
    robot_state != pi_arm_interfaces::msg::RobotState::DISABLED)
  {
    return {false, false, 2001, "motor power operation rejected by robot state"};
  }

  const bool desired_enabled = operation == MotorPowerOperation::ENABLE;
  const bool already_satisfied = std::all_of(
    selected.begin(), selected.end(), [&](const std::size_t index) {
      return hardware.motor_enabled[index] == desired_enabled;
    });
  if (already_satisfied) {
    return {
      false, true, 0,
      desired_enabled ?
      "requested motors are already enabled (confirmed by fresh feedback)" :
      "requested motors are already disabled (confirmed by fresh feedback)"};
  }
  return {true, false, 0, "ok"};
}

}  // namespace pi_arm_manager