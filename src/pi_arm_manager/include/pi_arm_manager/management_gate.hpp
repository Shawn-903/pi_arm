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

// ManagerNode 的使能/关闭条件检查，使用完整、新鲜且静止的原始反馈。
// 故障时保留静止后的关闭入口，使能则要求正常状态。
inline MotorPowerDecision decide_motor_power_operation(
  const MotorPowerOperation operation,
  const pi_arm_interfaces::msg::HardwareState & hardware,
  const uint8_t robot_state,
  const bool task_claimed,
  const bool still,
  const int requested_motor_id)
{
  if (hardware.teaching_active && operation == MotorPowerOperation::ENABLE) {
    return {false, false, 2001, "exit drag teaching to hold current position"};
  }
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
  if (robot_state == pi_arm_interfaces::msg::RobotState::FAULT &&
    operation == MotorPowerOperation::ENABLE) {
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
    robot_state != pi_arm_interfaces::msg::RobotState::DISABLED &&
    !(robot_state == pi_arm_interfaces::msg::RobotState::FAULT &&
    operation == MotorPowerOperation::DISABLE))
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

// 恢复操作沿用关闭电机的原始反馈检查，已关闭的电机也继续发送恢复命令。
inline MotorPowerDecision decide_motor_recovery(
  const pi_arm_interfaces::msg::HardwareState & hardware,
  uint8_t state, bool task_claimed, bool still, int motor_id)
{
  auto decision = decide_motor_power_operation(
    MotorPowerOperation::DISABLE, hardware, state, task_claimed, still, motor_id);
  if (decision.already_satisfied) {return {true, false, 0, "ok"};}
  return decision;
}

// 清错保留电源和位置参考，按所选电机的新鲜、静止反馈判断可用性。
// 硬件服务随后核验三个原始静止速度样本。
inline MotorPowerDecision decide_motor_clear_errors(
  const pi_arm_interfaces::msg::HardwareState & hardware,
  const bool task_claimed, const int motor_id)
{
  if (!complete_raw_motor_state(hardware) || !hardware.connected) {
    return {false, false, 2001, "CAN connection or raw motor state is unavailable"};
  }
  if (task_claimed || hardware.motion_stop_pending || hardware.motion_stop_failed) {
    return {false, false, 2001, "clear errors requires no active task and no unconfirmed stop"};
  }
  bool found = false;
  for (std::size_t i = 0; i < hardware.motor_ids.size(); ++i) {
    if (hardware.motor_fresh[i] && hardware.motor_moving[i]) {
      return {false, false, 2001, "clear errors is rejected while a motor is moving"};
    }
    if (motor_id == 0 || motor_id == hardware.motor_ids[i]) {
      found = true;
      if (!hardware.motor_fresh[i] || hardware.motor_moving[i]) {
        return {false, false, 2001, "selected motor feedback must be fresh and stationary"};
      }
    }
  }
  if (!found) {return {false, false, 2004, "motor_id is not configured (use 0 for all motors)"};}
  return {true, false, 0, "ok"};
}

}  // namespace pi_arm_manager
