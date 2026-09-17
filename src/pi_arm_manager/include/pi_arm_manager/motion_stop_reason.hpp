#pragma once

#include <cstdint>

#include "pi_arm_interfaces/msg/robot_state.hpp"
#include "pi_arm_interfaces/msg/task_state.hpp"

namespace pi_arm_manager
{

// Preserve why an active motion was interrupted. A single stop_requested flag
// is not sufficient: a user cancellation and a hardware fault must not produce
// the same public Action/TaskState result.
enum class MotionStopReason : std::uint8_t
{
  NONE = 0,
  ACTION_CANCEL,
  STOP_SERVICE,
  HARDWARE_DISABLED,
  HARDWARE_DISCONNECTED,
  HARDWARE_FAULT,
};

struct MotionStopOutcome
{
  bool success{false};
  std::int32_t code{0};
  std::uint8_t task_status{pi_arm_interfaces::msg::TaskState::FAILED};
  const char * message{"motion stopped"};
};

inline int motion_stop_priority(const MotionStopReason reason) noexcept
{
  switch (reason) {
    case MotionStopReason::NONE: return 0;
    case MotionStopReason::ACTION_CANCEL: return 1;
    case MotionStopReason::STOP_SERVICE: return 2;
    case MotionStopReason::HARDWARE_DISABLED: return 3;
    case MotionStopReason::HARDWARE_DISCONNECTED: return 4;
    case MotionStopReason::HARDWARE_FAULT: return 5;
  }
  return 0;
}

inline const char * motion_stop_reason_name(const MotionStopReason reason) noexcept
{
  switch (reason) {
    case MotionStopReason::NONE: return "none";
    case MotionStopReason::ACTION_CANCEL: return "action_cancel";
    case MotionStopReason::STOP_SERVICE: return "stop_motion";
    case MotionStopReason::HARDWARE_DISABLED: return "hardware_disabled";
    case MotionStopReason::HARDWARE_DISCONNECTED: return "hardware_disconnected";
    case MotionStopReason::HARDWARE_FAULT: return "hardware_fault";
  }
  return "unknown";
}

inline MotionStopReason motion_stop_reason_for_robot_state(const std::uint8_t state) noexcept
{
  switch (state) {
    case pi_arm_interfaces::msg::RobotState::DISCONNECTED:
      return MotionStopReason::HARDWARE_DISCONNECTED;
    case pi_arm_interfaces::msg::RobotState::FAULT:
      return MotionStopReason::HARDWARE_FAULT;
    case pi_arm_interfaces::msg::RobotState::DISABLED:
      return MotionStopReason::HARDWARE_DISABLED;
    default:
      return MotionStopReason::NONE;
  }
}

inline MotionStopOutcome motion_stop_outcome(const MotionStopReason reason) noexcept
{
  switch (reason) {
    case MotionStopReason::ACTION_CANCEL:
      return {
        true, 0, pi_arm_interfaces::msg::TaskState::CANCELED,
        "motion canceled by action client"};
    case MotionStopReason::STOP_SERVICE:
      return {
        false, 3003, pi_arm_interfaces::msg::TaskState::CANCELED,
        "motion stopped by stop_motion"};
    case MotionStopReason::HARDWARE_DISABLED:
      return {
        false, 3103, pi_arm_interfaces::msg::TaskState::FAILED,
        "motion stopped: hardware disabled"};
    case MotionStopReason::HARDWARE_DISCONNECTED:
      return {
        false, 3101, pi_arm_interfaces::msg::TaskState::FAILED,
        "motion stopped: hardware disconnected or feedback stale"};
    case MotionStopReason::HARDWARE_FAULT:
      return {
        false, 3102, pi_arm_interfaces::msg::TaskState::FAILED,
        "motion stopped: hardware fault"};
    case MotionStopReason::NONE:
      break;
  }
  return {
    false, 3004, pi_arm_interfaces::msg::TaskState::FAILED,
    "motion stopped without a recorded reason"};
}

}  // namespace pi_arm_manager
