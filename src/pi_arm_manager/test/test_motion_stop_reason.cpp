#include <gtest/gtest.h>

#include "pi_arm_interfaces/msg/task_state.hpp"
#include "pi_arm_manager/motion_stop_reason.hpp"

namespace pi_arm_manager
{
namespace
{

TEST(MotionStopReason, UserCancelIsCanceledAndSuccessful)
{
  const auto outcome = motion_stop_outcome(MotionStopReason::ACTION_CANCEL);
  EXPECT_TRUE(outcome.success);
  EXPECT_EQ(outcome.code, 0);
  EXPECT_EQ(outcome.task_status, pi_arm_interfaces::msg::TaskState::CANCELED);
}

TEST(MotionStopReason, StopServiceDoesNotReportMotionSuccess)
{
  const auto outcome = motion_stop_outcome(MotionStopReason::STOP_SERVICE);
  EXPECT_FALSE(outcome.success);
  EXPECT_NE(outcome.code, 0);
  EXPECT_EQ(outcome.task_status, pi_arm_interfaces::msg::TaskState::CANCELED);
}

TEST(MotionStopReason, HardwareInterruptionsAreFailures)
{
  for (const auto reason : {
      MotionStopReason::HARDWARE_DISABLED,
      MotionStopReason::HARDWARE_DISCONNECTED,
      MotionStopReason::HARDWARE_FAULT})
  {
    const auto outcome = motion_stop_outcome(reason);
    EXPECT_FALSE(outcome.success);
    EXPECT_NE(outcome.code, 0);
    EXPECT_EQ(outcome.task_status, pi_arm_interfaces::msg::TaskState::FAILED);
  }
}

TEST(MotionStopReason, RobotStateMapsToSpecificHardwareReason)
{
  EXPECT_EQ(
    motion_stop_reason_for_robot_state(pi_arm_interfaces::msg::RobotState::DISCONNECTED),
    MotionStopReason::HARDWARE_DISCONNECTED);
  EXPECT_EQ(
    motion_stop_reason_for_robot_state(pi_arm_interfaces::msg::RobotState::FAULT),
    MotionStopReason::HARDWARE_FAULT);
  EXPECT_EQ(
    motion_stop_reason_for_robot_state(pi_arm_interfaces::msg::RobotState::DISABLED),
    MotionStopReason::HARDWARE_DISABLED);
  EXPECT_EQ(
    motion_stop_reason_for_robot_state(pi_arm_interfaces::msg::RobotState::READY),
    MotionStopReason::NONE);
}

TEST(MotionStopReason, HardwareReasonsOverrideUserRequests)
{
  EXPECT_GT(
    motion_stop_priority(MotionStopReason::HARDWARE_DISABLED),
    motion_stop_priority(MotionStopReason::STOP_SERVICE));
  EXPECT_GT(
    motion_stop_priority(MotionStopReason::HARDWARE_DISCONNECTED),
    motion_stop_priority(MotionStopReason::HARDWARE_DISABLED));
  EXPECT_GT(
    motion_stop_priority(MotionStopReason::HARDWARE_FAULT),
    motion_stop_priority(MotionStopReason::HARDWARE_DISCONNECTED));
}

}  // namespace
}  // namespace pi_arm_manager
