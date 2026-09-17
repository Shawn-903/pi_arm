#include "pi_arm_hardware/motion_write_gate.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace
{

TEST(MotionWriteGate, EnableOrStartupCannotImplicitlyArm)
{
  pi_arm_hardware::MotionWriteGate gate;
  const std::vector<double> idle(6U, 0.0);

  EXPECT_FALSE(gate.armed());
  EXPECT_FALSE(gate.permit_write(idle, idle));
  EXPECT_FALSE(gate.arm(1U));  // no observed controller baseline yet

  gate.observe_idle(idle, idle);
  EXPECT_FALSE(gate.armed());
  EXPECT_FALSE(gate.permit_write(idle, idle));
}

TEST(MotionWriteGate, RequiresLeaseAndPostArmControllerChange)
{
  pi_arm_hardware::MotionWriteGate gate;
  const std::vector<double> idle(6U, 0.0);
  gate.observe_idle(idle, idle);

  EXPECT_FALSE(gate.arm(0U));
  ASSERT_TRUE(gate.arm(42U));
  EXPECT_TRUE(gate.armed());
  EXPECT_EQ(gate.active_lease_id(), 42U);

  // The command that existed before arming is never promoted to a real write.
  EXPECT_FALSE(gate.permit_write(idle, idle));

  auto moving = idle;
  moving[2] = 0.01;
  EXPECT_TRUE(gate.permit_write(moving, idle));
  EXPECT_TRUE(gate.permit_write(moving, idle));
}

TEST(MotionWriteGate, CancelAndFaultStyleDisarmAreFailClosed)
{
  pi_arm_hardware::MotionWriteGate gate;
  const std::vector<double> idle(6U, 0.0);
  gate.observe_idle(idle, idle);
  ASSERT_TRUE(gate.arm(7U));

  EXPECT_FALSE(gate.disarm(8U));
  EXPECT_TRUE(gate.armed());
  EXPECT_TRUE(gate.disarm(0U));  // zero is the internal force-disarm lease
  EXPECT_FALSE(gate.armed());

  auto moving = idle;
  moving[0] = 0.1;
  EXPECT_FALSE(gate.permit_write(moving, idle));
}

TEST(MotionWriteGate, NonFiniteCommandsNeverCreateOrPassABaseline)
{
  pi_arm_hardware::MotionWriteGate gate;
  std::vector<double> invalid(6U, 0.0);
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  const std::vector<double> velocities(6U, 0.0);

  gate.observe_idle(invalid, velocities);
  EXPECT_FALSE(gate.arm(1U));

  const std::vector<double> idle(6U, 0.0);
  gate.observe_idle(idle, velocities);
  ASSERT_TRUE(gate.arm(1U));
  EXPECT_FALSE(gate.permit_write(invalid, velocities));
}

}  // namespace
