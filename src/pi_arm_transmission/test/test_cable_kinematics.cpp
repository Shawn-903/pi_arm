#include "pi_arm_transmission/cable_kinematics.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTolerance = 1.0e-10;
constexpr double kAxis1Gain = 79.0 / 65.0;
constexpr double kAxis3Gain = 207.0 / 65.0;
constexpr double kAxis45Gain = 33.94 / 16.25;
constexpr double kAxis345CouplingGain = 114.0 / 65.0;
constexpr double kAxis6CouplingGain =
  0.5 * (20.25 / 12.25 + 28.5 / 17.25);

pi_arm_transmission::CableKinematicsParameters parameters()
{
  pi_arm_transmission::CableKinematicsParameters value;
  value.axis1 = {16.25, 19.75};
  value.axis2 = {16.0, 16.0};
  value.axis3 = {16.25, 28.5, 23.25, 28.5, 1};
  value.axis4 = {16.25, 33.94, 28.5, 1};
  value.axis5 = {16.25, 33.94, 28.5, 1};
  value.axis6_branch_a = {12.25, 12.25, 20.25, 1};
  value.axis6_branch_b = {17.25, 17.25, 28.5, 1};
  value.output_limits.fill({-10.0, 10.0});
  value.compatibility_tolerance = 1.0e-3;
  return value;
}

TEST(CableKinematics, RejectsInvalidAndIncompatibleParameters)
{
  auto invalid_radius = parameters();
  invalid_radius.axis3.motor_radius_mm = 0.0;
  EXPECT_THROW(
    pi_arm_transmission::CableKinematics kinematics(invalid_radius),
    std::invalid_argument);

  auto invalid_sign = parameters();
  invalid_sign.axis4.coupling_sign = 0;
  EXPECT_THROW(
    pi_arm_transmission::CableKinematics kinematics(invalid_sign),
    std::invalid_argument);

  auto incompatible_self = parameters();
  incompatible_self.axis6_branch_a.joint_radius_mm = 13.0;
  EXPECT_THROW(
    pi_arm_transmission::CableKinematics kinematics(incompatible_self),
    std::invalid_argument);

  auto incompatible_coupling = parameters();
  incompatible_coupling.axis6_branch_a.joint2_guide_radius_mm = 21.0;
  EXPECT_THROW(
    pi_arm_transmission::CableKinematics kinematics(incompatible_coupling),
    std::invalid_argument);

  auto invalid_output_limit = parameters();
  invalid_output_limit.output_limits[2] = {1.0, -1.0};
  EXPECT_THROW(
    pi_arm_transmission::CableKinematics kinematics(invalid_output_limit),
    std::invalid_argument);
}

TEST(CableKinematics, MapsJointPositionWithJoint2Coupling)
{
  const pi_arm_transmission::CableKinematics kinematics(parameters());
  const pi_arm_transmission::Vector6 joint{0.1, 0.2, -0.3, 0.4, -0.5, 0.6};
  pi_arm_transmission::Vector6 output{};

  ASSERT_TRUE(kinematics.joint_to_output_position(joint, output));
  EXPECT_NEAR(output[0], kAxis1Gain * joint[0], kTolerance);
  EXPECT_NEAR(output[1], joint[1], kTolerance);
  EXPECT_NEAR(
    output[2], kAxis3Gain * joint[2] + kAxis345CouplingGain * joint[1],
    kTolerance);
  EXPECT_NEAR(
    output[3],
    kAxis45Gain * std::sin(joint[3]) + kAxis345CouplingGain * joint[1],
    kTolerance);
  EXPECT_NEAR(
    output[4],
    kAxis45Gain * std::sin(joint[4]) + kAxis345CouplingGain * joint[1],
    kTolerance);
  EXPECT_NEAR(
    output[5], joint[5] + kAxis6CouplingGain * joint[1], kTolerance);

  pi_arm_transmission::Vector6 recovered{};
  ASSERT_TRUE(kinematics.output_to_joint_position(output, recovered));
  for (std::size_t index = 0; index < joint.size(); ++index) {
    EXPECT_NEAR(recovered[index], joint[index], kTolerance);
  }
}

TEST(CableKinematics, Joint2OnlyMovesDownstreamOutputAxes)
{
  const pi_arm_transmission::CableKinematics kinematics(parameters());
  const pi_arm_transmission::Vector6 joint{0.0, 0.3, 0.0, 0.0, 0.0, 0.0};
  pi_arm_transmission::Vector6 output{};

  ASSERT_TRUE(kinematics.joint_to_output_position(joint, output));
  EXPECT_DOUBLE_EQ(output[0], 0.0);
  EXPECT_NEAR(output[1], 0.3, kTolerance);
  for (std::size_t index = 2; index < 5U; ++index) {
    EXPECT_NEAR(output[index], kAxis345CouplingGain * 0.3, kTolerance);
  }
  EXPECT_NEAR(output[5], kAxis6CouplingGain * 0.3, kTolerance);
}

TEST(CableKinematics, ReportsInverseDomainAndVelocitySingularity)
{
  const pi_arm_transmission::CableKinematics kinematics(parameters());
  pi_arm_transmission::Vector6 output{};
  output[3] = kAxis45Gain * 1.01;
  pi_arm_transmission::Vector6 joint{};
  const auto position_status = kinematics.output_to_joint_position(output, joint);
  EXPECT_EQ(position_status.error, pi_arm_transmission::KinematicsError::ASIN_DOMAIN);
  EXPECT_EQ(position_status.axis, 4U);

  joint = {};
  joint[3] = kPi / 2.0;
  pi_arm_transmission::Vector6 output_velocity{};
  pi_arm_transmission::Vector6 joint_velocity{};
  const auto velocity_status =
    kinematics.output_to_joint_velocity(joint, output_velocity, joint_velocity);
  EXPECT_EQ(
    velocity_status.error, pi_arm_transmission::KinematicsError::VELOCITY_SINGULARITY);
  EXPECT_EQ(velocity_status.axis, 4U);
}

TEST(CableKinematics, RejectsNonFiniteRuntimeInput)
{
  const pi_arm_transmission::CableKinematics kinematics(parameters());
  pi_arm_transmission::Vector6 joint{};
  joint[2] = std::numeric_limits<double>::quiet_NaN();
  pi_arm_transmission::Vector6 output{};
  const auto status = kinematics.joint_to_output_position(joint, output);
  EXPECT_EQ(status.error, pi_arm_transmission::KinematicsError::NON_FINITE_INPUT);
  EXPECT_EQ(status.axis, 3U);
}

TEST(CableKinematics, ChecksConfiguredOutputPositionLimitsWithoutClamping)
{
  auto value = parameters();
  value.output_limits[2] = {-1.0, 2.0};
  value.output_limit_tolerance_rad = 1.0e-6;
  const pi_arm_transmission::CableKinematics kinematics(value);

  pi_arm_transmission::Vector6 output{};
  output[2] = 2.0 + 0.5e-6;
  EXPECT_TRUE(kinematics.check_output_position_limits(output));

  output[2] = 2.0 + 2.0e-6;
  const auto status = kinematics.check_output_position_limits(output);
  EXPECT_EQ(status.error, pi_arm_transmission::KinematicsError::OUTPUT_POSITION_LIMIT);
  EXPECT_EQ(status.axis, 3U);
  EXPECT_DOUBLE_EQ(status.value, output[2]);
}

TEST(CableKinematics, VelocityMatchesAnalyticAndInverseMapping)
{
  const pi_arm_transmission::CableKinematics kinematics(parameters());
  const pi_arm_transmission::Vector6 joint{0.1, 0.2, -0.3, 0.4, -0.5, 0.6};
  const pi_arm_transmission::Vector6 joint_velocity{
    0.01, -0.02, 0.03, -0.04, 0.05, -0.06};
  pi_arm_transmission::Vector6 output_velocity{};

  ASSERT_TRUE(
    kinematics.joint_to_output_velocity(joint, joint_velocity, output_velocity));
  EXPECT_NEAR(output_velocity[0], kAxis1Gain * joint_velocity[0], kTolerance);
  EXPECT_NEAR(output_velocity[1], joint_velocity[1], kTolerance);
  EXPECT_NEAR(
    output_velocity[2],
    kAxis3Gain * joint_velocity[2] +
    kAxis345CouplingGain * joint_velocity[1], kTolerance);
  EXPECT_NEAR(
    output_velocity[3],
    kAxis45Gain * std::cos(joint[3]) * joint_velocity[3] +
    kAxis345CouplingGain * joint_velocity[1],
    kTolerance);
  EXPECT_NEAR(
    output_velocity[4],
    kAxis45Gain * std::cos(joint[4]) * joint_velocity[4] +
    kAxis345CouplingGain * joint_velocity[1],
    kTolerance);
  EXPECT_NEAR(
    output_velocity[5],
    joint_velocity[5] + kAxis6CouplingGain * joint_velocity[1], kTolerance);

  pi_arm_transmission::Vector6 recovered{};
  ASSERT_TRUE(
    kinematics.output_to_joint_velocity(joint, output_velocity, recovered));
  for (std::size_t index = 0; index < joint_velocity.size(); ++index) {
    EXPECT_NEAR(recovered[index], joint_velocity[index], kTolerance);
  }
}

TEST(CableKinematics, JacobianMatchesFiniteDifference)
{
  const pi_arm_transmission::CableKinematics kinematics(parameters());
  const pi_arm_transmission::Vector6 joint{0.1, 0.2, -0.3, 0.4, -0.5, 0.6};
  pi_arm_transmission::Matrix6 jacobian{};
  ASSERT_TRUE(kinematics.jacobian(joint, jacobian));

  constexpr double step = 1.0e-7;
  for (std::size_t column = 0; column < joint.size(); ++column) {
    auto plus = joint;
    auto minus = joint;
    plus[column] += step;
    minus[column] -= step;
    pi_arm_transmission::Vector6 output_plus{};
    pi_arm_transmission::Vector6 output_minus{};
    ASSERT_TRUE(kinematics.joint_to_output_position(plus, output_plus));
    ASSERT_TRUE(kinematics.joint_to_output_position(minus, output_minus));
    for (std::size_t row = 0; row < joint.size(); ++row) {
      const double numerical = (output_plus[row] - output_minus[row]) / (2.0 * step);
      EXPECT_NEAR(jacobian[row][column], numerical, 1.0e-8);
    }
  }
}

TEST(CableKinematics, RosActiveCoordinatesMatchEquivalentMechanicalFormula)
{
  const pi_arm_transmission::CableKinematics kinematics(parameters());
  const pi_arm_transmission::Vector6 ros_joint{0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  pi_arm_transmission::Vector6 output{};
  ASSERT_TRUE(kinematics.joint_to_output_position(ros_joint, output));

  const double s3 = 2.0 * ros_joint[2];
  const double s4 = 2.0 * ros_joint[3];
  const double s5 = 2.0 * ros_joint[4];
  const double alpha33 = 28.5 / 16.25;
  const double alpha34 = 23.25 / 16.25;

  EXPECT_NEAR(
    output[2],
    0.5 * (alpha33 + alpha34) * s3 +
    kAxis345CouplingGain * ros_joint[1],
    kTolerance);
  EXPECT_NEAR(
    output[3],
    kAxis45Gain * std::sin(s4 / 2.0) +
    kAxis345CouplingGain * ros_joint[1],
    kTolerance);
  EXPECT_NEAR(
    output[4],
    kAxis45Gain * std::sin(s5 / 2.0) +
    kAxis345CouplingGain * ros_joint[1],
    kTolerance);
}

TEST(CableKinematics, Axis6BranchesCloseSimultaneously)
{
  const pi_arm_transmission::CableKinematics kinematics(parameters());
  const pi_arm_transmission::Vector6 joint{0.0, 0.31, 0.0, 0.0, 0.0, -0.27};
  pi_arm_transmission::Vector6 output{};
  ASSERT_TRUE(kinematics.joint_to_output_position(joint, output));

  const auto residual = kinematics.axis6_residual(joint[1], joint[5], output[5]);
  const double expected_a =
    12.25 * (kAxis6CouplingGain - 20.25 / 12.25) * joint[1];
  const double expected_b =
    17.25 * (kAxis6CouplingGain - 28.5 / 17.25) * joint[1];
  EXPECT_NEAR(residual.branch_a_mm, expected_a, kTolerance);
  EXPECT_NEAR(residual.branch_b_mm, expected_b, kTolerance);
  EXPECT_LT(std::abs(residual.branch_a_mm), 0.01);
  EXPECT_LT(std::abs(residual.branch_b_mm), 0.01);
}

}  // namespace
