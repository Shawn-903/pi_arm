#include "pi_arm_hardware/backend.hpp"
#include "pi_arm_hardware/actuator_coordinates.hpp"

#include <gtest/gtest.h>

#include <algorithm>

#include <string>
#include <vector>

namespace
{

pi_arm_hardware::BackendConfig make_config()
{
  pi_arm_hardware::BackendConfig config;
  config.cable_kinematics_file = PI_ARM_TEST_CABLE_CONFIG;
  const std::vector<std::uint8_t> motor_ids{1U, 2U, 3U, 4U, 5U, 6U};
  const std::vector<std::string> joint_names{
    "joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"};
  for (std::size_t index = 0; index < motor_ids.size(); ++index) {
    config.actuators.push_back(
      pi_arm_hardware::ActuatorConfig{
        joint_names[index], motor_ids[index], index < 3U ? 36.0 : 6.0, 1});
  }
  // In production these limits are filled from the URDF by the hardware plugin.
  config.joint_lower_limits_rad.assign(motor_ids.size(), -3.14);
  config.joint_upper_limits_rad.assign(motor_ids.size(), 3.14);
  config.joint_velocity_limits_rad_s.assign(motor_ids.size(), 0.17453292519943295);
  return config;
}

TEST(ActuatorCoordinates, RoundTripsProtocolAndOutputCoordinates)
{
  const pi_arm_hardware::ActuatorConfig actuator{"joint_04", 3U, 36.0, -1};
  pi_arm_hardware::validate_actuator_config(actuator);
  const double protocol =
    pi_arm_hardware::output_position_to_protocol_deg(0.25, actuator);
  EXPECT_LT(protocol, 0.0);
  EXPECT_NEAR(
    pi_arm_hardware::protocol_position_to_output_rad(protocol, actuator),
    0.25, 1.0e-12);
  EXPECT_NEAR(
    pi_arm_hardware::protocol_velocity_to_output_rad_s(
      pi_arm_hardware::output_velocity_to_protocol_dps(-0.4, actuator), actuator),
    -0.4, 1.0e-12);
}

TEST(MockBackend, FollowsPositionAndVelocityCommands)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  const std::vector<double> positions{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  const std::vector<double> velocities{0.0, -0.02, 0.03, 0.0, 0.05, 0.06};
  ASSERT_TRUE(backend.write(positions, velocities));

  ASSERT_TRUE(backend.read(sample));
  ASSERT_EQ(sample.positions_rad.size(), positions.size());
  ASSERT_EQ(sample.velocities_rad_s.size(), velocities.size());
  for (std::size_t index = 0; index < positions.size(); ++index) {
    EXPECT_NEAR(sample.positions_rad[index], positions[index], 1.0e-12);
    EXPECT_NEAR(sample.velocities_rad_s[index], velocities[index], 1.0e-12);
  }
  EXPECT_FALSE(sample.in_motion[0]);
  EXPECT_TRUE(sample.in_motion[1]);
}

TEST(MockBackend, ClampsVelocityAboveLimit)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  const std::vector<double> positions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> velocities{0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
  ASSERT_TRUE(backend.write(positions, velocities));

  ASSERT_TRUE(backend.read(sample));
  EXPECT_NEAR(sample.velocities_rad_s[0], 0.17453292519943295, 1e-9);
  EXPECT_TRUE(sample.in_motion[0]);
}

TEST(MockBackend, DeactivateClearsMotionAndDisconnects)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  const std::vector<double> positions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> velocities{0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
  ASSERT_TRUE(backend.write(positions, velocities));

  backend.deactivate();
  ASSERT_TRUE(backend.read(sample));
  EXPECT_FALSE(sample.connected);
  for (const double velocity : sample.velocities_rad_s) {
    EXPECT_DOUBLE_EQ(velocity, 0.0);
  }
  for (const bool moving : sample.in_motion) {
    EXPECT_FALSE(moving);
  }
  for (const bool fresh : sample.fresh) {
    EXPECT_FALSE(fresh);
  }
}

TEST(MockBackend, RejectsWholeCommandWhenMappedOutputExceedsLimit)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);

  // This is inside the deliberately broad test joint limit, but axis 3 maps
  // beyond motor_03's configured reducer-output limit.
  const std::vector<double> positions{0.0, 0.0, 3.0, 0.0, 0.0, 0.0};
  const std::vector<double> velocities(6U, 0.0);
  EXPECT_FALSE(backend.write(positions, velocities));
  ASSERT_TRUE(backend.read(sample));
  EXPECT_NE(sample.error_message.find("motor_03"), std::string::npos);
  for (const double position : sample.positions_rad) {
    EXPECT_DOUBLE_EQ(position, 0.0);
  }
}

TEST(MockBackend, SeparatesRawMotorAndJointDependencyState)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 4).success);
  ASSERT_TRUE(backend.read(sample));
  EXPECT_TRUE(sample.motor_enabled[3]);
  EXPECT_FALSE(sample.enabled[3]);  // joint_05 also depends on motor 2.
  EXPECT_EQ(sample.driver_mode, "MONITORING");

  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  ASSERT_TRUE(backend.read(sample));
  EXPECT_TRUE(std::all_of(sample.motor_enabled.begin(), sample.motor_enabled.end(), [](bool v) {
    return v;
  }));
  EXPECT_TRUE(std::all_of(sample.enabled.begin(), sample.enabled.end(), [](bool v) {return v;}));
  EXPECT_EQ(sample.driver_mode, "CONTROL");

  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_OFF, 2).success);
  ASSERT_TRUE(backend.read(sample));
  const std::vector<bool> expected_motors{true, false, true, true, true, true};
  const std::vector<bool> expected_joints{true, false, false, false, false, false};
  EXPECT_EQ(sample.motor_enabled, expected_motors);
  EXPECT_EQ(sample.enabled, expected_joints);
  EXPECT_EQ(sample.driver_mode, "MONITORING");

  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_OFF, 0).success);
  ASSERT_TRUE(backend.read(sample));
  EXPECT_TRUE(std::none_of(sample.motor_enabled.begin(), sample.motor_enabled.end(), [](bool v) {
    return v;
  }));

  EXPECT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_OFF, 2).success);
  EXPECT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 2).success);
  EXPECT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 2).success);
  EXPECT_FALSE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_OFF, 7).success);
  EXPECT_FALSE(
    backend.manage(
      pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO, 4).success);
}

}  // namespace
