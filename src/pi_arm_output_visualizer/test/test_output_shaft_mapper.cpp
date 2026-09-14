#include "pi_arm_output_visualizer/output_shaft_mapper.hpp"

#include "gtest/gtest.h"
#include "pi_arm_transmission/cable_kinematics.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <vector>

namespace
{

pi_arm_transmission::CableKinematicsParameters make_parameters()
{
  pi_arm_transmission::CableKinematicsParameters parameters;
  parameters.axis1 = {1.0, 2.0};
  parameters.axis2 = {1.0, 1.0};
  parameters.axis3 = {1.0, 2.0, 3.0, 1.0, 1};
  parameters.axis4 = {1.0, 2.0, 1.0, 1};
  parameters.axis5 = {1.0, 3.0, 1.0, -1};
  parameters.axis6_branch_a = {1.0, 1.0, 1.0, 1};
  parameters.axis6_branch_b = {2.0, 2.0, 2.0, 1};
  parameters.output_limits.fill({-100.0, 100.0});
  return parameters;
}

sensor_msgs::msg::JointState make_reordered_input(
  const pi_arm_transmission::Vector6 & output_position,
  const pi_arm_transmission::Vector6 * output_velocity = nullptr)
{
  sensor_msgs::msg::JointState message;
  const auto & names = pi_arm_output_visualizer::OutputShaftMapper::output_shaft_names();
  for (std::size_t reverse_index = 0; reverse_index < names.size(); ++reverse_index) {
    const std::size_t axis = names.size() - reverse_index - 1U;
    message.name.push_back(names[axis]);
    message.position.push_back(output_position[axis]);
    if (output_velocity != nullptr) {
      message.velocity.push_back((*output_velocity)[axis]);
    }
  }
  return message;
}

}  // namespace

TEST(OutputShaftMapper, RecoversActiveJointPositionsIndependentOfInputOrder)
{
  const auto parameters = make_parameters();
  const pi_arm_transmission::CableKinematics kinematics(parameters);
  const pi_arm_transmission::Vector6 expected_joint{0.1, 0.2, 0.3, 0.4, -0.3, 0.6};
  pi_arm_transmission::Vector6 output_position{};
  ASSERT_TRUE(kinematics.joint_to_output_position(expected_joint, output_position));

  const pi_arm_output_visualizer::OutputShaftMapper mapper(parameters);
  const auto input = make_reordered_input(output_position);
  sensor_msgs::msg::JointState output;
  std::string error;
  ASSERT_TRUE(mapper.map(input, output, error)) << error;

  EXPECT_EQ(
    output.name,
    std::vector<std::string>(
      pi_arm_output_visualizer::OutputShaftMapper::active_joint_names().begin(),
      pi_arm_output_visualizer::OutputShaftMapper::active_joint_names().end()));
  ASSERT_EQ(output.position.size(), expected_joint.size());
  for (std::size_t axis = 0; axis < expected_joint.size(); ++axis) {
    EXPECT_NEAR(output.position[axis], expected_joint[axis], 1.0e-12);
  }
  EXPECT_TRUE(output.velocity.empty());
}

TEST(OutputShaftMapper, ConvertsOutputShaftVelocityAtRecoveredPosition)
{
  const auto parameters = make_parameters();
  const pi_arm_transmission::CableKinematics kinematics(parameters);
  const pi_arm_transmission::Vector6 joint_position{0.1, 0.2, 0.3, 0.4, -0.3, 0.6};
  const pi_arm_transmission::Vector6 expected_joint_velocity{
    0.01, -0.02, 0.03, -0.04, 0.05, -0.06};
  pi_arm_transmission::Vector6 output_position{};
  pi_arm_transmission::Vector6 output_velocity{};
  ASSERT_TRUE(kinematics.joint_to_output_position(joint_position, output_position));
  ASSERT_TRUE(kinematics.joint_to_output_velocity(
    joint_position, expected_joint_velocity, output_velocity));

  const pi_arm_output_visualizer::OutputShaftMapper mapper(parameters);
  const auto input = make_reordered_input(output_position, &output_velocity);
  sensor_msgs::msg::JointState output;
  std::string error;
  ASSERT_TRUE(mapper.map(input, output, error)) << error;

  ASSERT_EQ(output.velocity.size(), expected_joint_velocity.size());
  for (std::size_t axis = 0; axis < expected_joint_velocity.size(); ++axis) {
    EXPECT_NEAR(output.velocity[axis], expected_joint_velocity[axis], 1.0e-12);
  }
}

TEST(OutputShaftMapper, RejectsMissingRequiredOutputShaft)
{
  const pi_arm_output_visualizer::OutputShaftMapper mapper(make_parameters());
  sensor_msgs::msg::JointState input;
  input.name = {"motor_01_output", "motor_02_output"};
  input.position = {0.0, 0.0};
  sensor_msgs::msg::JointState output;
  std::string error;

  EXPECT_FALSE(mapper.map(input, output, error));
  EXPECT_NE(error.find("motor_03_output"), std::string::npos);
}

TEST(OutputShaftMapper, RejectsConfiguredOutputShaftLimitViolation)
{
  auto parameters = make_parameters();
  parameters.output_limits[4] = {-0.5, 0.5};
  const pi_arm_output_visualizer::OutputShaftMapper mapper(parameters);
  pi_arm_transmission::Vector6 output_position{};
  output_position[4] = 0.6;
  const auto input = make_reordered_input(output_position);
  sensor_msgs::msg::JointState output;
  std::string error;

  EXPECT_FALSE(mapper.map(input, output, error));
  EXPECT_NE(error.find("OUTPUT_POSITION_LIMIT"), std::string::npos);
  EXPECT_NE(error.find("axis 5"), std::string::npos);
}
