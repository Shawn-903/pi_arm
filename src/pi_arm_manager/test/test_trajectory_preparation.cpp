#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "rclcpp/duration.hpp"
#include "trajectory_preparation.hpp"

namespace
{

class TrajectoryPreparation : public ::testing::Test
{
protected:
  TrajectoryPreparation()
  {
    parameters.axis1 = {1.0, 2.0};
    parameters.axis2 = {1.0, 1.0};
    parameters.axis3 = {1.0, 1.0, 1.0, 1.0, 1};
    parameters.axis4 = {1.0, 1.0, 1.0, 1};
    parameters.axis5 = {1.0, 1.0, 1.0, 1};
    parameters.axis6_branch_a = {1.0, 1.0, 1.0, 1};
    parameters.axis6_branch_b = parameters.axis6_branch_a;
    parameters.output_limits.fill({-10.0, 10.0});
    trajectory.joint_names = {
      "joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"};
    for (const auto & name : trajectory.joint_names) {
      limits[name] = {-1.0, 1.0, 1.0, 1.0, 0.05};
    }
  }

  void add_point(std::vector<double> positions, double seconds)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = std::move(positions);
    point.velocities.assign(6, 0.1);
    point.accelerations.assign(6, 0.2);
    point.effort.assign(6, 0.3);
    point.time_from_start = rclcpp::Duration::from_seconds(seconds);
    trajectory.points.push_back(point);
  }

  void prepare(double output_speed = 0.1)
  {
    pi_arm_manager::validate_timed_trajectory(trajectory);
    pi_arm_manager::retime_trajectory(
      trajectory, std::vector<double>(6, 0.0), limits,
      pi_arm_transmission::CableKinematics(parameters), ratios, output_speed);
  }

  double seconds(std::size_t index) const
  {
    return rclcpp::Duration(trajectory.points.at(index).time_from_start).seconds();
  }

  pi_arm_transmission::CableKinematicsParameters parameters;
  pi_arm_transmission::Vector6 ratios{36.0, 36.0, 36.0, 6.0, 6.0, 6.0};
  std::map<std::string, pi_arm_manager::JointLimits> limits;
  trajectory_msgs::msg::JointTrajectory trajectory;
};

TEST_F(TrajectoryPreparation, PreservesSlowerTimingAndSelectsPositionOnlyInterpolation)
{
  add_point({0.1, 0.0, 0.0, 0.0, 0.0, 0.0}, 10.0);
  add_point({0.2, 0.0, 0.0, 0.0, 0.0, 0.0}, 20.0);
  const auto original = trajectory;
  prepare();
  EXPECT_DOUBLE_EQ(seconds(0), 10.0);
  EXPECT_DOUBLE_EQ(seconds(1), 20.0);
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i];
    EXPECT_EQ(point.positions, original.points[i].positions);
    EXPECT_TRUE(point.velocities.empty());
    EXPECT_TRUE(point.accelerations.empty());
    EXPECT_TRUE(point.effort.empty());
  }
}

TEST_F(TrajectoryPreparation, CoupledWristSpeedUsesRoundedDownProtocolCeiling)
{
  // Only J2 moves, but its 1:1 coupling also drives the wrist outputs.
  // Their ratio of 6 rounds a requested 1.1 deg/s down to 1 deg/s:
  // a 10-degree output travel must therefore take at least 10 seconds.
  add_point({0.0, -10.0 * M_PI / 180.0, 0.0, 0.0, 0.0, 0.0}, 0.1);
  prepare(1.1 * M_PI / 180.0);
  EXPECT_NEAR(seconds(0), 10.0, 1e-8);
}

TEST_F(TrajectoryPreparation, JointVelocityLimitCanRequireLongerTravel)
{
  limits.at("joint_01").velocity = 0.02;
  add_point({0.1, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.1);
  prepare();
  EXPECT_NEAR(seconds(0), 5.0, 1e-8);
}

TEST_F(TrajectoryPreparation, RejectsUnrepresentableSpeedAndOutOfRangeOutput)
{
  add_point({0.1, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.1);
  EXPECT_THROW(prepare(0.001), std::runtime_error);
  parameters.output_limits[0] = {-0.01, 0.01};
  EXPECT_THROW(prepare(), std::runtime_error);
}

TEST_F(TrajectoryPreparation, RejectsEmptyMalformedAndBackwardsTrajectories)
{
  EXPECT_THROW(prepare(), std::runtime_error);
  add_point({0.0, 0.0, 0.0, 0.0, 0.0}, 0.1);
  EXPECT_THROW(prepare(), std::runtime_error);
  trajectory.points.front().positions.push_back(0.0);
  add_point(std::vector<double>(6, 0.0), 0.05);
  EXPECT_THROW(prepare(), std::runtime_error);
}

}  // namespace
