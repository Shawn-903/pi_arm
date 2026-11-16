#pragma once

#include <map>
#include <string>
#include <vector>

#include "pi_arm_manager/direct_move_validation.hpp"
#include "pi_arm_transmission/cable_kinematics.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace pi_arm_manager
{

void validate_timed_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory);

// The node validates timing and canonical joint order before taking this start
// snapshot. This calculation uses only that snapshot and immutable model limits;
// the caller retains clock stamping, locks, leases, and controller submission.
void retime_trajectory(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  std::vector<double> previous,
  const std::map<std::string, JointLimits> & joint_limits,
  const pi_arm_transmission::CableKinematics & cable,
  const pi_arm_transmission::Vector6 & protocol_ratios,
  double output_speed);

}  // namespace pi_arm_manager
