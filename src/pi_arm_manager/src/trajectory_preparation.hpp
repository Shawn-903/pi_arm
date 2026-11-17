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

// 调用前由节点校验时间和关节顺序，并取得起点 previous（rad）。这里按模型限位
// 和输出轴速度上限 output_speed（rad/s）调整轨迹时间，只使用传入快照。
// 时间戳、锁、运动租约和控制器提交仍由节点管理。
void retime_trajectory(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  std::vector<double> previous,
  const std::map<std::string, JointLimits> & joint_limits,
  const pi_arm_transmission::CableKinematics & cable,
  const pi_arm_transmission::Vector6 & protocol_ratios,
  double output_speed);

}  // namespace pi_arm_manager
