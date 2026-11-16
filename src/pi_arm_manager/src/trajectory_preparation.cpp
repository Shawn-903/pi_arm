#include "trajectory_preparation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "rclcpp/duration.hpp"

namespace pi_arm_manager
{

void validate_timed_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.joint_names.empty() || trajectory.points.empty()) {
    throw std::runtime_error("planned trajectory is empty");
  }
  double previous_time = -1.0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const double t = rclcpp::Duration(trajectory.points[i].time_from_start).seconds();
    if (!std::isfinite(t) || t + 1e-9 < previous_time) {
      throw std::runtime_error(
              "planned trajectory has invalid time_from_start at index " + std::to_string(i));
    }
    if (trajectory.points[i].positions.size() != trajectory.joint_names.size()) {
      throw std::runtime_error(
              "planned trajectory point/joint size mismatch at index " + std::to_string(i));
    }
    previous_time = t;
  }
}

void retime_trajectory(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  std::vector<double> previous,
  const std::map<std::string, JointLimits> & joint_limits,
  const pi_arm_transmission::CableKinematics & cable,
  const pi_arm_transmission::Vector6 & protocol_ratios,
  double output_speed)
{
  validate_direct_positions(joint_limits, trajectory.joint_names, previous, "trajectory start", true);
  pi_arm_transmission::Matrix6 gain{};
  cable.jacobian({}, gain);
  double old_time = 0.0, new_time = 0.0;
  for (auto & point : trajectory.points) {
    validate_direct_positions(joint_limits, trajectory.joint_names, point.positions, "trajectory target");
    pi_arm_transmission::Vector6 target{}, output{};
    std::copy(point.positions.begin(), point.positions.end(), target.begin());
    if (!cable.joint_to_output_position(target, output) ||
      !cable.check_output_position_limits(output)) {
      throw std::runtime_error("trajectory output limit exceeded");
    }
    const double time = rclcpp::Duration(point.time_from_start).seconds();
    double duration = std::max(0.001, time - old_time);
    for (size_t i = 0; i < 6; ++i) {
      double distance_bound = 0.0;
      for (size_t j = 0; j < 6; ++j) {
        distance_bound += std::abs(gain[i][j]) * std::abs(point.positions[j] - previous[j]);
      }
      const double encoded_speed = std::min(65535.0, std::floor(
          output_speed * 180.0 / M_PI * protocol_ratios[i] + 1e-10));
      if (encoded_speed < 1.0) {throw std::runtime_error("output speed below protocol resolution");}
      const double effective_speed = encoded_speed / protocol_ratios[i] * M_PI / 180.0;
      duration = std::max(duration, distance_bound / effective_speed);
      duration = std::max(duration, std::abs(point.positions[i] - previous[i]) /
        joint_limits.at(trajectory.joint_names[i]).velocity);
    }
    // Position-only waypoints select linear JTC interpolation: no spline
    // overshoot into the feedback allowance. |J(0)| bounds cable speed on
    // each linear segment, including the chord axes.
    point.velocities.clear();
    point.accelerations.clear();
    point.effort.clear();
    new_time += duration;
    if (!std::isfinite(new_time)) {throw std::runtime_error("invalid trajectory duration");}
    point.time_from_start = rclcpp::Duration::from_seconds(new_time);
    old_time = time;
    previous = point.positions;
  }
}

}  // namespace pi_arm_manager
