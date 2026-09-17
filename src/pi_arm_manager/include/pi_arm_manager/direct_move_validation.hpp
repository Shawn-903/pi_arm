#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace pi_arm_manager
{

// Filled at startup from the robot model (URDF position/velocity limits plus
// robot_description_planning acceleration limits) so the robot description
// stays the single source of truth.
struct JointLimits
{
  double lower{0.0};
  double upper{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double feedback_tolerance{0.0};
};

// A slow full-arm move can legitimately exceed the default result timeout.
inline double direct_result_timeout(
  double trajectory_seconds, double default_timeout, double completion_margin)
{
  if (!std::isfinite(trajectory_seconds) || trajectory_seconds < 0.0 ||
    !std::isfinite(default_timeout) || default_timeout <= 0.0 ||
    !std::isfinite(completion_margin) || completion_margin <= 0.0 ||
    !std::isfinite(trajectory_seconds + completion_margin))
  {throw std::invalid_argument("invalid direct_move result timeout");}
  return std::max(default_timeout, trajectory_seconds + completion_margin);
}

inline void validate_direct_positions(
  const std::map<std::string, JointLimits> & joint_limits,
  const std::vector<std::string> & names, const std::vector<double> & positions,
  const char * stage, bool feedback = false)
{
  if (names.size() != joint_limits.size() || positions.size() != names.size()) {
    throw std::runtime_error("direct_move requires complete joint feedback");
  }
  for (size_t i = 0; i < names.size(); ++i) {
    const auto found = joint_limits.find(names[i]);
    if (found == joint_limits.end() ||
      std::find(names.begin(), names.begin() + i, names[i]) != names.begin() + i)
    {
      throw std::runtime_error("direct_move contains unknown or duplicate joint");
    }
    const auto & bounds = found->second;
    const double tolerance = feedback ? bounds.feedback_tolerance : 0.0;
    if (!std::isfinite(positions[i]) || positions[i] < bounds.lower - tolerance ||
      positions[i] > bounds.upper + tolerance)
    {
      std::ostringstream message;
      message.precision(17);
      message << "direct_move " << stage << " outside limits: " << names[i]
              << " position=" << positions[i] << " limits=[" << bounds.lower
              << "," << bounds.upper << "]";
      throw std::runtime_error(message.str());
    }
  }
}

}  // namespace pi_arm_manager
