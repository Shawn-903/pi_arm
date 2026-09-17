#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace pi_arm_manager
{
struct ArrivalCheck
{
  bool valid{false};
  bool reached{false};
  std::vector<double> errors;
};

inline ArrivalCheck check_arrival(
  const std::vector<std::string> & target_names, const std::vector<double> & targets,
  const std::vector<std::string> & names, const std::vector<double> & positions,
  const std::vector<double> & velocities, bool fresh, double position_tolerance,
  double velocity_tolerance)
{
  ArrivalCheck result;
  if (!fresh || target_names.empty() || names != target_names || targets.size() != names.size() ||
    positions.size() != names.size() || velocities.size() != names.size() ||
    !std::isfinite(position_tolerance) || position_tolerance <= 0.0 ||
    !std::isfinite(velocity_tolerance) || velocity_tolerance < 0.0) {return result;}
  result.reached = true;
  for (size_t i = 0; i < names.size(); ++i) {
    for (size_t j = 0; j < i; ++j) {if (names[i] == names[j]) {return {};}}
    if (!std::isfinite(targets[i]) || !std::isfinite(positions[i]) ||
      !std::isfinite(velocities[i])) {return {};}
    const double error = targets[i] - positions[i];
    result.errors.push_back(error);
    result.reached = result.reached && std::abs(error) <= position_tolerance &&
      std::abs(velocities[i]) <= velocity_tolerance;
  }
  result.valid = true;
  return result;
}

// A repeated cached sample cannot establish a stable arrival interval.
class ArrivalWindow
{
public:
  explicit ArrivalWindow(double seconds) : seconds_(seconds) {}
  bool update(bool reached, double time, std::uint64_t sequence)
  {
    if (!reached || !std::isfinite(time)) {started_ = false; return false;}
    if (sequence == sequence_) {return false;}
    sequence_ = sequence;
    if (!started_) {start_ = time; started_ = true;}
    return time - start_ >= seconds_;
  }
private:
  double seconds_;
  double start_{0.0};
  std::uint64_t sequence_{0};
  bool started_{false};
};
}  // namespace pi_arm_manager
