#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <vector>

namespace pi_arm_hardware
{

// 运动租约只授予当前任务写入资格。取得租约后，还要等控制器命令的
// 位置或速度与空闲时缓存的基线不同，才允许第一次真实位置写入。
// 这样，使能电机或取得租约不会把控制器残留的保持目标当成新运动。
class MotionWriteGate
{
public:
  void observe_idle(
    const std::vector<double> & positions, const std::vector<double> & velocities)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (armed_ || positions.empty() || positions.size() != velocities.size() ||
      !all_finite(positions) || !all_finite(velocities))
    {
      return;
    }
    baseline_positions_ = positions;
    baseline_velocities_ = velocities;
  }

  bool arm(const std::uint64_t lease_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (lease_id == 0U || baseline_positions_.empty() ||
      baseline_positions_.size() != baseline_velocities_.size())
    {
      return false;
    }
    if (armed_) {
      return active_lease_id_ == lease_id;
    }
    active_lease_id_ = lease_id;
    fresh_command_seen_ = false;
    armed_ = true;
    return true;
  }

  bool disarm(const std::uint64_t lease_id = 0U)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!armed_) {
      return true;
    }
    // 非零 ID 只能释放自己的租约，防止旧任务清理时撤销新任务；0 表示强制撤销。
    if (lease_id != 0U && lease_id != active_lease_id_) {
      return false;
    }
    armed_ = false;
    fresh_command_seen_ = false;
    active_lease_id_ = 0U;
    return true;
  }

  bool permit_write(
    const std::vector<double> & positions, const std::vector<double> & velocities)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!armed_ || positions.size() != baseline_positions_.size() ||
      velocities.size() != baseline_velocities_.size())
    {
      return false;
    }
    if (!all_finite(positions) || !all_finite(velocities))
    {
      return false;
    }
    if (!fresh_command_seen_) {
      const bool position_changed = !std::equal(
        positions.begin(), positions.end(), baseline_positions_.begin());
      const bool velocity_changed = !std::equal(
        velocities.begin(), velocities.end(), baseline_velocities_.begin());
      if (!position_changed && !velocity_changed) {
        return false;
      }
      fresh_command_seen_ = true;
    }
    return true;
  }

  bool armed() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return armed_;
  }

  std::uint64_t active_lease_id() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_lease_id_;
  }

private:
  static bool all_finite(const std::vector<double> & values)
  {
    return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
  }

  mutable std::mutex mutex_;
  bool armed_{false};
  bool fresh_command_seen_{false};
  std::uint64_t active_lease_id_{0U};
  std::vector<double> baseline_positions_;
  std::vector<double> baseline_velocities_;
};

}  // namespace pi_arm_hardware
