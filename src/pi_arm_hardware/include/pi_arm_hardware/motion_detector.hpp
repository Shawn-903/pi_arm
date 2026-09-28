#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

namespace pi_arm_hardware
{

// Feedback-only motion classification. Raw position/velocity, command limits,
// stop ACKs and the driver's management/zeroing interlocks are not filtered.
// Times must be monotonic sensor receipt times in seconds, not read-loop times.
class MotionDetector
{
public:
  static constexpr double position_window_sec = 0.5;
  static constexpr double position_confirm_sec = 0.04;
  static constexpr double speed_confirm_sec = 0.15;
  static constexpr double settle_sec = 0.3;
  static constexpr double max_sample_gap_sec = 0.25;

  void reset() { *this = MotionDetector{}; }

  bool update(
    double position, double position_time, double speed, double speed_time,
    double now, bool fresh, double position_deadband, double speed_threshold)
  {
    if (!fresh || !std::isfinite(position) || !std::isfinite(speed) ||
      !std::isfinite(position_time) || !std::isfinite(speed_time) || !std::isfinite(now) ||
      !std::isfinite(position_deadband) || position_deadband < 0.0 ||
      !std::isfinite(speed_threshold) || speed_threshold < 0.0 ||
      position_time > now || speed_time > now ||
      now - position_time > max_sample_gap_sec || now - speed_time > max_sample_gap_sec)
    {
      reset();
      return true;  // Unknown cannot establish stationary feedback.
    }
    if (initialized_ && (position_time < position_time_ || speed_time < speed_time_ ||
      position_time - position_time_ > max_sample_gap_sec ||
      speed_time - speed_time_ > max_sample_gap_sec))
    {
      reset();
    }
    if (!initialized_) {
      initialized_ = true;
      position_time_ = position_time;
      speed_time_ = speed_time;
      last_motion_time_ = std::max(position_time, speed_time);
      history_.push_back({position_time, position});
      speed_evidence_.update(speed_sign(speed, speed_threshold), speed_time, speed_confirm_sec);
      return true;  // Require fresh observations before first reporting still.
    }

    bool confirmed = false;
    if (position_time > position_time_) {
      position_time_ = position_time;
      // Retain one reference at/before the window boundary.
      while (history_.size() > 1 &&
        history_[1].time <= position_time - position_window_sec)
      {
        history_.pop_front();
      }
      const double elapsed = position_time - history_.front().time;
      history_.push_back({position_time, position});
      // A net first-to-last displacement can miss oscillation with the same
      // period as the window. Use its span, discarding one extreme at either
      // end so a single bad position sample cannot establish motion.
      double low = std::numeric_limits<double>::infinity(), second_low = low;
      double high = -low, second_high = high;
      for (const auto & observation : history_) {
        const double value = observation.position;
        if (value <= low) {second_low = low; low = value;}
        else if (value < second_low) {second_low = value;}
        if (value >= high) {second_high = high; high = value;}
        else if (value > second_high) {second_high = value;}
      }
      const double displacement = history_.size() >= 4 ? second_high - second_low : 0.0;
      const double threshold = std::max(position_deadband, speed_threshold * elapsed);
      confirmed = position_evidence_.update(
        speed_sign(displacement, threshold), position_time, position_confirm_sec);
    }
    if (speed_time > speed_time_) {
      speed_time_ = speed_time;
      confirmed = speed_evidence_.update(
        speed_sign(speed, speed_threshold), speed_time, speed_confirm_sec) || confirmed;
    }
    if (confirmed) {
      moving_ = true;
      last_motion_time_ = std::max(position_time, speed_time);
    } else if (std::min(position_time, speed_time) - last_motion_time_ >= settle_sec) {
      moving_ = false;
    }
    return moving_;
  }

private:
  static int speed_sign(double value, double threshold)
  {
    // Avoid a floating-point roundoff turning exactly two encoder ticks into
    // displacement evidence. This epsilon is far below encoder resolution.
    return std::abs(value) > threshold + 1e-12 ? (value > 0.0 ? 1 : -1) : 0;
  }

  struct Evidence
  {
    int sign{0};
    double since{0.0};
    bool update(int direction, double time, double duration)
    {
      if (direction != sign) {sign = direction; since = time;}
      return sign != 0 && time - since + 1e-12 >= duration;
    }
  };
  struct Observation {double time; double position;};
  std::deque<Observation> history_;
  Evidence position_evidence_, speed_evidence_;
  bool initialized_{false};
  bool moving_{true};
  double position_time_{0.0}, speed_time_{0.0}, last_motion_time_{0.0};
};

}  // namespace pi_arm_hardware
