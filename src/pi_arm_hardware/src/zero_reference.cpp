#include "pi_arm_hardware/backend.hpp"
#include "pi_arm_hardware/software_zero.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <thread>

namespace pi_arm_hardware
{
ManagementResult RealBackend::set_zero_reference(
  const CalibrationVector6 & reference_joint_rad, const bool reference_pose_confirmed,
  HardwareSample & applied_sample)
{
  if (!config_.commissioning || config_.read_only || !reference_pose_confirmed) {
    return {false, 2001, "software zero requires commissioning mode and a confirmed physical reference pose"};
  }
  if (axis_test_active_ || motion_write_gate_.armed() || teaching_active_.load() ||
    axis_test_stop_failed_ || stop_hold_failed_.load())
  {
    return {false, 2003, "stop the active task and confirm holding before setting software zero"};
  }
  if (config_.software_zero_file.empty()) {
    return {false, 2001, "configure an absolute software_zero_file for persistent zero references"};
  }
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(reference_joint_rad[i]) ||
      reference_joint_rad[i] < config_.joint_lower_limits_rad.at(i) ||
      reference_joint_rad[i] > config_.joint_upper_limits_rad.at(i))
    {return {false, 1004, "reference pose must be finite and within the joint limits"};}
  }
  try {
    const auto output_path = std::filesystem::weakly_canonical(config_.software_zero_file);
    if (output_path == std::filesystem::weakly_canonical(config_.cable_kinematics_file)) {
      return {false, 1004, "software zero must not overwrite cable geometry"};
    }
    CalibrationResult capture_base;
    if (!config_.calibration_file.empty() &&
      output_path != std::filesystem::weakly_canonical(config_.calibration_file))
    {
      // Only a separately selected packing file supplies a preset for this capture.
      capture_base = load_pack_reference_calibration(config_.calibration_file,
        config_.cable_kinematics_file, config_.actuators, kinematics_);
      if (!capture_base.valid) {return {false, 2001, capture_base.message};}
    }
    const auto fingerprint = calibration_configuration_fingerprint(
      config_.cable_kinematics_file, config_.actuators);

    using Clock = std::chrono::steady_clock;
    constexpr int sample_count = 6;
    if (!driver_) {return {false, 2001, "CAN driver is unavailable"};}
    const auto generation = driver_->connection_generation();
    std::array<Clock::time_point, kCalibrationAxisCount> previous;
    previous.fill(Clock::now());
    CalibrationVector6 first{}, protocol{};
    for (int sample = 0; sample < sample_count; ++sample) {
      const auto deadline = Clock::now() + std::chrono::milliseconds(300);
      MotorSnapshot snapshot;
      while (true) {
        if (!axis_test_raw_ready(true, &snapshot) || driver_->connection_generation() != generation) {
          return {false, 2001, "software zero requires six fresh, stationary, error-free motors"};
        }
        bool all_updated = true;
        for (std::size_t i = 0; i < previous.size(); ++i) {
          const auto & motor = snapshot.at(config_.actuators[i].motor_id);
          if (sample > 0 && std::abs(motor.protocol_angle_deg - first[i]) > 0.02 + 1e-9) {
            return {false, 2001, "encoder position changed while capturing software zero"};
          }
          if (motor.angle_update <= previous[i]) {all_updated = false;}
        }
        if (all_updated) {break;}
        if (Clock::now() >= deadline) {
          return {false, 2001, "timed out waiting for new encoder feedback on all six motors"};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      for (std::size_t i = 0; i < previous.size(); ++i) {
        const auto & motor = snapshot.at(config_.actuators[i].motor_id);
        previous[i] = motor.angle_update;
        if (sample == 0) {first[i] = motor.protocol_angle_deg;}
        protocol[i] += motor.protocol_angle_deg / sample_count;
      }
    }
    const auto time = std::time(nullptr);
    std::tm utc{};
    ::gmtime_r(&time, &utc);
    std::ostringstream stamp;
    stamp << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    const auto * packing = capture_base.valid && capture_base.data.approved ? &capture_base : nullptr;
    auto next = capture_pose_zero(config_.software_zero_file, fingerprint, reference_joint_rad,
      protocol, config_.actuators, kinematics_, stamp.str(), packing);
    // No CAN writes, and no concurrent controller exists in commissioning mode.
    calibration_ = std::move(next);
    for (auto & detector : joint_motion_detectors_) {detector.reset();}
    position_initialized_ = false;
    applied_sample.raw_protocol_positions_deg.assign(protocol.begin(), protocol.end());
    applied_sample.software_output_offsets_rad.assign(
      calibration_.software_output_offsets_rad.begin(), calibration_.software_output_offsets_rad.end());
    return {true, 0, "software zero saved and applied; motor ROM and physical targets unchanged"};
  } catch (const std::exception & error) {
    return {false, 2002, error.what()};
  }
}
}  // namespace pi_arm_hardware
