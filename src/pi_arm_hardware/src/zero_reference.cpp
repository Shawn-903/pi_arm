#include "pi_arm_hardware/backend.hpp"
#include "pi_arm_hardware/software_zero.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
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
    if (!config_.calibration_file.empty() &&
      output_path != std::filesystem::weakly_canonical(config_.calibration_file) &&
      !calibration_.valid)
    {return {false, 2001, calibration_.message};}
    const auto fingerprint = calibration_configuration_fingerprint(
      config_.cable_kinematics_file, config_.actuators);

    CalibrationVector6 first{}, protocol{};
    for (int sample = 0; sample < 6; ++sample) {
      if (sample != 0) {std::this_thread::sleep_for(std::chrono::milliseconds(20));}
      if (!axis_test_raw_ready(true)) {
        return {false, 2001, "software zero requires six fresh, stationary, error-free motors"};
      }
      const auto snapshot = driver_->snapshot();
      for (std::size_t i = 0; i < 6; ++i) {
        const auto value = snapshot.at(config_.actuators[i].motor_id).protocol_angle_deg;
        if (sample == 0) {first[i] = value;}
        if (std::abs(value - first[i]) > 0.02 + 1e-9) {
          return {false, 2001, "encoder position changed while capturing software zero"};
        }
        protocol[i] += value / 6.0;
      }
    }
    std::string reference_id;
    std::ifstream uuid("/proc/sys/kernel/random/uuid");
    if (!(uuid >> reference_id)) {throw std::runtime_error("cannot generate reference ID");}
    const auto time = std::time(nullptr);
    std::tm utc{};
    ::gmtime_r(&time, &utc);
    std::ostringstream stamp;
    stamp << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    const auto * packing = calibration_.valid && calibration_.data.approved ? &calibration_ : nullptr;
    auto next = capture_pose_zero(config_.software_zero_file, fingerprint, reference_joint_rad,
      protocol, config_.actuators, kinematics_, reference_id, stamp.str(), packing);
    // No CAN writes, and no concurrent controller exists in commissioning mode.
    calibration_ = std::move(next);
    for (auto & detector : joint_motion_detectors_) {detector.reset();}
    position_initialized_ = false;
    applied_sample.software_zero_id = calibration_.software_zero_id;
    applied_sample.raw_protocol_positions_deg.assign(protocol.begin(), protocol.end());
    applied_sample.software_output_offsets_rad.assign(
      calibration_.software_output_offsets_rad.begin(), calibration_.software_output_offsets_rad.end());
    return {true, 0, "software zero saved and applied; motor ROM and physical targets unchanged"};
  } catch (const std::exception & error) {
    return {false, 2002, error.what()};
  }
}
}  // namespace pi_arm_hardware
