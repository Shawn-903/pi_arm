#include "pi_arm_hardware/backend.hpp"
#include "backend_common.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "pi_arm_transmission/config_loader.hpp"

namespace pi_arm_hardware
{
using detail::clamp_joint_velocity;
using detail::joint_depends_on_motor;
using detail::kAxisCount;
using detail::log_write_reject;
using detail::to_vector;
using detail::validate_backend_config;

namespace
{

void aggregate_mock_motor_state_to_joints(HardwareSample & sample, const bool swap_wrist_channels)
{
  if (sample.motor_enabled.size() != kAxisCount ||
    sample.motor_has_error.size() != kAxisCount ||
    sample.motor_fresh.size() != kAxisCount ||
    sample.motor_error_codes.size() != kAxisCount ||
    sample.motor_ages_sec.size() != kAxisCount)
  {
    return;
  }
  for (std::size_t joint_index = 0; joint_index < kAxisCount; ++joint_index) {
    bool enabled = true;
    bool has_error = false;
    bool fresh = true;
    std::int32_t error_code = 0;
    double age = 0.0;
    for (std::size_t motor_index = 0; motor_index < kAxisCount; ++motor_index) {
      if (!joint_depends_on_motor(joint_index, motor_index, swap_wrist_channels)) {
        continue;
      }
      enabled = enabled && sample.motor_enabled[motor_index];
      has_error = has_error || sample.motor_has_error[motor_index];
      fresh = fresh && sample.motor_fresh[motor_index];
      error_code |= sample.motor_error_codes[motor_index];
      age = std::max(age, sample.motor_ages_sec[motor_index]);
    }
    sample.enabled[joint_index] = enabled;
    sample.has_error[joint_index] = has_error;
    sample.fresh[joint_index] = fresh;
    sample.error_codes[joint_index] = error_code;
    sample.ages_sec[joint_index] = age;
  }
}

}  // namespace

MockBackend::MockBackend(BackendConfig config)
: config_(std::move(config)),
  kinematics_(
    pi_arm_transmission::load_cable_kinematics_config(config_.cable_kinematics_file))
{
  validate_backend_config(config_);
  sample_.positions_rad.assign(kAxisCount, 0.0);
  sample_.velocities_rad_s.assign(kAxisCount, 0.0);
  sample_.enabled.assign(kAxisCount, false);
  sample_.has_error.assign(kAxisCount, false);
  sample_.in_motion.assign(kAxisCount, false);
  sample_.fresh.assign(kAxisCount, true);
  sample_.error_codes.assign(kAxisCount, 0);
  sample_.ages_sec.assign(kAxisCount, 0.0);
  sample_.motor_enabled.assign(kAxisCount, false);
  sample_.motor_has_error.assign(kAxisCount, false);
  sample_.motor_in_motion.assign(kAxisCount, false);
  sample_.motor_fresh.assign(kAxisCount, true);
  sample_.motor_error_codes.assign(kAxisCount, 0);
  sample_.motor_ages_sec.assign(kAxisCount, 0.0);
  sample_.calibrated = true;
  sample_.kinematics_valid = true;
  sample_.control_permitted = false;
  sample_.calibration_id = "MOCK";
  sample_.raw_protocol_positions_deg.assign(kAxisCount, 0.0);
  sample_.raw_output_positions_rad.assign(kAxisCount, 0.0);
  sample_.software_output_offsets_rad.assign(kAxisCount, 0.0);
  sample_.model_output_positions_rad.assign(kAxisCount, 0.0);
  stationary_samples_.assign(kAxisCount, 0U);
  aggregate_mock_motor_state_to_joints(sample_, kinematics_.parameters().swap_wrist_channels);
  sample_.driver_mode = "OFFLINE";
}

void MockBackend::configure()
{
  std::lock_guard<std::mutex> lock(mutex_);
  configured_ = true;
  sample_.connected = true;
  sample_.driver_mode = "MONITORING";
}

void MockBackend::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {
    throw std::runtime_error("mock backend is not configured");
  }
  active_ = true;
  sample_.connected = true;
  std::fill(sample_.motor_fresh.begin(), sample_.motor_fresh.end(), true);
  aggregate_mock_motor_state_to_joints(sample_, kinematics_.parameters().swap_wrist_channels);
  const bool all_enabled = std::all_of(
    sample_.motor_enabled.begin(), sample_.motor_enabled.end(), [](bool value) {return value;});
  sample_.driver_mode = all_enabled ? "CONTROL" : "MONITORING";
  sample_.control_permitted = all_enabled;
}

void MockBackend::deactivate() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  sample_.connected = false;
  sample_.control_permitted = false;
  sample_.driver_mode = configured_ ? "MONITORING" : "OFFLINE";
  output_velocities_rad_s_ = {};
  std::fill(sample_.velocities_rad_s.begin(), sample_.velocities_rad_s.end(), 0.0);
  std::fill(sample_.in_motion.begin(), sample_.in_motion.end(), false);
  std::fill(sample_.motor_in_motion.begin(), sample_.motor_in_motion.end(), false);
  std::fill(sample_.motor_fresh.begin(), sample_.motor_fresh.end(), false);
  aggregate_mock_motor_state_to_joints(sample_, kinematics_.parameters().swap_wrist_channels);
}

void MockBackend::cleanup() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  configured_ = false;
  sample_.connected = false;
  sample_.control_permitted = false;
  sample_.driver_mode = "OFFLINE";
  std::fill(sample_.motor_enabled.begin(), sample_.motor_enabled.end(), false);
  std::fill(sample_.motor_fresh.begin(), sample_.motor_fresh.end(), false);
  aggregate_mock_motor_state_to_joints(sample_, kinematics_.parameters().swap_wrist_channels);
}

bool MockBackend::read(HardwareSample & sample) noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t index = 0; index < kAxisCount; ++index) {
    stationary_samples_[index] =
      std::abs(sample_.velocities_rad_s[index]) <= config_.moving_speed_threshold_rad_s ?
      std::min<std::size_t>(3U, stationary_samples_[index] + 1U) : 0U;
  }
  sample = sample_;
  return configured_;
}

bool MockBackend::write(
  const std::vector<double> & positions_rad,
  const std::vector<double> & velocities_rad_s) noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || positions_rad.size() != kAxisCount ||
    velocities_rad_s.size() != kAxisCount)
  {
    log_write_reject(
      "mock write rejected: inactive or size/config mismatch (active=%d pos=%zu)",
      active_ ? 1 : 0, positions_rad.size());
    return false;
  }

  if (hold_mock_current_) {return true;}
  pi_arm_transmission::Vector6 joint_positions{};
  pi_arm_transmission::Vector6 joint_velocities{};
  for (std::size_t index = 0; index < kAxisCount; ++index) {
    if (!std::isfinite(positions_rad[index]) || !std::isfinite(velocities_rad_s[index])) {
      return false;
    }
    if (positions_rad[index] < config_.joint_lower_limits_rad[index] ||
      positions_rad[index] > config_.joint_upper_limits_rad[index])
    {
      return false;
    }
    joint_positions[index] = positions_rad[index];
    joint_velocities[index] = clamp_joint_velocity(
      velocities_rad_s[index], config_.joint_velocity_limits_rad_s[index], index);
  }

  const bool all_enabled =
    std::all_of(
    sample_.motor_enabled.begin(), sample_.motor_enabled.end(), [](const bool value) {return value;});
  if (!all_enabled) {
    output_velocities_rad_s_ = {};
    std::fill(sample_.velocities_rad_s.begin(), sample_.velocities_rad_s.end(), 0.0);
    std::fill(sample_.in_motion.begin(), sample_.in_motion.end(), false);
    std::fill(sample_.motor_in_motion.begin(), sample_.motor_in_motion.end(), false);
    return true;
  }

  auto status = kinematics_.joint_to_output_position(joint_positions, output_positions_rad_);
  if (!status) {
    sample_.error_message = pi_arm_transmission::to_string(status.error);
    return false;
  }
  status = kinematics_.check_output_position_limits(output_positions_rad_);
  if (!status) {
    const std::size_t index = status.axis - 1U;
    const auto & limit = kinematics_.parameters().output_limits[index];
    sample_.error_message =
      "output-shaft limit exceeded at motor_0" + std::to_string(status.axis) +
      ": value=" + std::to_string(status.value) + " rad, limit=[" +
      std::to_string(limit.lower_position_rad) + "," +
      std::to_string(limit.upper_position_rad) + "] rad";
    return false;
  }
  status = kinematics_.joint_to_output_velocity(
    joint_positions, joint_velocities, output_velocities_rad_s_);
  if (!status) {
    sample_.error_message = pi_arm_transmission::to_string(status.error);
    return false;
  }

  pi_arm_transmission::Vector6 recovered_positions{};
  status = kinematics_.output_to_joint_position(output_positions_rad_, recovered_positions);
  if (!status) {
    sample_.error_message = pi_arm_transmission::to_string(status.error);
    return false;
  }
  pi_arm_transmission::Vector6 recovered_velocities{};
  status = kinematics_.output_to_joint_velocity(
    recovered_positions, output_velocities_rad_s_, recovered_velocities);
  if (!status) {
    sample_.error_message = pi_arm_transmission::to_string(status.error);
    return false;
  }

  sample_.positions_rad = to_vector(recovered_positions);
  sample_.velocities_rad_s = to_vector(recovered_velocities);
  sample_.kinematics_valid = true;
  sample_.model_output_positions_rad = to_vector(output_positions_rad_);
  sample_.raw_output_positions_rad = sample_.model_output_positions_rad;
  for (std::size_t index = 0; index < kAxisCount; ++index) {
    sample_.raw_protocol_positions_deg[index] =
      output_position_to_protocol_deg(output_positions_rad_[index], config_.actuators[index]);
    sample_.in_motion[index] =
      std::abs(recovered_velocities[index]) > config_.moving_speed_threshold_rad_s;
    sample_.motor_in_motion[index] =
      std::abs(output_velocities_rad_s_[index]) > config_.moving_speed_threshold_rad_s;
    sample_.motor_fresh[index] = true;
    sample_.motor_ages_sec[index] = 0.0;
  }
  aggregate_mock_motor_state_to_joints(sample_, kinematics_.parameters().swap_wrist_channels);
  return true;
}

bool MockBackend::valid_motor_id(const int motor_id) const noexcept
{
  return std::any_of(
    config_.actuators.begin(), config_.actuators.end(),
    [motor_id](const auto & actuator) {return actuator.motor_id == motor_id;});
}

ManagementResult MockBackend::manage(
  const pi_arm_can::ManagementOperation operation, const int motor_id) noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {
    return {false, 2, "mock backend is not configured"};
  }
  if (operation == pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO) {
    return {
      false, 6,
      "ROM zero is disabled in the normal hardware service; use the local maintenance CLI"};
  }
  if (operation == pi_arm_can::ManagementOperation::MOTOR_REBOOT && motor_id <= 0) {
    return {false, 1004, "reboot requires one explicit motor_id"};
  }
  if (motor_id != 0 && !valid_motor_id(motor_id)) {
    return {false, 1, "motor_id is not configured (use 0 for all motors)"};
  }
  if (sample_.teaching_active && operation == pi_arm_can::ManagementOperation::MOTOR_RUN) {
    return {false, 2001, "exit drag teaching before enabling"};
  }
  if (operation != pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS) {
    for (const auto samples : stationary_samples_) {
      if (samples < 3U) {
        return {false, 5, "motor must be stationary for three consecutive samples"};
      }
    }
  }

  for (std::size_t index = 0; index < kAxisCount; ++index) {
    if (motor_id != 0 && config_.actuators[index].motor_id != motor_id) {
      continue;
    }
    switch (operation) {
      case pi_arm_can::ManagementOperation::NONE:
        return {false, 1, "management operation NONE is not executable"};
      case pi_arm_can::ManagementOperation::MOTOR_OFF:
        sample_.motor_enabled[index] = false;
        sample_.motor_in_motion[index] = false;
        break;
      case pi_arm_can::ManagementOperation::MOTOR_RUN:
        sample_.motor_enabled[index] = true;
        break;
      case pi_arm_can::ManagementOperation::MOTOR_REBOOT:
        sample_.motor_enabled[index] = false;
        sample_.motor_in_motion[index] = false;
        [[fallthrough]];
      case pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS:
        sample_.motor_has_error[index] = false;
        sample_.motor_error_codes[index] = 0;
        break;
      case pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO:
        return {false, 6, "ROM zero is unavailable through normal backend management"};
    }
  }
  aggregate_mock_motor_state_to_joints(sample_, kinematics_.parameters().swap_wrist_channels);
  if (active_) {
    const bool all_enabled = std::all_of(
      sample_.motor_enabled.begin(), sample_.motor_enabled.end(), [](bool value) {return value;});
    sample_.driver_mode = all_enabled ? "CONTROL" : "MONITORING";
    sample_.control_permitted = all_enabled && !sample_.teaching_active;
  }
  return {true, 0, "mock management operation applied"};
}

ManagementResult MockBackend::set_teaching(const bool enabled) noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_ || !active_ || config_.read_only || config_.commissioning ||
    std::any_of(stationary_samples_.begin(), stationary_samples_.end(), [](auto n) {return n < 3U;})) {
    return {false, 2001, "mock teaching requires stationary active hardware"};
  }
  if (!enabled && (!sample_.teaching_active || !std::all_of(sample_.motor_enabled.begin(),
    sample_.motor_enabled.end(), [](bool value) {return value;}))) {
    return {false, 2001, "holding requires enabled motors in drag teaching"};
  }
  hold_mock_current_ = true;
  sample_.teaching_active = enabled;
  sample_.control_permitted = !enabled && std::all_of(sample_.motor_enabled.begin(), sample_.motor_enabled.end(),
    [](bool value) {return value;});
  aggregate_mock_motor_state_to_joints(sample_, kinematics_.parameters().swap_wrist_channels);
  return {true, 0, enabled ? "mock drag teaching active" : "mock holding current pose"};
}

MotionArmResult MockBackend::set_motion_armed(
  const bool armed, const std::uint64_t lease_id, const double max_output_velocity, const bool hold_position) noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {
    return {false, false, "mock backend is not configured"};
  }
  if (armed && sample_.teaching_active) {return {false, false, "drag teaching active"};}
  if (armed && lease_id == 0U) {
    return {false, false, "invalid mock motion lease"};
  }
  if (armed && (!std::isfinite(max_output_velocity) ||
    (max_output_velocity != 0.0 && (max_output_velocity < 0.002908882086657216 ||
    max_output_velocity > 0.17453292519943295)))) {
    return {false, false, "invalid output speed limit"};
  }
  if (armed) {hold_mock_current_ = false;}
  if (!armed && hold_position && lease_id == 0U) {
    hold_mock_current_ = true;
    output_velocities_rad_s_ = {};
    std::fill(sample_.velocities_rad_s.begin(), sample_.velocities_rad_s.end(), 0.0);
    std::fill(sample_.in_motion.begin(), sample_.in_motion.end(), false);
    std::fill(sample_.motor_in_motion.begin(), sample_.motor_in_motion.end(), false);
  }
  // Mock has no CAN write boundary; accept the manager handshake without
  // changing its existing deterministic command behavior.
  return {true, armed, armed ? "mock motion lease accepted" : "mock motion lease released"};
}

}  // namespace pi_arm_hardware
