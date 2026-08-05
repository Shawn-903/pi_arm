#include "pi_arm_hardware/backend.hpp"

#include "pi_arm_transmission/config_loader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"

namespace pi_arm_hardware
{
namespace
{

constexpr std::size_t kAxisCount = 6U;
constexpr int64_t kWriteRejectLogPeriodMs = 1000;
constexpr std::array<const char *, kAxisCount> kExpectedJointNames{
  "joint_01", "joint_02", "joint_03", "joint_05", "joint_06", "joint_09"};

rclcpp::Logger backend_logger()
{
  return rclcpp::get_logger("PiArmBackend");
}

rclcpp::Clock & steady_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

template<typename ... Args>
void log_write_reject(const char * format, Args && ... args) noexcept
{
  try {
    RCLCPP_ERROR_THROTTLE(
      backend_logger(), steady_clock(), kWriteRejectLogPeriodMs,
      format, std::forward<Args>(args)...);
  } catch (...) {
  }
}

template<typename ... Args>
void log_write_warn(const char * format, Args && ... args) noexcept
{
  try {
    RCLCPP_WARN_THROTTLE(
      backend_logger(), steady_clock(), kWriteRejectLogPeriodMs,
      format, std::forward<Args>(args)...);
  } catch (...) {
  }
}

double clamp_joint_velocity(
  const double velocity, const double velocity_limit, const std::size_t index) noexcept
{
  if (!(std::abs(velocity) > velocity_limit + 1e-6)) {
    return velocity;
  }
  log_write_warn(
    "write velocity clamped at joint[%zu]: vel=%.6g -> lim=%.6g",
    index, velocity, velocity_limit);
  return std::copysign(velocity_limit, velocity);
}

double age_seconds(
  const std::chrono::steady_clock::time_point timestamp,
  const std::chrono::steady_clock::time_point now)
{
  if (timestamp == std::chrono::steady_clock::time_point{}) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(now - timestamp).count();
}

void validate_backend_config(const BackendConfig & config)
{
  if (config.actuators.size() != kAxisCount) {
    throw std::invalid_argument("backend requires exactly six actuators");
  }
  if (config.cable_kinematics_file.empty()) {
    throw std::invalid_argument("cable_kinematics_file must not be empty");
  }
  if (config.joint_lower_limits_rad.size() != kAxisCount ||
    config.joint_upper_limits_rad.size() != kAxisCount ||
    config.joint_velocity_limits_rad_s.size() != kAxisCount)
  {
    throw std::invalid_argument("backend requires six joint position and velocity limits");
  }

  std::unordered_set<std::uint8_t> motor_ids;
  for (std::size_t index = 0; index < config.actuators.size(); ++index) {
    validate_actuator_config(config.actuators[index]);
    if (config.actuators[index].joint_name != kExpectedJointNames[index]) {
      throw std::invalid_argument(
              "actuator order must be joint_01, joint_02, joint_03, joint_05, joint_06, joint_09");
    }
    if (!motor_ids.insert(config.actuators[index].motor_id).second) {
      throw std::invalid_argument("actuator motor IDs must be unique");
    }
    if (!std::isfinite(config.joint_lower_limits_rad[index]) ||
      !std::isfinite(config.joint_upper_limits_rad[index]) ||
      config.joint_lower_limits_rad[index] > config.joint_upper_limits_rad[index] ||
      !std::isfinite(config.joint_velocity_limits_rad_s[index]) ||
      config.joint_velocity_limits_rad_s[index] <= 0.0)
    {
      throw std::invalid_argument("backend joint limits must be finite and valid");
    }
  }
}

std::vector<std::uint8_t> selected_motor_ids(
  const BackendConfig & config, const int requested_motor_id)
{
  std::vector<std::uint8_t> result;
  for (const auto & actuator : config.actuators) {
    if (requested_motor_id == 0 || requested_motor_id == actuator.motor_id) {
      result.push_back(actuator.motor_id);
    }
  }
  return result;
}

bool joint_depends_on_motor(
  const std::size_t joint_index, const std::size_t motor_index) noexcept
{
  if (joint_index == 0U) {
    return motor_index == 0U;
  }
  if (joint_index == 1U) {
    return motor_index == 1U;
  }
  return motor_index == 1U || motor_index == joint_index;
}

template<typename T>
std::vector<double> to_vector(const T & values)
{
  return std::vector<double>(values.begin(), values.end());
}

}  // namespace

RealBackend::RealBackend(BackendConfig config)
: config_(std::move(config)),
  kinematics_(
    pi_arm_transmission::load_cable_kinematics_config(config_.cable_kinematics_file))
{
  validate_backend_config(config_);
}

RealBackend::~RealBackend()
{
  cleanup();
}

void RealBackend::configure()
{
  power_cycle_required_ = false;
  pi_arm_can::DriverConfig driver_config;
  driver_config.interface_name = config_.can_interface;
  for (const auto & actuator : config_.actuators) {
    driver_config.motor_ids.push_back(actuator.motor_id);
  }
  driver_config.state_poll_interval = config_.poll_interval;
  driver_config.status1_poll_divider = config_.status1_poll_divider;
  driver_ = std::make_unique<pi_arm_can::CanDriver>(std::move(driver_config));
  driver_->start();
}

void RealBackend::activate()
{
  if (!driver_ || !driver_->running()) {
    throw std::runtime_error("CAN driver is not running");
  }
}

void RealBackend::deactivate() noexcept
{
  if (driver_) {
    driver_->set_mode(pi_arm_can::DriverMode::MONITORING);
  }
}

void RealBackend::cleanup() noexcept
{
  if (driver_) {
    driver_->stop();
    driver_.reset();
  }
  position_initialized_ = false;
}

bool RealBackend::read(HardwareSample & sample) noexcept
{
  if (!driver_) {
    return false;
  }

  try {
    const auto snapshot = driver_->snapshot();
    const auto now = std::chrono::steady_clock::now();
    sample.positions_rad = to_vector(last_joint_positions_rad_);
    sample.velocities_rad_s.assign(kAxisCount, 0.0);
    sample.enabled.assign(kAxisCount, false);
    sample.has_error.assign(kAxisCount, false);
    sample.in_motion.assign(kAxisCount, false);
    sample.fresh.assign(kAxisCount, false);
    sample.error_codes.assign(kAxisCount, 0);
    sample.ages_sec.assign(kAxisCount, std::numeric_limits<double>::infinity());
    sample.connected = driver_->running() && driver_->mode() != pi_arm_can::DriverMode::FAULT;
    sample.driver_mode = pi_arm_can::to_string(driver_->mode());
    sample.management_operation = pi_arm_can::to_string(driver_->management_operation());
    sample.error_message = driver_->last_error();
    if (power_cycle_required_) {
      sample.error_message = "motor zero was written to ROM; power-cycle is required";
    }

    pi_arm_transmission::Vector6 output_positions{};
    pi_arm_transmission::Vector6 output_velocities{};
    std::array<bool, kAxisCount> angle_valid{};
    std::array<bool, kAxisCount> velocity_valid{};
    std::array<bool, kAxisCount> motor_enabled{};
    std::array<bool, kAxisCount> motor_error{};
    std::array<bool, kAxisCount> motor_fresh{};
    std::array<std::int32_t, kAxisCount> motor_error_codes{};
    std::array<double, kAxisCount> motor_ages{};

    for (std::size_t index = 0; index < kAxisCount; ++index) {
      const auto & actuator = config_.actuators[index];
      const auto & state = snapshot.at(actuator.motor_id);
      angle_valid[index] = state.angle_valid;
      velocity_valid[index] = state.status2_valid;
      if (state.angle_valid) {
        output_positions[index] =
          protocol_position_to_output_rad(state.protocol_angle_deg, actuator);
      }
      if (state.status2_valid) {
        output_velocities[index] =
          protocol_velocity_to_output_rad_s(state.protocol_speed_dps, actuator);
      }
      if (state.status1_valid) {
        motor_enabled[index] = state.motor_state == 0x00U || state.motor_state == 0x30U;
        motor_error[index] = state.error_state != 0U;
        motor_error_codes[index] = state.error_state;
      }
      const double angle_age = age_seconds(state.angle_update, now);
      const double status1_age = age_seconds(state.status1_update, now);
      const double status2_age = age_seconds(state.status2_update, now);
      motor_ages[index] = std::max({angle_age, status1_age, status2_age});
      motor_fresh[index] =
        state.angle_valid && state.status1_valid && state.status2_valid &&
        motor_ages[index] <= config_.state_stale_timeout_sec;
    }

    const bool all_angles_valid =
      std::all_of(angle_valid.begin(), angle_valid.end(), [](const bool value) {return value;});
    if (all_angles_valid) {
      pi_arm_transmission::Vector6 recovered{};
      const auto status = kinematics_.output_to_joint_position(output_positions, recovered);
      if (!status) {
        sample.error_message =
          std::string("cable position inverse failed: ") +
          pi_arm_transmission::to_string(status.error);
        return false;
      }
      last_joint_positions_rad_ = recovered;
      position_initialized_ = true;
      sample.positions_rad = to_vector(recovered);
    }

    const bool all_velocities_valid =
      std::all_of(
      velocity_valid.begin(), velocity_valid.end(), [](const bool value) {return value;});
    if (position_initialized_ && all_velocities_valid) {
      pi_arm_transmission::Vector6 recovered_velocity{};
      const auto status = kinematics_.output_to_joint_velocity(
        last_joint_positions_rad_, output_velocities, recovered_velocity);
      if (!status) {
        sample.error_message =
          std::string("cable velocity inverse failed: ") +
          pi_arm_transmission::to_string(status.error);
        return false;
      }
      sample.velocities_rad_s = to_vector(recovered_velocity);
    }

    for (std::size_t joint_index = 0; joint_index < kAxisCount; ++joint_index) {
      bool enabled = true;
      bool has_error = false;
      bool fresh = position_initialized_;
      std::int32_t error_code = 0;
      double age = 0.0;
      for (std::size_t motor_index = 0; motor_index < kAxisCount; ++motor_index) {
        if (!joint_depends_on_motor(joint_index, motor_index)) {
          continue;
        }
        enabled = enabled && motor_enabled[motor_index];
        has_error = has_error || motor_error[motor_index];
        fresh = fresh && motor_fresh[motor_index];
        error_code |= motor_error_codes[motor_index];
        age = std::max(age, motor_ages[motor_index]);
      }
      sample.enabled[joint_index] = enabled;
      sample.has_error[joint_index] = has_error;
      sample.fresh[joint_index] = fresh;
      sample.error_codes[joint_index] = error_code;
      sample.ages_sec[joint_index] = age;
      sample.in_motion[joint_index] =
        std::abs(sample.velocities_rad_s[joint_index]) >
        config_.moving_speed_threshold_rad_s;
    }

    const bool all_enabled =
      std::all_of(
      motor_enabled.begin(), motor_enabled.end(), [](const bool value) {return value;});
    const bool all_fresh =
      std::all_of(motor_fresh.begin(), motor_fresh.end(), [](const bool value) {return value;});
    if (driver_->mode() == pi_arm_can::DriverMode::MONITORING &&
      all_enabled && all_fresh && position_initialized_ && !power_cycle_required_)
    {
      driver_->set_mode(pi_arm_can::DriverMode::CONTROL);
      sample.driver_mode = pi_arm_can::to_string(driver_->mode());
    } else if (driver_->mode() == pi_arm_can::DriverMode::CONTROL &&
      (!all_enabled || !all_fresh || !position_initialized_ || power_cycle_required_))
    {
      driver_->set_mode(pi_arm_can::DriverMode::MONITORING);
      sample.driver_mode = pi_arm_can::to_string(driver_->mode());
    }
    return true;
  } catch (const std::exception & error) {
    sample.error_message = error.what();
    return false;
  }
}

bool RealBackend::write(
  const std::vector<double> & positions_rad,
  const std::vector<double> & velocities_rad_s) noexcept
{
  if (!driver_ || positions_rad.size() != kAxisCount ||
    velocities_rad_s.size() != kAxisCount ||
    config_.joint_lower_limits_rad.size() != kAxisCount ||
    config_.joint_upper_limits_rad.size() != kAxisCount ||
    config_.joint_velocity_limits_rad_s.size() != kAxisCount)
  {
    log_write_reject(
      "write rejected: size/config mismatch (driver=%d pos=%zu vel=%zu actuators=%zu)",
      driver_ ? 1 : 0, positions_rad.size(), velocities_rad_s.size(),
      config_.actuators.size());
    return false;
  }
  if (driver_->mode() == pi_arm_can::DriverMode::MONITORING ||
    driver_->mode() == pi_arm_can::DriverMode::EXCLUSIVE_MANAGEMENT)
  {
    return true;
  }
  if (driver_->mode() != pi_arm_can::DriverMode::CONTROL) {
    log_write_reject(
      "write rejected: driver mode is not CONTROL (mode=%d)",
      static_cast<int>(driver_->mode()));
    return false;
  }

  try {
    pi_arm_transmission::Vector6 joint_positions{};
    pi_arm_transmission::Vector6 joint_velocities{};
    for (std::size_t index = 0; index < kAxisCount; ++index) {
      if (!std::isfinite(positions_rad[index]) || !std::isfinite(velocities_rad_s[index])) {
        log_write_reject(
          "write rejected: non-finite command at joint[%zu] pos=%.6g vel=%.6g",
          index, positions_rad[index], velocities_rad_s[index]);
        return false;
      }
      if (positions_rad[index] < config_.joint_lower_limits_rad[index] ||
        positions_rad[index] > config_.joint_upper_limits_rad[index])
      {
        log_write_reject(
          "write rejected: position limit exceeded at joint[%zu] pos=%.6g lim=[%.6g,%.6g]",
          index, positions_rad[index],
          config_.joint_lower_limits_rad[index], config_.joint_upper_limits_rad[index]);
        return false;
      }
      joint_positions[index] = positions_rad[index];
      joint_velocities[index] = clamp_joint_velocity(
        velocities_rad_s[index], config_.joint_velocity_limits_rad_s[index], index);
    }

    pi_arm_transmission::Vector6 output_positions{};
    auto status = kinematics_.joint_to_output_position(joint_positions, output_positions);
    if (!status) {
      log_write_reject(
        "write rejected: cable position mapping failed (%s axis=%zu value=%.6g)",
        pi_arm_transmission::to_string(status.error), status.axis, status.value);
      return false;
    }

    pi_arm_transmission::Vector6 output_velocities{};
    status = kinematics_.joint_to_output_velocity(
      joint_positions, joint_velocities, output_velocities);
    if (!status) {
      log_write_reject(
        "write rejected: cable velocity mapping failed (%s axis=%zu value=%.6g)",
        pi_arm_transmission::to_string(status.error), status.axis, status.value);
      return false;
    }

    pi_arm_transmission::Matrix6 jacobian{};
    status = kinematics_.jacobian(joint_positions, jacobian);
    if (!status) {
      return false;
    }

    std::vector<pi_arm_can::PositionCommand> commands;
    commands.reserve(kAxisCount);
    for (std::size_t index = 0; index < kAxisCount; ++index) {
      const auto & actuator = config_.actuators[index];
      double protocol_speed =
        std::abs(output_velocity_to_protocol_dps(output_velocities[index], actuator));
      if (protocol_speed < 1.0) {
        double output_speed_envelope = 0.0;
        for (std::size_t joint_index = 0; joint_index < kAxisCount; ++joint_index) {
          output_speed_envelope +=
            std::abs(jacobian[index][joint_index]) *
            config_.joint_velocity_limits_rad_s[joint_index];
        }
        protocol_speed =
          std::max(
          1.0,
          std::abs(output_velocity_to_protocol_dps(output_speed_envelope, actuator)));
      }
      commands.push_back(
        pi_arm_can::PositionCommand{
          actuator.motor_id,
          output_position_to_protocol_deg(output_positions[index], actuator),
          protocol_speed});
    }
    if (!driver_->enqueue_position_batch(commands)) {
      const auto detail = driver_->last_error();
      log_write_reject(
        "write rejected: enqueue_position_batch failed (%s)",
        detail.empty() ? "no detail" : detail.c_str());
      return false;
    }
    return true;
  } catch (const std::exception & error) {
    log_write_reject("write rejected: %s", error.what());
    return false;
  } catch (...) {
    log_write_reject("write rejected: unexpected exception");
    return false;
  }
}

ManagementResult RealBackend::manage(
  const pi_arm_can::ManagementOperation operation, const int motor_id) noexcept
{
  if (!driver_) {
    return {false, 2, "CAN driver is not configured"};
  }
  if (operation == pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO &&
    motor_id != 0)
  {
    return {false, 1, "set_zero is a whole-arm operation; motor_id must be 0"};
  }
  const auto ids = selected_motor_ids(config_, motor_id);
  if (ids.empty()) {
    return {false, 1, "motor_id is not configured (use 0 for all motors)"};
  }
  if (operation != pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS) {
    const auto snapshot = driver_->snapshot();
    const auto now = std::chrono::steady_clock::now();
    for (const auto & actuator : config_.actuators) {
      const auto & state = snapshot.at(actuator.motor_id);
      if (!state.status2_valid ||
        age_seconds(state.status2_update, now) > config_.state_stale_timeout_sec)
      {
        return {false, 4, "motor speed feedback is stale"};
      }
      if (state.protocol_speed_dps != 0.0 || state.stationary_samples < 3U) {
        return {false, 5, "motor must report zero speed for three consecutive samples"};
      }
    }
  }
  if (!driver_->execute_management(operation, ids)) {
    const auto detail = driver_->last_error();
    return {false, 3, detail.empty() ? "management command was rejected" : detail};
  }
  if (operation == pi_arm_can::ManagementOperation::MOTOR_OFF) {
    driver_->set_mode(pi_arm_can::DriverMode::MONITORING);
  }
  if (operation == pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO) {
    power_cycle_required_ = true;
    position_initialized_ = false;
    driver_->set_mode(pi_arm_can::DriverMode::MONITORING);
    return {
      true, 0,
      "zero written to ROM for all motors; power-cycle and restart the hardware lifecycle"};
  }
  return {true, 0, "management command transmitted"};
}

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
  stationary_samples_.assign(kAxisCount, 0U);
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
  sample_.driver_mode = "CONTROL";
  std::fill(sample_.fresh.begin(), sample_.fresh.end(), true);
}

void MockBackend::deactivate() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  sample_.connected = false;
  sample_.driver_mode = configured_ ? "MONITORING" : "OFFLINE";
  output_velocities_rad_s_ = {};
  std::fill(sample_.velocities_rad_s.begin(), sample_.velocities_rad_s.end(), 0.0);
  std::fill(sample_.in_motion.begin(), sample_.in_motion.end(), false);
  std::fill(sample_.fresh.begin(), sample_.fresh.end(), false);
}

void MockBackend::cleanup() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  configured_ = false;
  sample_.connected = false;
  sample_.driver_mode = "OFFLINE";
  std::fill(sample_.enabled.begin(), sample_.enabled.end(), false);
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
    sample_.enabled.begin(), sample_.enabled.end(), [](const bool value) {return value;});
  if (!all_enabled) {
    output_velocities_rad_s_ = {};
    std::fill(sample_.velocities_rad_s.begin(), sample_.velocities_rad_s.end(), 0.0);
    std::fill(sample_.in_motion.begin(), sample_.in_motion.end(), false);
    return true;
  }

  auto status = kinematics_.joint_to_output_position(joint_positions, output_positions_rad_);
  if (!status) {
    sample_.error_message = pi_arm_transmission::to_string(status.error);
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
  for (std::size_t index = 0; index < kAxisCount; ++index) {
    sample_.in_motion[index] =
      std::abs(recovered_velocities[index]) > config_.moving_speed_threshold_rad_s;
    sample_.fresh[index] = true;
    sample_.ages_sec[index] = 0.0;
  }
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
  if (operation == pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO &&
    motor_id != 0)
  {
    return {false, 1, "set_zero is a whole-arm operation; motor_id must be 0"};
  }
  if (motor_id != 0 && !valid_motor_id(motor_id)) {
    return {false, 1, "motor_id is not configured (use 0 for all motors)"};
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
        sample_.enabled[index] = false;
        sample_.velocities_rad_s[index] = 0.0;
        sample_.in_motion[index] = false;
        break;
      case pi_arm_can::ManagementOperation::MOTOR_RUN:
        sample_.enabled[index] = true;
        break;
      case pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS:
        sample_.has_error[index] = false;
        sample_.error_codes[index] = 0;
        break;
      case pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO:
        break;
    }
  }
  if (operation == pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO) {
    output_positions_rad_ = {};
    output_velocities_rad_s_ = {};
    std::fill(sample_.positions_rad.begin(), sample_.positions_rad.end(), 0.0);
    std::fill(sample_.velocities_rad_s.begin(), sample_.velocities_rad_s.end(), 0.0);
    std::fill(sample_.in_motion.begin(), sample_.in_motion.end(), false);
    return {true, 0, "mock whole-arm zero applied"};
  }
  return {true, 0, "mock management operation applied"};
}

}  // namespace pi_arm_hardware
