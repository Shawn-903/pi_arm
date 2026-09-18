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
  "joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"};

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
    "write velocity clamped at joint[%zu]: vel=%.17g -> lim=%.17g",
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
              "actuator order must be joint_01, joint_02, joint_04, joint_05, joint_06, joint_09");
    }
    if (!motor_ids.insert(config.actuators[index].motor_id).second) {
      throw std::invalid_argument("actuator motor IDs must be unique");
    }
    if (!std::isfinite(config.joint_lower_limits_rad[index]) ||
      !std::isfinite(config.joint_feedback_tolerances_rad[index]) ||
      config.joint_feedback_tolerances_rad[index] < 0.0 ||
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
  const std::size_t joint_index, const std::size_t motor_index, const bool swap_wrist_channels) noexcept
{
  if (joint_index == 0U) {
    return motor_index == 0U;
  }
  if (joint_index == 1U) {
    return motor_index == 1U;
  }
  const auto own_motor = swap_wrist_channels && (joint_index == 3U || joint_index == 4U) ?
    7U - joint_index : joint_index;
  return motor_index == 1U || motor_index == own_motor;
}

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
  calibration_ = load_pack_reference_calibration(
    config_.calibration_file, config_.cable_kinematics_file, config_.actuators, kinematics_);
}

RealBackend::~RealBackend()
{
  cleanup();
}

void RealBackend::configure()
{
  pi_arm_can::DriverConfig driver_config;
  driver_config.interface_name = config_.can_interface;
  driver_config.read_only = config_.read_only;
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
  control_permitted_.store(false);
  set_motion_armed(false, 0U);
}

void RealBackend::cleanup() noexcept
{
  if (axis_test_active_) {stop_axis_test("stopped: process shutdown");}
  control_permitted_.store(false);
  set_motion_armed(false, 0U);
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
    const double nan = std::numeric_limits<double>::quiet_NaN();
    sample.positions_rad.assign(kAxisCount, nan);
    sample.velocities_rad_s.assign(kAxisCount, nan);
    sample.enabled.assign(kAxisCount, false);
    sample.has_error.assign(kAxisCount, false);
    sample.in_motion.assign(kAxisCount, false);
    sample.fresh.assign(kAxisCount, false);
    sample.error_codes.assign(kAxisCount, 0);
    sample.ages_sec.assign(kAxisCount, std::numeric_limits<double>::infinity());
    sample.motor_enabled.assign(kAxisCount, false);
    sample.motor_has_error.assign(kAxisCount, false);
    sample.motor_in_motion.assign(kAxisCount, false);
    sample.motor_fresh.assign(kAxisCount, false);
    sample.motor_error_codes.assign(kAxisCount, 0);
    sample.motor_ages_sec.assign(kAxisCount, std::numeric_limits<double>::infinity());
    sample.raw_protocol_positions_deg.assign(kAxisCount, nan);
    sample.raw_output_positions_rad.assign(kAxisCount, nan);
    sample.model_output_positions_rad.assign(kAxisCount, nan);
    sample.software_output_offsets_rad.assign(kAxisCount, nan);
    sample.calibrated = calibration_.valid;
    sample.calibration_id = calibration_.data.calibration_id;
    sample.kinematics_valid = false;
    sample.control_permitted = false;
    sample.kinematics_error_code = calibration_.valid ? "NONE" : "CALIBRATION_INVALID";
    sample.kinematics_error_axis = 0;
    sample.kinematics_error_value = 0.0;
    sample.kinematics_error_message = calibration_.valid ? "" : calibration_.message;
    sample.connected = driver_->running() && driver_->mode() != pi_arm_can::DriverMode::FAULT;
    sample.driver_mode = pi_arm_can::to_string(driver_->mode());
    sample.management_operation = pi_arm_can::to_string(driver_->management_operation());
    sample.error_message = driver_->last_error();
    if (!calibration_.valid && sample.error_message.empty()) {
      sample.error_message = "control inhibited: " + calibration_.message;
    }

    pi_arm_transmission::Vector6 model_output_positions{};
    pi_arm_transmission::Vector6 output_velocities{};
    std::array<bool, kAxisCount> angle_valid{};
    std::array<bool, kAxisCount> velocity_valid{};
    std::array<bool, kAxisCount> motor_enabled{};
    std::array<bool, kAxisCount> motor_error{};
    std::array<bool, kAxisCount> motor_in_motion{};
    std::array<bool, kAxisCount> motor_fresh{};
    std::array<std::int32_t, kAxisCount> motor_error_codes{};
    std::array<double, kAxisCount> motor_ages{};
    std::array<bool, kAxisCount> output_limit_error{};

    for (std::size_t index = 0; index < kAxisCount; ++index) {
      const auto & actuator = config_.actuators[index];
      const auto & state = snapshot.at(actuator.motor_id);
      angle_valid[index] = state.angle_valid;
      velocity_valid[index] = state.status2_valid;
      if (state.angle_valid) {
        const double raw_output =
          protocol_position_to_output_rad(state.protocol_angle_deg, actuator);
        sample.raw_protocol_positions_deg[index] = state.protocol_angle_deg;
        sample.raw_output_positions_rad[index] = raw_output;
        if (calibration_.valid) {
          const double offset = calibration_.software_output_offsets_rad[index];
          const double model_output = rom_relative_to_model_output_rad(raw_output, offset);
          sample.software_output_offsets_rad[index] = offset;
          sample.model_output_positions_rad[index] = model_output;
          model_output_positions[index] = model_output;
        }
      }
      if (state.status2_valid) {
        output_velocities[index] =
          protocol_velocity_to_output_rad_s(state.protocol_speed_dps, actuator);
        motor_in_motion[index] =
          std::abs(output_velocities[index]) > config_.moving_speed_threshold_rad_s;
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
    sample.motor_enabled.assign(motor_enabled.begin(), motor_enabled.end());
    sample.motor_has_error.assign(motor_error.begin(), motor_error.end());
    sample.motor_in_motion.assign(motor_in_motion.begin(), motor_in_motion.end());
    sample.motor_fresh.assign(motor_fresh.begin(), motor_fresh.end());
    sample.motor_error_codes.assign(motor_error_codes.begin(), motor_error_codes.end());
    sample.motor_ages_sec.assign(motor_ages.begin(), motor_ages.end());

    const bool all_angles_valid =
      std::all_of(angle_valid.begin(), angle_valid.end(), [](const bool value) {return value;});
    if (calibration_.valid && all_angles_valid) {
      auto status = kinematics_.check_output_feedback_limits(
        model_output_positions, config_.joint_feedback_tolerances_rad);
      if (!status) {
        const std::size_t index = status.axis - 1U;
        output_limit_error[index] = true;
        const auto & limit = kinematics_.parameters().output_limits[index];
        sample.kinematics_error_code = pi_arm_transmission::to_string(status.error);
        sample.kinematics_error_axis = static_cast<std::int32_t>(status.axis);
        sample.kinematics_error_value = status.value;
        sample.kinematics_error_message =
          "model output feedback envelope exceeded at motor_0" + std::to_string(status.axis);
        sample.error_message =
          sample.kinematics_error_message + ": value=" + std::to_string(status.value) +
          " rad, command envelope=[" + std::to_string(limit.lower_position_rad) + "," +
          std::to_string(limit.upper_position_rad) +
          "] rad; propagated joint feedback allowance also exceeded";
      } else {
        pi_arm_transmission::Vector6 recovered{};
        status = kinematics_.output_to_joint_position(model_output_positions, recovered);
        if (!status) {
          sample.kinematics_error_code = pi_arm_transmission::to_string(status.error);
          sample.kinematics_error_axis = static_cast<std::int32_t>(status.axis);
          sample.kinematics_error_value = status.value;
          sample.kinematics_error_message =
            std::string("cable position inverse failed: ") +
            pi_arm_transmission::to_string(status.error);
          sample.error_message = sample.kinematics_error_message;
        } else {
          {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            last_joint_positions_rad_ = recovered;
          }
          sample.positions_rad = to_vector(recovered);
          sample.kinematics_valid = true;
          sample.kinematics_error_code = "NONE";
          sample.kinematics_error_message.clear();
          for (size_t i = 0; i < kAxisCount; ++i) {
            const double lower = config_.joint_lower_limits_rad[i];
            const double upper = config_.joint_upper_limits_rad[i];
            const double tolerance = config_.joint_feedback_tolerances_rad[i];
            if (recovered[i] < lower - tolerance || recovered[i] > upper + tolerance)
            {
              sample.kinematics_valid = false;
              sample.kinematics_error_code = "JOINT_POSITION_LIMIT";
              sample.kinematics_error_axis = static_cast<int32_t>(i + 1);
              sample.kinematics_error_value = recovered[i];
              sample.kinematics_error_message = "joint feedback limit exceeded: " +
                config_.actuators[i].joint_name + " value=" + std::to_string(recovered[i]) +
                " rad, feedback limit=[" + std::to_string(lower - tolerance) + "," +
                std::to_string(upper + tolerance) + "] rad";
              sample.error_message = sample.kinematics_error_message;
              output_limit_error[i] = true;
              break;
            }
            if ((recovered[i] < lower || recovered[i] > upper) &&
              sample.kinematics_error_code == "NONE")
            {
              sample.kinematics_error_code = "JOINT_COMMAND_LIMIT_WARNING";
              sample.kinematics_error_axis = static_cast<int32_t>(i + 1);
              sample.kinematics_error_value = recovered[i];
              sample.kinematics_error_message =
                "feedback outside command limits, within feedback allowance: " +
                config_.actuators[i].joint_name + " command=[" + std::to_string(lower) +
                "," + std::to_string(upper) + "] rad, feedback allowance=" +
                std::to_string(tolerance) + " rad; command limits remain enforced";
              sample.error_message = sample.kinematics_error_message;
            }
          }
        }
      }
    } else if (calibration_.valid) {
      sample.kinematics_error_code = "RAW_ANGLE_STALE";
      sample.kinematics_error_message = "not all raw motor angles are available";
    }

    const bool all_velocities_valid =
      std::all_of(
      velocity_valid.begin(), velocity_valid.end(), [](const bool value) {return value;});
    if (sample.kinematics_valid && all_velocities_valid) {
      pi_arm_transmission::Vector6 recovered_velocity{};
      const auto status = kinematics_.output_to_joint_velocity(
        last_joint_positions_rad_, output_velocities, recovered_velocity);
      if (!status) {
        sample.kinematics_valid = false;
        sample.positions_rad.assign(kAxisCount, nan);
        sample.kinematics_error_code = pi_arm_transmission::to_string(status.error);
        sample.kinematics_error_axis = static_cast<std::int32_t>(status.axis);
        sample.kinematics_error_value = status.value;
        sample.kinematics_error_message =
          std::string("cable velocity inverse failed: ") +
          pi_arm_transmission::to_string(status.error);
        sample.error_message = sample.kinematics_error_message;
      } else {
        sample.velocities_rad_s = to_vector(recovered_velocity);
      }
    }
    position_initialized_ = sample.kinematics_valid;

    for (std::size_t joint_index = 0; joint_index < kAxisCount; ++joint_index) {
      bool enabled = true;
      bool has_error = false;
      bool fresh = sample.kinematics_valid;
      std::int32_t error_code = 0;
      double age = 0.0;
      for (std::size_t motor_index = 0; motor_index < kAxisCount; ++motor_index) {
        if (!joint_depends_on_motor(joint_index, motor_index, kinematics_.parameters().swap_wrist_channels)) {
          continue;
        }
        enabled = enabled && motor_enabled[motor_index];
        has_error = has_error || motor_error[motor_index] || output_limit_error[motor_index];
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
        std::isfinite(sample.velocities_rad_s[joint_index]) &&
        std::abs(sample.velocities_rad_s[joint_index]) >
        config_.moving_speed_threshold_rad_s;
    }

    const bool all_enabled =
      std::all_of(
      motor_enabled.begin(), motor_enabled.end(), [](const bool value) {return value;});
    const bool all_fresh =
      std::all_of(motor_fresh.begin(), motor_fresh.end(), [](const bool value) {return value;});
    const bool any_motor_error =
      std::any_of(motor_error.begin(), motor_error.end(), [](const bool value) {return value;});
    const auto stop_status = driver_->motion_stop_status();
    const bool stopping = stop_status == pi_arm_can::MotionStopStatus::PENDING;
    const bool stop_failed = stop_status == pi_arm_can::MotionStopStatus::FAILED;
    sample.motion_stop_pending = stopping;
    sample.motion_stop_failed = stop_failed;
    if (stop_failed) {
      sample.error_message = "STOP UNCONFIRMED: retry Stop; motion is inhibited";
      sample.has_error.assign(kAxisCount, true);
    } else if (stopping) {
      sample.error_message = "waiting for motor stop acknowledgements";
    }
    if (reboot_reference_uncertain_.load()) {
      sample.calibrated = false;
      sample.kinematics_valid = false;
      sample.kinematics_error_code = "REBOOT_REFERENCE_UNVERIFIED";
      sample.kinematics_error_message = "reboot position reference requires verification; motion inhibited";
      sample.error_message = sample.kinematics_error_message;
    }
    sample.teaching_active = teaching_active_.load();
    sample.control_permitted = !sample.teaching_active &&
      config_.allow_real_control && calibration_.valid && sample.kinematics_valid &&
      all_enabled && all_fresh && !any_motor_error && !stopping && !stop_failed;

    control_permitted_.store(sample.control_permitted);
    if (!sample.control_permitted && !axis_test_active_ &&
      !(sample.teaching_active && !motion_write_gate_.armed())) {
      // Feedback, calibration and kinematics validity are independent of an
      // accepted motion lease. Losing any prerequisite revokes the lease and
      // returns the CAN driver to read-only polling.
      disarm_motion(0U);
    }
    sample.driver_mode = pi_arm_can::to_string(driver_->mode());
    sample.motor_test_permitted = config_.commissioning && !axis_test_active_ &&
      !axis_test_stop_failed_ && axis_test_raw_ready(true);
    sample.joint_test_permitted = sample.motor_test_permitted && sample.calibrated &&
      sample.kinematics_valid && all_enabled;
    return true;
  } catch (const std::exception & error) {
    control_permitted_.store(false);
    disarm_motion(0U);
    sample.error_message = error.what();
    return false;
  }
}

MotionArmResult RealBackend::disarm_motion(
  const std::uint64_t lease_id, const bool retry_stop) noexcept
{
  std::lock_guard<std::mutex> lock(motion_mutex_);
  if (!motion_write_gate_.disarm(lease_id)) {
    return {false, true, "motion lease does not match the active lease"};
  }
  if (driver_ && (normal_motion_sent_ ||
    // Lease zero is an explicit force-stop, including after a process restart
    // when this backend cannot know what target the device last received.
    (retry_stop && lease_id == 0U && !config_.read_only && !config_.commissioning) ||
    (retry_stop && !config_.commissioning &&
    driver_->motion_stop_status() == pi_arm_can::MotionStopStatus::FAILED)))
  {
    std::vector<std::uint8_t> ids;
    for (const auto & actuator : config_.actuators) {ids.push_back(actuator.motor_id);}
    const bool requested = driver_->request_motor_stop(ids, retry_stop);
    // The driver retains a failed-stop latch even if enqueue failed.
    normal_motion_sent_ = false;
    if (!requested) {return {false, false, "STOP UNCONFIRMED: motion inhibited; retry Stop"};}
  }
  if (driver_ && driver_->mode() != pi_arm_can::DriverMode::FAULT &&
    driver_->mode() != pi_arm_can::DriverMode::OFFLINE)
  {
    driver_->set_mode(pi_arm_can::DriverMode::MONITORING);
  }
  return {true, false, "real position writes disarmed"};
}

MotionArmResult RealBackend::set_motion_armed(
  const bool armed, const std::uint64_t lease_id, const double max_output_velocity, const bool hold_position) noexcept
{
  if (hold_position) {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (armed || lease_id == 0U || !motion_write_gate_.armed() ||
      motion_write_gate_.active_lease_id() != lease_id || !control_permitted_.load() ||
      !driver_ || teaching_active_.load() || driver_->motion_stop_status() == pi_arm_can::MotionStopStatus::PENDING ||
      driver_->motion_stop_status() == pi_arm_can::MotionStopStatus::FAILED) {
      return {false, motion_write_gate_.armed(), "invalid hold release or hardware not ready"};
    }
    // Purge queued controller writes, retaining the last transmitted A4 target.
    // Keep normal_motion_sent_ set so faults and explicit Stop still stop the motor.
    if (!driver_->set_mode(pi_arm_can::DriverMode::MONITORING)) {
      return {false, true, "cannot close controller writes for holding"};
    }
    if (!normal_motion_sent_) {
      // An already-reached goal may never produce a fresh JTC command. It
      // still needs an explicit position loop (RUN alone is not a hold).
      const auto snapshot = driver_->snapshot();
      std::vector<pi_arm_can::PositionCommand> commands;
      const auto now = std::chrono::steady_clock::now();
      for (const auto & actuator : config_.actuators) {
        const auto & state = snapshot.at(actuator.motor_id);
        if (!state.angle_valid || age_seconds(state.angle_update, now) > config_.state_stale_timeout_sec ||
          state.protocol_speed_dps != 0.0 || state.stationary_samples < 3U) {
          return {false, true, "fresh stationary angles required for already-reached goal hold"};
        }
        commands.push_back({actuator.motor_id, state.protocol_angle_deg,
          std::max(1.0, std::floor(std::abs(output_velocity_to_protocol_dps(0.017453292519943295, actuator))))});
      }
      if (!driver_->hold_positions(commands)) {return {false, true, "current-position hold not confirmed"};}
      normal_motion_sent_ = true;
    }
    motion_write_gate_.disarm(lease_id);
    return {true, false, "trajectory lease released; motor position loop retained"};
  }
  if (!armed) {
    const auto result = disarm_motion(lease_id, true);
    if (!result.success) {return result;}
    if (driver_ && !config_.commissioning) {
      const auto status = driver_->motion_stop_status();
      if ((status == pi_arm_can::MotionStopStatus::PENDING && !driver_->wait_for_motor_stop()) ||
        status == pi_arm_can::MotionStopStatus::FAILED)
      {return {false, false, "STOP UNCONFIRMED: motion inhibited; retry Stop"};}
    }
    return {true, false, "position writes disarmed; sent motion stopped or no motion submitted"};
  }

  std::lock_guard<std::mutex> lock(motion_mutex_);
  if (driver_ && (driver_->motion_stop_status() == pi_arm_can::MotionStopStatus::PENDING ||
    driver_->motion_stop_status() == pi_arm_can::MotionStopStatus::FAILED))
  {return {false, false, "motor stop pending/unconfirmed"};}
  if (config_.read_only) {
    return {false, false, "read-only backend cannot arm position writes"};
  }
  if (config_.commissioning) {
    return {false, false, "commissioning mode cannot arm normal controller writes"};
  }
  if (teaching_active_.load() || !config_.allow_real_control || !calibration_.valid || !control_permitted_.load() ||
    reboot_reference_uncertain_.load()) {
    return {false, false, "real-control prerequisites are not satisfied"};
  }
  if (!driver_ || !driver_->running() ||
    (driver_->mode() != pi_arm_can::DriverMode::MONITORING &&
    driver_->mode() != pi_arm_can::DriverMode::CONTROL))
  {
    return {false, false, "CAN driver is not available for motion arming"};
  }
  const double speed_limit = max_output_velocity == 0.0 ? 0.08726646259971647 : max_output_velocity;
  if (!std::isfinite(speed_limit) || speed_limit < 0.002908882086657216 ||
    speed_limit > 0.17453292519943295) {
    return {false, false, "output speed limit must be finite and within 1/6..10 deg/s"};
  }
  if (motion_write_gate_.armed()) {
    return {false, true, "motion lease already armed"};
  }
  if (!motion_write_gate_.arm(lease_id))
  {
    return {
      false, motion_write_gate_.armed(),
      "invalid/conflicting motion lease or no finite idle command baseline"};
  }
  output_speed_limit_rad_s_ = speed_limit;
  {
    std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
    recovery_start_ = last_joint_positions_rad_;
  }
  // Remain in MONITORING until write() observes a controller command that is
  // newer than the idle/hold command captured above.
  return {true, true, "real position writes armed; waiting for a fresh controller command"};
}

bool RealBackend::write(
  const std::vector<double> & positions_rad,
  const std::vector<double> & velocities_rad_s) noexcept
{
  if (config_.commissioning) {return false;}
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
    disarm_motion(0U);
    return false;
  }

  if (!motion_write_gate_.armed()) {
    motion_write_gate_.observe_idle(positions_rad, velocities_rad_s);
    return true;
  }
  if (!control_permitted_.load()) {
    disarm_motion(0U);
    return true;
  }
  if (!std::all_of(positions_rad.begin(), positions_rad.end(), [](double value) {
      return std::isfinite(value);
    }) ||
    !std::all_of(velocities_rad_s.begin(), velocities_rad_s.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    log_write_reject("write rejected: armed command contains a non-finite value");
    disarm_motion(0U);
    return false;
  }
  const auto lease_id = motion_write_gate_.active_lease_id();
  pi_arm_transmission::Vector6 recovery_start{};
  double output_speed_limit = 0.0;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    recovery_start = recovery_start_;
    output_speed_limit = output_speed_limit_rad_s_;
  }
  if (!motion_write_gate_.permit_write(positions_rad, velocities_rad_s)) {
    // This is the captured idle/hold command, not a new controller command.
    return true;
  }

  try {
    pi_arm_transmission::Vector6 joint_positions{};
    pi_arm_transmission::Vector6 joint_velocities{};
    for (std::size_t index = 0; index < kAxisCount; ++index) {
      if (!std::isfinite(positions_rad[index]) || !std::isfinite(velocities_rad_s[index])) {
        log_write_reject(
          "write rejected: non-finite command at joint[%zu] pos=%.17g vel=%.17g",
          index, positions_rad[index], velocities_rad_s[index]);
        disarm_motion(0U);
        return false;
      }
      const double lower = config_.joint_lower_limits_rad[index];
      const double upper = config_.joint_upper_limits_rad[index];
      const double tolerance = config_.joint_feedback_tolerances_rad[index];
      // JTC may interpolate from measured feedback in the allowance region.
      // Accept only that captured start envelope, and project transmitted
      // targets onto the standard command range. Never send out-of-range q.
      if (positions_rad[index] < std::min(lower, recovery_start[index]) - 1e-12 ||
        positions_rad[index] > std::max(upper, recovery_start[index]) + 1e-12 ||
        positions_rad[index] < lower - tolerance || positions_rad[index] > upper + tolerance)
      {
        log_write_reject(
          "write rejected: position limit exceeded at joint[%zu] pos=%.17g lim=[%.17g,%.17g]",
          index, positions_rad[index],
          config_.joint_lower_limits_rad[index], config_.joint_upper_limits_rad[index]);
        disarm_motion(0U);
        return false;
      }
      joint_positions[index] = std::clamp(positions_rad[index], lower, upper);
      joint_velocities[index] = clamp_joint_velocity(
        velocities_rad_s[index], config_.joint_velocity_limits_rad_s[index], index);
    }

    pi_arm_transmission::Vector6 output_positions{};
    auto status = kinematics_.joint_to_output_position(joint_positions, output_positions);
    if (!status) {
      log_write_reject(
        "write rejected: cable position mapping failed (%s axis=%zu value=%.17g)",
        pi_arm_transmission::to_string(status.error), status.axis, status.value);
      disarm_motion(0U);
      return false;
    }
    status = kinematics_.check_output_position_limits(output_positions);
    if (!status) {
      const std::size_t index = status.axis - 1U;
      const auto & limit = kinematics_.parameters().output_limits[index];
      log_write_reject(
        "write rejected: output-shaft limit exceeded at motor[%zu] value=%.17g lim=[%.17g,%.17g]",
        status.axis, status.value, limit.lower_position_rad, limit.upper_position_rad);
      disarm_motion(0U);
      return false;
    }

    pi_arm_transmission::Vector6 output_velocities{};
    status = kinematics_.joint_to_output_velocity(
      joint_positions, joint_velocities, output_velocities);
    if (!status) {
      log_write_reject(
        "write rejected: cable velocity mapping failed (%s axis=%zu value=%.17g)",
        pi_arm_transmission::to_string(status.error), status.axis, status.value);
      disarm_motion(0U);
      return false;
    }

    std::vector<pi_arm_can::PositionCommand> commands;
    commands.reserve(kAxisCount);
    for (std::size_t index = 0; index < kAxisCount; ++index) {
      const auto & actuator = config_.actuators[index];
      const double protocol_cap = std::min(65535.0, std::floor(
          std::abs(output_velocity_to_protocol_dps(output_speed_limit, actuator)) + 1e-10));
      if (protocol_cap < 1.0) {disarm_motion(0U); return false;}
      // A4 speed is a ceiling, not a feedforward velocity. JTC's sampled
      // positions define the trajectory timing. Rounding feedforward down
      // accumulates tracking lag; setting zero can remove the motor's limit.
      const double protocol_speed = protocol_cap;
      commands.push_back(
        pi_arm_can::PositionCommand{
          actuator.motor_id,
          model_output_position_to_protocol_deg(
            output_positions[index], calibration_.software_output_offsets_rad[index], actuator),
          protocol_speed});
    }
    std::unique_lock<std::mutex> motion_lock(motion_mutex_);
    if (!motion_write_gate_.armed() || motion_write_gate_.active_lease_id() != lease_id ||
      !control_permitted_.load()) {return true;}
    if (!driver_->set_mode(pi_arm_can::DriverMode::CONTROL)) {return true;}
    if (!driver_->enqueue_position_batch(commands)) {
      motion_lock.unlock();
      if (!motion_write_gate_.armed() ||
        driver_->mode() == pi_arm_can::DriverMode::MONITORING)
      {
        // A concurrent cancel/stop revoked the lease after this cycle began.
        // Treat the dropped batch as the intended fail-closed outcome, not as
        // a ros2_control lifecycle communication error.
        return true;
      }
      const auto detail = driver_->last_error();
      log_write_reject(
        "write rejected: enqueue_position_batch failed (%s)",
        detail.empty() ? "no detail" : detail.c_str());
      disarm_motion(0U);
      return false;
    }
    normal_motion_sent_ = true;
    return true;
  } catch (const std::exception & error) {
    log_write_reject("write rejected: %s", error.what());
    disarm_motion(0U);
    return false;
  } catch (...) {
    log_write_reject("write rejected: unexpected exception");
    disarm_motion(0U);
    return false;
  }
}

ManagementResult RealBackend::manage(
  const pi_arm_can::ManagementOperation operation, const int motor_id) noexcept
{
  if (teaching_active_.load() && operation == pi_arm_can::ManagementOperation::MOTOR_RUN) {
    return {false, 2001, "exit drag teaching to hold the current position before enabling"};
  }
  if (axis_test_active_) {
    return {false, 2003, "stop the active axis test before changing motor power"};
  }
  if (!driver_) {
    return {false, 2, "CAN driver is not configured"};
  }
  if (operation == pi_arm_can::ManagementOperation::MOTOR_REBOOT && motor_id <= 0) {
    return {false, 1004, "reboot requires one explicit motor_id"};
  }
  if (operation == pi_arm_can::ManagementOperation::MOTOR_RUN && reboot_reference_uncertain_.load()) {
    return {false, 2002, "reboot position reference unverified; enabling is inhibited"};
  }
  // Power and fault-management operations are never motion-authorizing. They
  // revoke an existing lease before any management frame is queued.
  const auto stopped = set_motion_armed(false, 0U);
  if (!stopped.success && (operation == pi_arm_can::ManagementOperation::MOTOR_RUN ||
    operation == pi_arm_can::ManagementOperation::MOTOR_REBOOT)) {
    return {false, 2002, stopped.message};
  }
  if (operation == pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO) {
    return {
      false, 6,
      "ROM zero is disabled in the normal hardware service; use the local maintenance CLI"};
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
  if (operation == pi_arm_can::ManagementOperation::MOTOR_REBOOT) {
    // Do not mix reboot with a trajectory, or infer a preserved multi-turn
    // reference merely from CAN returning. No ROM/RAM zero command is sent.
    const auto before = driver_->snapshot();
    const auto before_time = std::chrono::steady_clock::now();
    for (const auto id : ids) {
      const auto & state = before.at(id);
      if (!state.angle_valid || age_seconds(state.angle_update, before_time) > config_.state_stale_timeout_sec) {
        return {false, 2001, "reboot requires fresh pre-reboot angle feedback"};
      }
    }
    const bool previously_uncertain = reboot_reference_uncertain_.exchange(true);
    control_permitted_.store(false);
    if (!driver_->execute_management(operation, ids)) {
      return {false, 2002, driver_->last_error() + "; verify position reference before resuming"};
    }
    const auto after = driver_->snapshot();
    for (const auto & actuator : config_.actuators) {
      if (std::find(ids.begin(), ids.end(), actuator.motor_id) == ids.end()) {continue;}
      const auto difference = std::abs(protocol_position_to_output_rad(
          after.at(actuator.motor_id).protocol_angle_deg - before.at(actuator.motor_id).protocol_angle_deg, actuator));
      if (!std::isfinite(difference) || difference > 0.003490658503988659) {
        return {false, 2002, "reboot feedback recovered but output angle changed >0.2 deg; position reference unverified"};
      }
    }
    reboot_reference_uncertain_.store(previously_uncertain);
    if (previously_uncertain) {
      return {false, 2002, "motor feedback recovered; earlier reboot reference remains unverified"};
    }
    return {true, 0, "reboot command sent (no ACK); fresh feedback recovered, errors clear, motor disabled; enable separately"};
  }
  if (!driver_->execute_management(operation, ids)) {
    const auto detail = driver_->last_error();
    return {false, 3, detail.empty() ? "management command was rejected" : detail};
  }
  if (operation == pi_arm_can::ManagementOperation::MOTOR_OFF) {
    driver_->set_mode(pi_arm_can::DriverMode::MONITORING);
  }

  return {true, 0, "management command transmitted"};
}

ManagementResult RealBackend::set_teaching(const bool enabled) noexcept
{
  if (config_.read_only || config_.commissioning || !config_.allow_real_control || !driver_) {
    return {false, 2001, "drag teaching requires normal writable hardware"};
  }
  if (enabled) {
    // Latch before releasing torque; even a partial failure must forbid motion.
    teaching_active_.store(true);
    control_permitted_.store(false);
    // 0x80 clears motor multi-turn reference according to the vendor manual.
    // 0x81 releases the position loop while retaining the running state/reference.
    const auto result = set_motion_armed(false, 0U);
    return result.success ? ManagementResult{true, 0, "drag teaching active; position loop stopped, support arm manually"} :
      ManagementResult{false, 2002, result.message};
  }
  if (!teaching_active_.load()) {return {false, 2001, "drag teaching is not active"};}
  HardwareSample current;
  if (!read(current) || !current.calibrated || !current.kinematics_valid ||
    current.motion_stop_pending || current.motion_stop_failed || reboot_reference_uncertain_.load()) {
    return {false, 2001, "fresh valid calibration/kinematics required to hold current pose"};
  }
  const auto snapshot = driver_->snapshot();
  const auto now = std::chrono::steady_clock::now();
  std::vector<pi_arm_can::PositionCommand> commands;
  for (const auto & actuator : config_.actuators) {
    const auto & state = snapshot.at(actuator.motor_id);
    if (!state.angle_valid || !state.status1_valid || !state.status2_valid ||
      age_seconds(state.angle_update, now) > config_.state_stale_timeout_sec ||
      age_seconds(state.status1_update, now) > config_.state_stale_timeout_sec ||
      age_seconds(state.status2_update, now) > config_.state_stale_timeout_sec ||
      state.error_state != 0 || (state.motor_state != 0x00 && state.motor_state != 0x30) ||
      state.protocol_speed_dps != 0.0 || state.stationary_samples < 3U) {
      return {false, 2001, "holding requires fresh enabled, error-free stationary motors; power-off requires reference verification"};
    }
    commands.push_back({actuator.motor_id, state.protocol_angle_deg,
      std::max(1.0, std::floor(std::abs(output_velocity_to_protocol_dps(0.017453292519943295, actuator))))});
  }
  // Services are serialized by the auxiliary executor. The control loop is
  // disarmed; no old controller target can overwrite these captured targets.
  if (!driver_->hold_positions(commands)) {
    return {false, 2002, "failed to confirm current-position hold; teaching remains active"};
  }
  {std::lock_guard<std::mutex> lock(motion_mutex_); normal_motion_sent_ = true;}
  teaching_active_.store(false);
  return {true, 0, "drag teaching ended; motors holding captured current positions"};
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
  (void)hold_position;
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
  // Mock has no CAN write boundary; accept the manager handshake without
  // changing its existing deterministic command behavior.
  return {true, armed, armed ? "mock motion lease accepted" : "mock motion lease released"};
}

}  // namespace pi_arm_hardware
