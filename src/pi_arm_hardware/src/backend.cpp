#include "pi_arm_hardware/backend.hpp"
#include "backend_common.hpp"
#include "pi_arm_hardware/software_zero.hpp"

#include "pi_arm_transmission/config_loader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

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

double age_seconds(
  const std::chrono::steady_clock::time_point timestamp,
  const std::chrono::steady_clock::time_point now)
{
  if (timestamp == std::chrono::steady_clock::time_point{}) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(now - timestamp).count();
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

}  // namespace

RealBackend::RealBackend(BackendConfig config)
: config_(std::move(config)),
  kinematics_(
    pi_arm_transmission::load_cable_kinematics_config(config_.cable_kinematics_file))
{
  validate_backend_config(config_);
  calibration_ = load_pack_reference_calibration(
    config_.calibration_file, config_.cable_kinematics_file, config_.actuators, kinematics_);
  if (!config_.software_zero_file.empty()) {
    try {
      if (!std::filesystem::path(config_.software_zero_file).is_absolute()) {
        throw std::invalid_argument("software_zero_file must be absolute");
      }
      if (!config_.commissioning || std::filesystem::exists(config_.software_zero_file)) {
        calibration_ = load_software_zero_reference(config_.software_zero_file,
          calibration_, config_.actuators, kinematics_);
      }
    } catch (const std::exception & error) {
      calibration_.valid = false;
      calibration_.message = std::string("software zero invalid: ") + error.what();
    }
  }
}

RealBackend::~RealBackend()
{
  cleanup();
}

void RealBackend::configure()
{
  for (auto & detector : motor_motion_detectors_) {detector.reset();}
  for (auto & detector : joint_motion_detectors_) {detector.reset();}
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
  active_.store(true);
}

void RealBackend::deactivate() noexcept
{
  active_.store(false);
  control_permitted_.store(false);
  const auto stopped = set_motion_armed(false, 0U);
  if (!stopped.success) {log_write_reject("deactivate: %s", stopped.message.c_str());}
}

void RealBackend::cleanup() noexcept
{
  if (axis_test_active_) {stop_axis_test("stopped: process shutdown");}
  deactivate();
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

  const auto feedback_hold_epoch = stop_hold_epoch_.load();
  try {
    const auto generation = driver_->connection_generation();
    if (generation != last_read_generation_) {
      control_permitted_.store(false);
      disarm_motion(0U);
      for (auto & detector : motor_motion_detectors_) {detector.reset();}
      for (auto & detector : joint_motion_detectors_) {detector.reset();}
      position_initialized_ = false;
      last_read_generation_ = generation;
    }
    const auto snapshot = driver_->snapshot();
    const auto now = std::chrono::steady_clock::now();
    const auto seconds = [](const auto time) {
        return std::chrono::duration<double>(time.time_since_epoch()).count();
      };
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
    sample.software_zero_id = calibration_.software_zero_id;
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
    pi_arm_transmission::Vector6 output_deadbands{};
    pi_arm_transmission::Vector6 joint_deadbands{};
    std::array<double, kAxisCount> angle_times{}, speed_times{};
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
      // Two 0.01-degree protocol encoder ticks, converted with the actual
      // reducer ratio. Do not round positions or filter the published speed.
      output_deadbands[index] = std::abs(protocol_velocity_to_output_rad_s(0.02, actuator));
      angle_times[index] = seconds(state.angle_update);
      speed_times[index] = seconds(state.status2_update);
      motor_in_motion[index] = motor_motion_detectors_[index].update(
        sample.raw_output_positions_rad[index], angle_times[index],
        output_velocities[index], speed_times[index], seconds(now), motor_fresh[index],
        output_deadbands[index], config_.moving_speed_threshold_rad_s);
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
        // Propagate encoder resolution through the local inverse Jacobian,
        // including wrist channel swaps and cross-axis coupling. Sum absolute
        // contributions so opposite signs cannot cancel the noise allowance.
        for (std::size_t motor = 0; motor < kAxisCount; ++motor) {
          pi_arm_transmission::Vector6 basis{}, mapped{};
          basis[motor] = output_deadbands[motor];
          const auto resolution_status = kinematics_.output_to_joint_velocity(
            last_joint_positions_rad_, basis, mapped);
          for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
            joint_deadbands[joint] += resolution_status ? std::abs(mapped[joint]) : nan;
          }
        }
      }
    }
    position_initialized_ = sample.kinematics_valid;

    for (std::size_t joint_index = 0; joint_index < kAxisCount; ++joint_index) {
      bool enabled = true;
      bool has_error = false;
      bool fresh = sample.kinematics_valid;
      std::int32_t error_code = 0;
      double age = 0.0;
      double angle_time = seconds(now), speed_time = seconds(now);
      for (std::size_t motor_index = 0; motor_index < kAxisCount; ++motor_index) {
        if (!joint_depends_on_motor(joint_index, motor_index, kinematics_.parameters().swap_wrist_channels)) {
          continue;
        }
        enabled = enabled && motor_enabled[motor_index];
        has_error = has_error || motor_error[motor_index] || output_limit_error[motor_index];
        fresh = fresh && motor_fresh[motor_index];
        error_code |= motor_error_codes[motor_index];
        age = std::max(age, motor_ages[motor_index]);
        angle_time = std::min(angle_time, angle_times[motor_index]);
        speed_time = std::min(speed_time, speed_times[motor_index]);
      }
      sample.enabled[joint_index] = enabled;
      sample.has_error[joint_index] = has_error;
      sample.fresh[joint_index] = fresh;
      sample.error_codes[joint_index] = error_code;
      sample.ages_sec[joint_index] = age;
      sample.in_motion[joint_index] = joint_motion_detectors_[joint_index].update(
        sample.positions_rad[joint_index], angle_time, sample.velocities_rad_s[joint_index],
        speed_time, seconds(now), fresh, joint_deadbands[joint_index],
        config_.moving_speed_threshold_rad_s);
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
    const bool stop_failed = stop_status == pi_arm_can::MotionStopStatus::FAILED || stop_hold_failed_.load();
    sample.motion_stop_pending = stopping;
    sample.motion_stop_failed = stop_failed;
    if (stop_failed) {
      sample.error_message = stop_hold_failed_.load() ?
        "HOLD UNCONFIRMED: position holding could not be verified; retry Stop; motion is inhibited" :
        "STOP UNCONFIRMED: retry Stop; motion is inhibited";
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
    sample.control_permitted = active_.load() && sample.connected &&
      generation == driver_->connection_generation() && !sample.teaching_active &&
      config_.allow_real_control && calibration_.valid && sample.kinematics_valid &&
      all_enabled && all_fresh && !any_motor_error && !stopping && !stop_failed;

    feedback_generation_.store(generation);
    control_permitted_.store(sample.control_permitted);
    const bool commissioning_hold = config_.commissioning && stop_hold_active_.load() &&
      all_fresh && !any_motor_error && !stop_failed &&
      std::all_of(axis_test_commands_.begin(), axis_test_commands_.end(), [&](const auto & command) {
        const auto & state = snapshot.at(command.motor_id);
        return state.motor_state == 0x00U || state.motor_state == 0x30U;
      });
    if (!sample.control_permitted && !axis_test_active_ && !commissioning_hold &&
      !(sample.teaching_active && !motion_write_gate_.armed())) {
      // Feedback, calibration and kinematics validity are independent of an
      // accepted motion lease. Losing any prerequisite revokes the lease and
      // returns the CAN driver to read-only polling.
      disarm_motion(0U, false, feedback_hold_epoch);
    }
    sample.driver_mode = pi_arm_can::to_string(driver_->mode());
    sample.motor_test_permitted = config_.commissioning && !axis_test_active_ &&
      !axis_test_stop_failed_ && axis_test_raw_ready(true);
    sample.joint_test_permitted = sample.motor_test_permitted && sample.calibrated &&
      sample.kinematics_valid && all_enabled;
    return true;
  } catch (const std::exception & error) {
    control_permitted_.store(false);
    disarm_motion(0U, false, feedback_hold_epoch);
    sample.error_message = error.what();
    return false;
  }
}

MotionArmResult RealBackend::disarm_motion(
  const std::uint64_t lease_id, const bool retry_stop, const std::uint64_t feedback_hold_epoch) noexcept
{
  std::lock_guard<std::mutex> lock(motion_mutex_);
  if (feedback_hold_epoch != 0U && feedback_hold_epoch != stop_hold_epoch_.load()) {
    return {true, motion_write_gate_.armed(), "feedback predates stop/hold transition; recheck next sample"};
  }
  // 用户停止已接管位置保持时，旧任务的清理视为完成；故障、示教和退出用零租约显式释放。
  if (lease_id != 0U && stop_hold_active_.load() && !motion_write_gate_.armed()) {
    return {true, false, "obsolete trajectory cleanup ignored; stop position remains held"};
  }
  if (!motion_write_gate_.disarm(lease_id)) {
    return {false, true, "motion lease does not match the active lease"};
  }
  stop_hold_active_.store(false);
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

MotionArmResult RealBackend::stop_and_hold(const std::vector<std::uint8_t> & motor_ids) noexcept
{
  std::unique_lock<std::mutex> lock(motion_mutex_);
  if (!driver_ || config_.read_only || motor_ids.empty()) {
    return {false, motion_write_gate_.armed(), "writable CAN feedback required for stop-and-hold"};
  }
  if (teaching_active_.load()) {
    return {true, false, "drag teaching remains active; position loop intentionally released"};
  }
  ++stop_hold_epoch_;
  const auto fail = [&](const std::string & message) {
      stop_hold_failed_.store(true);
      stop_hold_active_.store(false);
      ++stop_hold_epoch_;
      lock.unlock();
      // 保持条件失效时改走故障停止，并保留本次保持失败的结果。
      set_motion_armed(false, 0U);
      return MotionArmResult{false, false, "HOLD UNCONFIRMED: " + message};
    };
  try {
    motion_write_gate_.disarm(0U);
    if (driver_->motion_stop_status() == pi_arm_can::MotionStopStatus::FAILED ||
      driver_->motion_stop_status() == pi_arm_can::MotionStopStatus::PENDING)
    {
      if (!driver_->stop_motors(motor_ids)) {return fail("previous stop is unconfirmed");}
    }
    // Purge and fence old controller batches BEFORE capturing the hold target.
    if (!driver_->set_mode(pi_arm_can::DriverMode::MONITORING)) {
      return fail("cannot close the previous command stream");
    }
    const auto generation = driver_->connection_generation();
    const auto snapshot = driver_->snapshot();
    const auto now = std::chrono::steady_clock::now();
    const bool all_off = std::all_of(motor_ids.begin(), motor_ids.end(), [&](const auto id) {
        const auto & state = snapshot.at(id);
        return state.status1_valid && age_seconds(state.status1_update, now) <= config_.state_stale_timeout_sec &&
               state.motor_state != 0x00U && state.motor_state != 0x30U;
      });
    if (all_off) {
      // 全部电机已关闭时确认停止应答，并保持关闭状态。
      if (!driver_->stop_motors(motor_ids)) {return fail("disabled motor stop ACK missing");}
      normal_motion_sent_ = false;
      stop_hold_active_.store(false);
      stop_hold_failed_.store(false);
      ++stop_hold_epoch_;
      return {true, false, "motors remain disabled; no position hold enabled"};
    }
    if (!config_.commissioning && (!config_.allow_real_control || !calibration_.valid ||
      !position_initialized_.load() || reboot_reference_uncertain_.load()))
    {
      return fail("normal control reference/kinematics unavailable");
    }
    std::vector<pi_arm_can::PositionCommand> commands;
    for (const auto & actuator : config_.actuators) {
      if (std::find(motor_ids.begin(), motor_ids.end(), actuator.motor_id) == motor_ids.end()) {continue;}
      const auto & state = snapshot.at(actuator.motor_id);
      if (!state.angle_valid || !state.status1_valid || !state.status2_valid ||
        !std::isfinite(state.protocol_angle_deg) ||
        age_seconds(state.angle_update, now) > 0.1 || age_seconds(state.status2_update, now) > 0.1 ||
        age_seconds(state.status1_update, now) > config_.state_stale_timeout_sec ||
        state.error_state != 0 || (state.motor_state != 0x00U && state.motor_state != 0x30U))
      {
        return fail("fresh enabled, error-free motor feedback required");
      }
      commands.push_back({actuator.motor_id, state.protocol_angle_deg,
        std::max(1.0, std::floor(std::abs(output_velocity_to_protocol_dps(0.017453292519943295, actuator))))});
    }
    if (stop_hold_active_.load() && !stop_hold_failed_.load()) {
      return {true, false, "stop position already held"};  // 重复停止沿用已确认目标，保持固定参考。
    }
    // 用 A4 直接接替轨迹目标，使位置闭环保持连续；应答和新鲜静止反馈共同确认保持结果。
    if (!driver_->hold_positions(commands, generation)) {return fail("captured position hold was not acknowledged/settled");}
    normal_motion_sent_ = true;
    stop_hold_failed_.store(false);
    stop_hold_active_.store(true);
    ++stop_hold_epoch_;
    return {true, false, "motion stopped; captured encoder positions held"};
  } catch (const std::exception & error) {
    return fail(error.what());
  }
}

MotionArmResult RealBackend::set_motion_armed(
  const bool armed, const std::uint64_t lease_id, const double max_output_velocity, const bool hold_position) noexcept
{
  // Feedback remains readable while inactive, but cannot reactivate motion.
  if ((armed || hold_position) && !active_.load()) {
    return {false, false, "backend is inactive; activate before arming or holding"};
  }
  if (hold_position && !armed && lease_id == 0U) {
    std::vector<std::uint8_t> ids;
    for (const auto & actuator : config_.actuators) {ids.push_back(actuator.motor_id);}
    return stop_and_hold(ids);
  }
  if (hold_position) {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (armed || lease_id == 0U || !motion_write_gate_.armed() ||
      motion_write_gate_.active_lease_id() != lease_id || !control_permitted_.load() ||
      (driver_ && motion_generation_.load() != driver_->connection_generation()) ||
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
      // 起点已到达目标时，JTC 可能直接完成；在此补发 A4，建立当前位置保持。
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
      if (!driver_->hold_positions(commands, motion_generation_.load())) {return {false, true, "current-position hold not confirmed"};}
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
    reboot_reference_uncertain_.load() || stop_hold_failed_.load()) {
    return {false, false, "real-control prerequisites are not satisfied"};
  }
  const auto generation = feedback_generation_.load();
  if (!driver_ || !driver_->running() || generation != driver_->connection_generation() ||
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
  motion_generation_.store(generation);
  stop_hold_active_.store(false);
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
  if (!active_.load()) {return true;}
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
  if (!control_permitted_.load() || motion_generation_.load() != driver_->connection_generation()) {
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
  const auto generation = motion_generation_.load();
  pi_arm_transmission::Vector6 recovery_start{};
  double output_speed_limit = 0.0;
  {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    recovery_start = recovery_start_;
    output_speed_limit = output_speed_limit_rad_s_;
  }
  if (!motion_write_gate_.permit_write(positions_rad, velocities_rad_s)) {
    // 等待本次租约下的新目标；捕获的空闲/保持命令在此跳过发送。
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
      // JTC 采样位置决定轨迹时序，A4 速度字段使用输出轴上限。
      // 若改用向下取整的瞬时前馈速度，会累积跟随延迟；零值可能解除电机限速。
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
      !active_.load() || !control_permitted_.load()) {return true;}
    if (!driver_->set_mode(pi_arm_can::DriverMode::CONTROL)) {return true;}
    if (!driver_->enqueue_position_batch(commands, generation)) {
      motion_lock.unlock();
      if (!motion_write_gate_.armed() ||
        driver_->mode() == pi_arm_can::DriverMode::MONITORING ||
        generation != driver_->connection_generation() ||
        driver_->mode() == pi_arm_can::DriverMode::FAULT)
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
  const auto generation = driver_->connection_generation();
  if (operation == pi_arm_can::ManagementOperation::MOTOR_REBOOT && motor_id <= 0) {
    return {false, 1004, "reboot requires one explicit motor_id"};
  }
  if (operation == pi_arm_can::ManagementOperation::MOTOR_RUN && reboot_reference_uncertain_.load()) {
    return {false, 2002, "reboot position reference unverified; enabling is inhibited"};
  }
  const auto ids = selected_motor_ids(config_, motor_id);
  if (ids.empty()) {
    return {false, 1, "motor_id is not configured (use 0 for all motors)"};
  }
  if (operation == pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS) {
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      if (config_.read_only || motion_write_gate_.armed() ||
        driver_->mode() != pi_arm_can::DriverMode::MONITORING ||
        driver_->motion_stop_status() == pi_arm_can::MotionStopStatus::PENDING ||
        driver_->motion_stop_status() == pi_arm_can::MotionStopStatus::FAILED ||
        stop_hold_failed_.load() || axis_test_stop_failed_)
      {
        return {false, 2001, "clear errors requires idle writable CAN and no unconfirmed stop"};
      }
    }
    const auto snapshot = driver_->snapshot();
    const auto now = std::chrono::steady_clock::now();
    for (const auto & actuator : config_.actuators) {
      const auto & state = snapshot.at(actuator.motor_id);
      if (state.status2_valid && age_seconds(state.status2_update, now) <= config_.state_stale_timeout_sec &&
        state.protocol_speed_dps != 0.0) {
        return {false, 2001, "clear errors is rejected while a motor is moving"};
      }
      if (std::find(ids.begin(), ids.end(), actuator.motor_id) == ids.end()) {continue;}
      if (!state.angle_valid || !state.status1_valid || !state.status2_valid ||
        std::max({age_seconds(state.angle_update, now), age_seconds(state.status1_update, now),
          age_seconds(state.status2_update, now)}) > config_.state_stale_timeout_sec ||
        state.protocol_speed_dps != 0.0 || state.stationary_samples < 3U)
      {
        return {false, 2001, "selected motor feedback must be fresh and stationary"};
      }
    }
    // 清错与其他管理服务串行执行，并保持当前运动租约、位置目标和电机电源状态。
    if (!driver_->execute_management(operation, ids, std::chrono::milliseconds(1000), generation)) {
      return {false, 2002, "clear error state was not confirmed: " + driver_->last_error()};
    }
    return {true, 0, "selected motor errors cleared and confirmed by feedback"};
  }
  // 进入电源管理或重启流程前先撤销运动租约，再排入管理帧。
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
  {
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
    // 管理入口已确认整臂静止。保存重启前的角度，供恢复后核验多圈参考。
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
    if (!driver_->execute_management(operation, ids, std::chrono::milliseconds(1000), generation)) {
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
  if (!driver_->execute_management(operation, ids, std::chrono::milliseconds(1000), generation)) {
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
  if (!active_.load() || config_.read_only || config_.commissioning || !config_.allow_real_control || !driver_) {
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
  const auto generation = driver_->connection_generation();
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
  if (!driver_->hold_positions(commands, generation)) {
    return {false, 2002, "failed to confirm current-position hold; teaching remains active"};
  }
  {std::lock_guard<std::mutex> lock(motion_mutex_); normal_motion_sent_ = true;}
  teaching_active_.store(false);
  return {true, 0, "drag teaching ended; motors holding captured current positions"};
}

}  // namespace pi_arm_hardware
