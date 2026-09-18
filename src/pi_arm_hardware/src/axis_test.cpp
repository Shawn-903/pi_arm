#include "pi_arm_hardware/backend.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace pi_arm_hardware
{
namespace
{
constexpr double kRad = 3.14159265358979323846 / 180.0;
// Raw motor tests use reducer-output degrees, before direction/cable mapping.
constexpr double kMotorStepLimitDeg = 5.0;
constexpr double kMotorSessionLimitDeg = 30.0;
constexpr double kMotorSpeedLimitDegS = 10.0;
constexpr double kJointStepLimitDeg = 0.5;
constexpr double kJointSessionLimitDeg = 2.0;
constexpr double kJointSpeedLimitDegS = 10.0;
}

bool RealBackend::axis_test_raw_ready(const bool require_still) const
{
  if (reboot_reference_uncertain_.load()) {return false;}
  if (!driver_ || !driver_->running() || config_.read_only || !config_.commissioning ||
    (driver_->mode() != pi_arm_can::DriverMode::MONITORING &&
    driver_->mode() != pi_arm_can::DriverMode::CONTROL)) {return false;}
  const auto now = std::chrono::steady_clock::now();
  for (const auto & actuator : config_.actuators) {
    const auto state = driver_->motor_state(actuator.motor_id);
    if (!state.angle_valid || !state.status1_valid || !state.status2_valid ||
      !std::isfinite(state.protocol_angle_deg) || state.error_state != 0 ||
      std::chrono::duration<double>(now - state.angle_update).count() > 0.3 ||
      std::chrono::duration<double>(now - state.status1_update).count() > 0.3 ||
      std::chrono::duration<double>(now - state.status2_update).count() > 0.3 ||
      (require_still && (state.protocol_speed_dps != 0.0 || state.stationary_samples < 3)))
    {return false;}
  }
  return true;
}

ManagementResult RealBackend::start_axis_test(
  const std::string & space, const int axis_id, const double delta_deg, const double speed_deg_s)
{
  if (axis_test_active_) {return {false, 2003, "another axis test is active"};}
  if (axis_test_stop_failed_) {return {false, 2001, "previous stop not acknowledged; retry Stop first"};}
  if (!axis_test_raw_ready(true)) {
    return {false, 2001, "axis test requires commissioning mode and six fresh, error-free, stationary motors"};
  }
  const bool motor_test = space == "motor";
  const double step_limit = motor_test ? kMotorStepLimitDeg : kJointStepLimitDeg;
  const double session_limit = motor_test ? kMotorSessionLimitDeg : kJointSessionLimitDeg;
  const double speed_limit = motor_test ? kMotorSpeedLimitDegS : kJointSpeedLimitDegS;
  if ((space != "motor" && space != "joint") || axis_id < 1 || axis_id > 6 ||
    !std::isfinite(delta_deg) || std::abs(delta_deg) < 0.01 || std::abs(delta_deg) > step_limit ||
    !std::isfinite(speed_deg_s) || speed_deg_s < 0.05 || speed_deg_s > speed_limit)
  {
    return {false, 1004, motor_test ?
      "motor output step must be +/-0.01..5 deg; output speed 0.05..10 deg/s; CAN ID 1..6" :
      "joint step must be +/-0.01..0.5 deg; speed 0.05..10 deg/s; axis 1..6"};
  }
  const auto joint_index = static_cast<size_t>(axis_id - 1);
  const auto selected_motor_index = kinematics_.parameters().swap_wrist_channels &&
    (joint_index == 3U || joint_index == 4U) ? 7U - joint_index : joint_index;
  const auto snapshot = driver_->snapshot();
  std::vector<double> origin;
  for (const auto & actuator : config_.actuators) {
    origin.push_back(snapshot.at(actuator.motor_id).protocol_angle_deg);
  }
  auto target = origin;
  double selected_protocol_speed_cap = 0.0;
  if (space == "motor") {
    const auto found = std::find_if(config_.actuators.begin(), config_.actuators.end(),
      [axis_id](const auto & actuator) {return actuator.motor_id == axis_id;});
    if (found == config_.actuators.end()) {return {false, 1004, "CAN ID not configured"};}
    const auto index = std::distance(config_.actuators.begin(), found);
    // Protocol-positive output shaft angle: no motor_direction, calibration or cable mapping.
    target[index] += delta_deg * found->protocol_angle_ratio;
  } else {
    HardwareSample sample;
    if (!read(sample) || !sample.calibrated || !sample.kinematics_valid ||
      sample.positions_rad.size() != 6 ||
      !std::all_of(sample.motor_enabled.begin(), sample.motor_enabled.end(), [](bool v) {return v;}))
    {
      return {false, 2001, "joint test requires valid calibration/kinematics and all motors enabled"};
    }
    pi_arm_transmission::Vector6 joints{}, outputs{};
    for (size_t i = 0; i < 6; ++i) {
      joints[i] = sample.positions_rad[i];
    }
    pi_arm_transmission::Matrix6 start_jacobian{}, end_jacobian{};
    kinematics_.jacobian(joints, start_jacobian);
    joints[axis_id - 1] += delta_deg * kRad;
    kinematics_.jacobian(joints, end_jacobian);
    const size_t selected = static_cast<size_t>(axis_id - 1);
    // The chord axes change their velocity gain along the path. Use the
    // smaller endpoint gain so the chosen joint's speed limit is not raised.
    selected_protocol_speed_cap = speed_deg_s * config_.actuators[selected_motor_index].protocol_angle_ratio *
      std::min(std::abs(start_jacobian[selected_motor_index][selected]),
      std::abs(end_jacobian[selected_motor_index][selected]));
    if (!kinematics_.joint_to_output_position(joints, outputs)) {
      return {false, 1004, "joint-to-output mapping failed"};
    }
    for (size_t i = 0; i < 6; ++i) {
      target[i] = model_output_position_to_protocol_deg(
        outputs[i], calibration_.software_output_offsets_rad[i], config_.actuators[i]);
    }
  }
  // Validate the actual 0.01-degree targets sent on the wire, not unencoded floats.
  for (auto & value : target) {value = std::round(value * 100.0) / 100.0;}
  if (space == "joint") {
    // No synchronized start/progress guarantee exists for independent A4 motors.
    // Every asynchronous monotone combination lies in this output-coordinate
    // box. For this triangular linear/asin cable model extrema occur at corners.
    for (unsigned corner = 0; corner < 64; ++corner) {
      pi_arm_transmission::Vector6 output{}, recovered{};
      for (size_t i = 0; i < 6; ++i) {
        output[i] = protocol_position_to_model_output_rad(
          (corner & (1U << i)) ? target[i] : origin[i],
          calibration_.software_output_offsets_rad[i], config_.actuators[i]);
      }
      if (!kinematics_.check_output_position_limits(output) ||
        !kinematics_.output_to_joint_position(output, recovered))
      {return {false, 1004, "asynchronous motor path exceeds output/inverse limits"};}
      for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(recovered[i]) || recovered[i] < config_.joint_lower_limits_rad[i] ||
          recovered[i] > config_.joint_upper_limits_rad[i])
        {return {false, 1004, "asynchronous motor path exceeds joint limit: " +
          config_.actuators[i].joint_name + "; reduce step or move away from limit"};}
      }
      pi_arm_transmission::Matrix6 encoded_jacobian{};
      kinematics_.jacobian(recovered, encoded_jacobian);
      const size_t selected = static_cast<size_t>(axis_id - 1);
      selected_protocol_speed_cap = std::min(selected_protocol_speed_cap,
        speed_deg_s * config_.actuators[selected_motor_index].protocol_angle_ratio *
        std::abs(encoded_jacobian[selected_motor_index][selected]));
    }
  }
  const auto & session_origin = axis_test_session_origin_.empty() ? origin : axis_test_session_origin_;
  double duration = std::max(0.2, std::abs(delta_deg) / speed_deg_s);
  std::vector<pi_arm_can::PositionCommand> commands;
  for (size_t i = 0; i < 6; ++i) {
    const auto & actuator = config_.actuators[i];
    const double output_delta = (target[i] - origin[i]) / actuator.protocol_angle_ratio;
    if (std::abs(output_delta) > step_limit + 1e-9 ||
      std::abs((target[i] - session_origin[i]) / actuator.protocol_angle_ratio) > session_limit + 1e-9)
    {return {false, 1004, motor_test ?
      "output step exceeds 5 deg or session displacement exceeds 30 deg" :
      "output step exceeds 0.5 deg or session displacement exceeds 2 deg"};}
    duration = std::max(duration, std::abs(output_delta) / speed_limit);
  }
  double encoded_duration = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    const auto & actuator = config_.actuators[i];
    if (std::abs(target[i] - origin[i]) < 0.005) {continue;}
    const auto & state = snapshot.at(actuator.motor_id);
    if (state.motor_state != 0x00U && state.motor_state != 0x30U) {
      return {false, 2001, "selected/coupled motor is not enabled"};
    }
    double speed_cap = std::min(std::abs(target[i] - origin[i]) / duration,
      actuator.protocol_angle_ratio * speed_limit);
    if (space == "motor") {
      speed_cap = std::min(speed_cap, actuator.protocol_angle_ratio * speed_deg_s);
    } else if (i == selected_motor_index) {
      speed_cap = std::min(speed_cap, selected_protocol_speed_cap);
    }
    const double encoded_speed = std::floor(speed_cap);
    if (encoded_speed < 1.0) {
      return {false, 1004, "speed below protocol resolution at CAN" +
        std::to_string(actuator.motor_id) + "; minimum output speed=" +
        std::to_string(1.0 / actuator.protocol_angle_ratio) +
        " deg/s; choose a representable speed (joint tests include coupled motors)"};
    }
    encoded_duration = std::max(encoded_duration, std::abs(target[i] - origin[i]) / encoded_speed);
    commands.push_back({actuator.motor_id, target[i], encoded_speed});
  }
  if (commands.empty()) {return {false, 1004, "step smaller than protocol resolution"};}
  if (!driver_->set_mode(pi_arm_can::DriverMode::CONTROL)) {
    return {false, 2002, "CAN driver cannot enter CONTROL"};
  }
  axis_test_commands_ = commands;
  axis_test_origin_protocol_ = origin;
  axis_test_target_protocol_ = target;
  if (axis_test_session_origin_.empty()) {axis_test_session_origin_ = origin;}
  axis_test_space_ = space;
  axis_test_started_ = axis_test_heartbeat_ = std::chrono::steady_clock::now();
  axis_test_timeout_sec_ = encoded_duration + 2.0;
  axis_test_active_ = true;
  ++axis_test_id_;
  axis_test_status_ = "running: " + space + " axis=" + std::to_string(axis_id) +
    " delta_deg=" + std::to_string(delta_deg);
  if (!driver_->enqueue_position_batch(commands)) {
    stop_axis_test("failed: CAN position enqueue rejected");
    return {false, 2002, axis_test_status_};
  }
  return {true, 0, axis_test_status_};
}

bool RealBackend::heartbeat_axis_test(const std::uint64_t test_id)
{
  if (!axis_test_active_ || test_id != axis_test_id_) {return false;}
  axis_test_heartbeat_ = std::chrono::steady_clock::now();
  return true;
}

ManagementResult RealBackend::stop_axis_test(const std::string & reason)
{
  if (!axis_test_active_ && !axis_test_stop_failed_) {return {true, 0, "no active axis test"};}
  std::vector<std::uint8_t> ids;
  for (const auto & command : axis_test_commands_) {ids.push_back(command.motor_id);}
  const bool stopped = driver_ && driver_->stop_motors(ids);
  axis_test_active_ = false;
  axis_test_stop_failed_ = !stopped;
  axis_test_status_ = reason + (stopped ? " (0x81 acknowledged)" : " (STOP UNCONFIRMED)");
  return {stopped, stopped ? 0 : 2002, axis_test_status_};
}

void RealBackend::update_axis_test(const HardwareSample & sample)
{
  if (!axis_test_active_) {return;}
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - axis_test_heartbeat_).count() > 0.8) {
    stop_axis_test("stopped: browser heartbeat expired"); return;
  }
  if (!axis_test_raw_ready(false) ||
    (axis_test_space_ == "joint" && (!sample.calibrated || !sample.kinematics_valid)))
  {stop_axis_test("failed: feedback unavailable or fault"); return;}
  if (std::chrono::duration<double>(now - axis_test_started_).count() > axis_test_timeout_sec_) {
    stop_axis_test("failed: axis test timed out"); return;
  }
  bool reached = true;
  for (size_t i = 0; i < config_.actuators.size(); ++i) {
    const auto & actuator = config_.actuators[i];
    const auto state = driver_->motor_state(actuator.motor_id);
    const auto command = std::find_if(axis_test_commands_.begin(), axis_test_commands_.end(),
      [&](const auto & item) {return item.motor_id == actuator.motor_id;});
    if (axis_test_space_ == "joint" &&
      (state.protocol_angle_deg < std::min(axis_test_origin_protocol_[i], axis_test_target_protocol_[i]) - 1e-8 ||
      state.protocol_angle_deg > std::max(axis_test_origin_protocol_[i], axis_test_target_protocol_[i]) + 1e-8))
    {stop_axis_test("failed: motor left the validated asynchronous path"); return;}
    const double displacement = std::abs(state.protocol_angle_deg - axis_test_origin_protocol_[i]) /
      actuator.protocol_angle_ratio;
    const double requested_displacement = std::abs(axis_test_target_protocol_[i] -
      axis_test_origin_protocol_[i]) / actuator.protocol_angle_ratio;
    const double session_displacement = std::abs(state.protocol_angle_deg -
      axis_test_session_origin_[i]) / actuator.protocol_angle_ratio;
    const double session_limit = axis_test_space_ == "motor" ?
      kMotorSessionLimitDeg : kJointSessionLimitDeg;
    if (displacement > requested_displacement + 0.05 ||
      session_displacement > session_limit + 0.05 ||
      (command == axis_test_commands_.end() && displacement > 0.05))
    {stop_axis_test("failed: unexpected output-shaft displacement"); return;}
    if (command != axis_test_commands_.end()) {
      if (state.motor_state != 0x00U && state.motor_state != 0x30U) {
        stop_axis_test("failed: tested motor became disabled"); return;
      }
      reached = reached && state.angle_update > axis_test_started_ &&
        std::abs(state.protocol_angle_deg - command->protocol_angle_deg) <= 0.02 &&
        state.protocol_speed_dps == 0.0 && state.stationary_samples >= 3;
    }
  }
  if (reached) {stop_axis_test("succeeded: target reached");}
}
}  // namespace pi_arm_hardware
