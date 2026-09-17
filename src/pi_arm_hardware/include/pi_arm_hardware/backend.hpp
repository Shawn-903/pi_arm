#pragma once

#include "pi_arm_can/can_driver.hpp"
#include "pi_arm_hardware/actuator_coordinates.hpp"
#include "pi_arm_hardware/calibration.hpp"
#include "pi_arm_hardware/motion_write_gate.hpp"
#include "pi_arm_transmission/cable_kinematics.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace pi_arm_hardware
{

struct BackendConfig
{
  std::string can_interface{"can0"};
  std::string cable_kinematics_file;
  std::string calibration_file;
  bool allow_real_control{false};
  bool read_only{false};
  bool commissioning{false};
  std::vector<ActuatorConfig> actuators;
  // Per-joint limits, filled by the hardware plugin from the URDF <limit> tags.
  // The single source of truth for these values is the robot description.
  std::vector<double> joint_lower_limits_rad;
  std::vector<double> joint_upper_limits_rad;
  // Read-side allowance only; never added to command or trajectory limits.
  pi_arm_transmission::Vector6 joint_feedback_tolerances_rad{};
  std::vector<double> joint_velocity_limits_rad_s;
  std::chrono::milliseconds poll_interval{20};
  std::size_t status1_poll_divider{5U};
  double moving_speed_threshold_rad_s{0.00017453292519943296};
  double state_stale_timeout_sec{1.0};
};

struct HardwareSample
{
  std::vector<double> positions_rad;
  std::vector<double> velocities_rad_s;
  std::vector<bool> enabled;
  std::vector<bool> has_error;
  std::vector<bool> in_motion;
  std::vector<bool> fresh;
  std::vector<std::int32_t> error_codes;
  std::vector<double> ages_sec;
  // Raw motor-controller state in BackendConfig::actuators order. Keep this
  // separate from the joint-level arrays above: one joint can depend on more
  // than one motor in the cable transmission.
  std::vector<bool> motor_enabled;
  std::vector<bool> motor_has_error;
  std::vector<bool> motor_in_motion;
  std::vector<bool> motor_fresh;
  std::vector<std::int32_t> motor_error_codes;
  std::vector<double> motor_ages_sec;
  bool calibrated{false};
  bool kinematics_valid{false};
  bool teaching_active{false};
  bool control_permitted{false};
  bool motion_stop_pending{false};
  bool motion_stop_failed{false};
  std::string calibration_id;
  std::vector<double> raw_protocol_positions_deg;
  std::vector<double> raw_output_positions_rad;
  std::vector<double> software_output_offsets_rad;
  std::vector<double> model_output_positions_rad;
  std::string kinematics_error_code{"NONE"};
  std::int32_t kinematics_error_axis{0};
  double kinematics_error_value{0.0};
  std::string kinematics_error_message;
  bool connected{false};
  std::string driver_mode{"OFFLINE"};
  std::string management_operation{"NONE"};
  std::string error_message;
  bool motor_test_permitted{false};
  bool joint_test_permitted{false};
};

struct ManagementResult
{
  bool success{false};
  int code{-1};
  std::string message;
};

struct MotionArmResult
{
  bool success{false};
  bool armed{false};
  std::string message;
};

// Thread-safety contract: read() and write() are called from the ros2_control
// loop while manage() and set_motion_armed() can be called concurrently from
// the auxiliary service thread. Every implementation must keep these paths
// safe without blocking the control loop for the duration of management I/O.
// Lifecycle methods (configure/activate/deactivate/cleanup) are never called
// concurrently with each other or with read()/write()/manage(); the plugin
// guarantees this ordering.
class Backend
{
public:
  virtual ~Backend() = default;
  virtual void configure() = 0;
  virtual void activate() = 0;
  virtual void deactivate() noexcept = 0;
  virtual void cleanup() noexcept = 0;
  virtual bool read(HardwareSample & sample) noexcept = 0;
  virtual bool write(
    const std::vector<double> & positions_rad,
    const std::vector<double> & velocities_rad_s) noexcept = 0;
  virtual ManagementResult manage(
    pi_arm_can::ManagementOperation operation, int motor_id) noexcept = 0;
  virtual ManagementResult set_teaching(bool enabled) noexcept = 0;
  virtual MotionArmResult set_motion_armed(
    bool armed, std::uint64_t lease_id, double max_output_velocity = 0.0, bool hold_position = false) noexcept = 0;
};

class RealBackend final : public Backend
{
public:
  explicit RealBackend(BackendConfig config);
  ~RealBackend() override;

  void configure() override;
  void activate() override;
  void deactivate() noexcept override;
  void cleanup() noexcept override;
  bool read(HardwareSample & sample) noexcept override;
  bool write(
    const std::vector<double> & positions_rad,
    const std::vector<double> & velocities_rad_s) noexcept override;
  ManagementResult manage(
    pi_arm_can::ManagementOperation operation, int motor_id) noexcept override;
  ManagementResult set_teaching(bool enabled) noexcept override;
  MotionArmResult set_motion_armed(
    bool armed, std::uint64_t lease_id, double max_output_velocity = 0.0, bool hold_position = false) noexcept override;

  // Only the dedicated, single-threaded commissioning node may call these.
  ManagementResult start_axis_test(const std::string & space, int axis_id,
    double delta_deg, double speed_deg_s);
  ManagementResult stop_axis_test(const std::string & reason);
  bool heartbeat_axis_test(std::uint64_t test_id);
  void update_axis_test(const HardwareSample & sample);
  bool axis_test_active() const {return axis_test_active_;}
  std::uint64_t axis_test_id() const {return axis_test_id_;}
  std::string axis_test_status() const {return axis_test_status_;}

private:
  MotionArmResult disarm_motion(std::uint64_t lease_id, bool retry_stop = false) noexcept;
  // Serialize the final command submission with lease changes. ACK waiting
  // happens after releasing this mutex, never in read()/write().
  std::mutex motion_mutex_;
  std::mutex feedback_mutex_;
  pi_arm_transmission::Vector6 recovery_start_{};
  double output_speed_limit_rad_s_{0.08726646259971647};
  bool normal_motion_sent_{false};
  bool axis_test_raw_ready(bool require_still) const;
  bool axis_test_active_{false};
  bool axis_test_stop_failed_{false};
  std::uint64_t axis_test_id_{0};
  std::string axis_test_status_{"idle"};
  std::string axis_test_space_;
  std::vector<pi_arm_can::PositionCommand> axis_test_commands_;
  std::vector<double> axis_test_origin_protocol_;
  std::vector<double> axis_test_target_protocol_;
  std::vector<double> axis_test_session_origin_;
  std::chrono::steady_clock::time_point axis_test_started_{};
  std::chrono::steady_clock::time_point axis_test_heartbeat_{};
  double axis_test_timeout_sec_{0.0};
  BackendConfig config_;
  pi_arm_transmission::CableKinematics kinematics_;
  CalibrationResult calibration_;
  std::atomic_bool reboot_reference_uncertain_{false};
  pi_arm_transmission::Vector6 last_joint_positions_rad_{};
  std::atomic_bool position_initialized_{false};
  std::atomic_bool control_permitted_{false};
  std::atomic_bool teaching_active_{false};
  MotionWriteGate motion_write_gate_;
  std::unique_ptr<pi_arm_can::CanDriver> driver_;
};

class MockBackend final : public Backend
{
public:
  explicit MockBackend(BackendConfig config);

  void configure() override;
  void activate() override;
  void deactivate() noexcept override;
  void cleanup() noexcept override;
  bool read(HardwareSample & sample) noexcept override;
  bool write(
    const std::vector<double> & positions_rad,
    const std::vector<double> & velocities_rad_s) noexcept override;
  ManagementResult manage(
    pi_arm_can::ManagementOperation operation, int motor_id) noexcept override;
  ManagementResult set_teaching(bool enabled) noexcept override;
  MotionArmResult set_motion_armed(
    bool armed, std::uint64_t lease_id, double max_output_velocity = 0.0, bool hold_position = false) noexcept override;

private:
  bool hold_mock_current_{false};
  bool valid_motor_id(int motor_id) const noexcept;

  BackendConfig config_;
  pi_arm_transmission::CableKinematics kinematics_;
  pi_arm_transmission::Vector6 output_positions_rad_{};
  pi_arm_transmission::Vector6 output_velocities_rad_s_{};
  // Guards sample_/stationary_samples_/flags: read()/write() run on the control
  // loop while manage() runs on a service thread. Mock operations are cheap, so
  // a single mutex is sufficient and never blocks for long.
  mutable std::mutex mutex_;
  HardwareSample sample_;
  std::vector<std::size_t> stationary_samples_;
  bool configured_{false};
  bool active_{false};
};

}  // namespace pi_arm_hardware
