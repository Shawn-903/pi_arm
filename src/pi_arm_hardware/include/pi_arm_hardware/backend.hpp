#pragma once

#include "pi_arm_can/can_driver.hpp"
#include "pi_arm_hardware/actuator_coordinates.hpp"
#include "pi_arm_hardware/calibration.hpp"
#include "pi_arm_hardware/motion_write_gate.hpp"
#include "pi_arm_hardware/motion_detector.hpp"
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
  std::string software_zero_file;
  bool allow_real_control{false};
  bool read_only{false};
  bool commissioning{false};
  std::vector<ActuatorConfig> actuators;
  // 关节限位由插件从 URDF <limit> 读取，模型描述是这些数值的唯一来源。
  std::vector<double> joint_lower_limits_rad;
  std::vector<double> joint_upper_limits_rad;
  // 反馈/起点核验使用的容差；命令和轨迹目标使用标准限位。
  pi_arm_transmission::Vector6 joint_feedback_tolerances_rad{};
  std::vector<double> joint_velocity_limits_rad_s;
  std::chrono::milliseconds poll_interval{20};
  std::size_t status1_poll_divider{5U};
  double moving_speed_threshold_rad_s{0.00017453292519943296};
  double state_stale_timeout_sec{1.0};
};

struct HardwareSample
{
  // 关节状态按主动关节顺序排列；耦合关节的状态由所依赖的电机共同决定。
  std::vector<double> positions_rad;
  std::vector<double> velocities_rad_s;
  std::vector<bool> enabled;
  std::vector<bool> has_error;
  std::vector<bool> in_motion;
  std::vector<bool> fresh;
  std::vector<std::int32_t> error_codes;
  std::vector<double> ages_sec;
  // 原始电机状态按 BackendConfig::actuators 排列；上方关节数组按关节顺序排列。
  // 一个关节的状态由其依赖的多个绳索电机共同决定。
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

// 并发约定：ros2_control 循环调用 read()/write()，辅助服务线程可同时调用
// manage()/set_motion_armed()。共享状态由锁保护，等待管理命令回包前释放
// 控制循环所需的锁，使 read()/write() 能继续运行。
// configure/activate/deactivate/cleanup 由插件按生命周期顺序调用；调用方
// 负责避免它们相互重叠，或与 read()/write()/管理操作并发。
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
  ManagementResult stop_axis_test(const std::string & reason, bool hold_position = false);
  bool heartbeat_axis_test(std::uint64_t test_id);
  void update_axis_test(const HardwareSample & sample);
  bool axis_test_active() const {return axis_test_active_;}
  std::uint64_t axis_test_id() const {return axis_test_id_;}
  std::string axis_test_status() const {return axis_test_status_;}
  // Maintenance mode only, serialized by the monitor's single-threaded executor.
  ManagementResult set_zero_reference(const CalibrationVector6 & reference_joint_rad,
    bool reference_pose_confirmed, HardwareSample & applied_sample);

private:
  MotionArmResult stop_and_hold(const std::vector<std::uint8_t> & motor_ids) noexcept;
  MotionArmResult disarm_motion(std::uint64_t lease_id, bool retry_stop = false,
    std::uint64_t feedback_hold_epoch = 0U) noexcept;
  // CAN 目标提交与租约变更共用此锁，防止停止后漏发旧目标。
  // 等待应答前必须释放该锁；read()/write() 不等待应答。
  std::mutex motion_mutex_;
  std::mutex feedback_mutex_;
  pi_arm_transmission::Vector6 recovery_start_{};
  double output_speed_limit_rad_s_{0.08726646259971647};
  bool normal_motion_sent_{false};
  std::atomic_bool stop_hold_active_{false};
  std::atomic_bool stop_hold_failed_{false};
  std::atomic_uint64_t stop_hold_epoch_{1U};
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
  std::array<MotionDetector, 6> motor_motion_detectors_;
  std::array<MotionDetector, 6> joint_motion_detectors_;
  pi_arm_transmission::CableKinematics kinematics_;
  CalibrationResult calibration_;
  std::atomic_bool reboot_reference_uncertain_{false};
  pi_arm_transmission::Vector6 last_joint_positions_rad_{};
  std::atomic_bool position_initialized_{false};
  std::atomic_bool active_{false};
  std::atomic_bool control_permitted_{false};
  std::atomic_uint64_t feedback_generation_{0U};
  std::atomic_uint64_t motion_generation_{0U};
  std::uint64_t last_read_generation_{0U};
  std::uint64_t axis_test_generation_{0U};
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
  // 控制循环的 read()/write() 与服务线程的 manage() 共用此锁，
  // 串行更新 sample_、stationary_samples_ 和配置/激活标志。
  mutable std::mutex mutex_;
  HardwareSample sample_;
  std::vector<std::size_t> stationary_samples_;
  bool configured_{false};
  bool active_{false};
};

}  // namespace pi_arm_hardware
