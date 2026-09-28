#include "pi_arm_hardware/backend.hpp"

#include "hardware_interface/component_parser.hpp"
#include "pi_arm_interfaces/msg/hardware_state.hpp"
#include "pi_arm_interfaces/srv/axis_test.hpp"
#include "pi_arm_interfaces/srv/manage_motor.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pi_arm_hardware
{
namespace
{

constexpr std::size_t kAxisCount = 6U;

template<typename T>
T parse_value(const std::string & text, const std::string & name)
{
  std::istringstream stream(text);
  T value{};
  stream >> value;
  if (!stream || !(stream >> std::ws).eof()) {
    throw std::invalid_argument("invalid value for '" + name + "': " + text);
  }
  return value;
}

std::string parameter_or(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const std::string & fallback)
{
  const auto found = parameters.find(name);
  return found == parameters.end() ? fallback : found->second;
}

const std::string & required_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & owner, const std::string & name)
{
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    throw std::invalid_argument(owner + " is missing required parameter '" + name + "'");
  }
  return found->second;
}

BackendConfig backend_config_from_robot_description(const std::string & robot_description)
{
  const auto resources = hardware_interface::parse_control_resources_from_urdf(robot_description);
  if (resources.size() != 1U) {
    throw std::invalid_argument("robot_description must contain exactly one ros2_control system");
  }
  const auto & info = resources.front();
  if (info.joints.size() != kAxisCount) {
    throw std::invalid_argument("monitor requires exactly six active ros2_control joints");
  }

  BackendConfig config;
  config.can_interface = parameter_or(info.hardware_parameters, "can_interface", "can0");
  config.cable_kinematics_file =
    required_parameter(info.hardware_parameters, "ros2_control hardware", "cable_kinematics_file");
  config.calibration_file =
    required_parameter(info.hardware_parameters, "ros2_control hardware", "calibration_file");
  config.poll_interval = std::chrono::milliseconds{
    parse_value<int>(
      parameter_or(info.hardware_parameters, "state_poll_interval_ms", "20"),
      "state_poll_interval_ms")};
  config.status1_poll_divider = parse_value<std::size_t>(
    parameter_or(info.hardware_parameters, "status1_poll_divider", "5"),
    "status1_poll_divider");
  config.moving_speed_threshold_rad_s = parse_value<double>(
    parameter_or(
      info.hardware_parameters, "moving_speed_threshold_rad_s",
      "0.00017453292519943296"),
    "moving_speed_threshold_rad_s");
  config.state_stale_timeout_sec = parse_value<double>(
    parameter_or(info.hardware_parameters, "state_stale_timeout_sec", "1.0"),
    "state_stale_timeout_sec");
  config.allow_real_control = false;
  config.read_only = true;

  std::unordered_set<int> motor_ids;
  for (const auto & joint : info.joints) {
    const auto limits = info.limits.find(joint.name);
    if (limits == info.limits.end() || !limits->second.has_position_limits ||
      !limits->second.has_velocity_limits || limits->second.max_velocity <= 0.0)
    {
      throw std::invalid_argument("joint '" + joint.name + "' has incomplete URDF limits");
    }
    const int motor_id = parse_value<int>(
      required_parameter(joint.parameters, joint.name, "motor_id"), "motor_id");
    if (motor_id <= 0 || motor_id > 255 || !motor_ids.insert(motor_id).second) {
      throw std::invalid_argument("motor IDs must be unique values in 1..255");
    }
    config.actuators.push_back(
      ActuatorConfig{
        joint.name,
        static_cast<std::uint8_t>(motor_id),
        parse_value<double>(
          required_parameter(joint.parameters, joint.name, "protocol_angle_ratio"),
          "protocol_angle_ratio"),
        parse_value<int>(
          required_parameter(joint.parameters, joint.name, "motor_direction"),
          "motor_direction")});
    config.joint_lower_limits_rad.push_back(limits->second.min_position);
    config.joint_upper_limits_rad.push_back(limits->second.max_position);
    config.joint_feedback_tolerances_rad[config.actuators.size() - 1U] =
      parse_value<double>(parameter_or(
        joint.parameters, "feedback_position_tolerance_rad", "0"),
        "feedback_position_tolerance_rad");
    config.joint_velocity_limits_rad_s.push_back(limits->second.max_velocity);
  }
  return config;
}

}  // namespace

class HardwareMonitorNode final : public rclcpp::Node
{
public:
  HardwareMonitorNode()
  : Node("pi_arm_hardware_monitor")
  {
    const auto robot_description = declare_parameter<std::string>("robot_description", "");
    const double publish_rate_hz = declare_parameter<double>("state_publish_rate_hz", 10.0);
    if (robot_description.empty()) {
      throw std::invalid_argument("parameter 'robot_description' must not be empty");
    }
    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("state_publish_rate_hz must be positive");
    }

    config_ = backend_config_from_robot_description(robot_description);
    config_.commissioning = declare_parameter<bool>("commissioning", false);
    config_.read_only = !config_.commissioning;
    joint_names_.reserve(config_.actuators.size());
    for (const auto & actuator : config_.actuators) {
      joint_names_.push_back(actuator.joint_name);
    }

    backend_ = std::make_unique<RealBackend>(config_);
    backend_->configure();
    backend_->activate();
    if (config_.commissioning) {
      using AxisTest = pi_arm_interfaces::srv::AxisTest;
      axis_test_service_ = create_service<AxisTest>("/pi_arm/commissioning/axis_test",
        [this](const std::shared_ptr<AxisTest::Request> request,
          std::shared_ptr<AxisTest::Response> response) {
          ManagementResult result;
          if (request->operation == "start") {
            result = backend_->start_axis_test(request->space, request->axis_id,
              request->delta_deg, request->speed_deg_s);
          } else if (request->operation == "heartbeat") {
            const bool ok = backend_->heartbeat_axis_test(request->test_id);
            result = {ok, ok ? 0 : 2001, ok ? "ok" : "axis test is no longer active"};
          } else if (request->operation == "stop") {
            if (request->test_id != 0 && request->test_id != backend_->axis_test_id()) {
              result = {false, 2001, "test ID does not match"};
            } else {result = backend_->stop_axis_test("stopped: operator request", true);}
          } else {result = {false, 1004, "unknown axis-test operation"};}
          response->success = result.success;
          response->code = result.code;
          response->message = result.message;
          response->test_id = backend_->axis_test_id();
          if (request->operation != "heartbeat") {
            RCLCPP_INFO(get_logger(), "axis test: operation=%s space=%s axis=%d delta=%.9g speed=%.9g id=%llu success=%d %s",
              request->operation.c_str(), request->space.c_str(), request->axis_id,
              request->delta_deg, request->speed_deg_s,
              static_cast<unsigned long long>(response->test_id), result.success, result.message.c_str());
          }
        });
      using ManageMotor = pi_arm_interfaces::srv::ManageMotor;
      for (const auto & entry : std::vector<std::pair<std::string, pi_arm_can::ManagementOperation>>{
          {"enable_motor", pi_arm_can::ManagementOperation::MOTOR_RUN},
          {"disable_motor", pi_arm_can::ManagementOperation::MOTOR_OFF},
          {"reset_motor", pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS},
          {"reboot_motor", pi_arm_can::ManagementOperation::MOTOR_REBOOT}})
      {
        management_services_.push_back(create_service<ManageMotor>(
          "/pi_arm/commissioning/" + entry.first,
          [this, operation = entry.second](const std::shared_ptr<ManageMotor::Request> request,
            std::shared_ptr<ManageMotor::Response> response) {
            const auto result = backend_->manage(operation, request->motor_id);
            response->success = result.success; response->code = result.code;
            response->message = result.message;
          }));
      }
    }
    publisher_ = create_publisher<pi_arm_interfaces::msg::HardwareState>(
      "/pi_arm/hardware_state", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      [this]() {publish_sample();});

    RCLCPP_WARN(
      get_logger(),
      "Monitor owns SocketCAN '%s'; mode=%s",
      config_.can_interface.c_str(), config_.commissioning ? "BOUNDED AXIS TESTS" : "READ ONLY");
  }

  ~HardwareMonitorNode() override
  {
    if (backend_) {
      backend_->deactivate();
      backend_->cleanup();
    }
  }

private:
  void publish_sample()
  {
    HardwareSample sample;
    if (!backend_->read(sample)) {
      backend_->stop_axis_test("failed: hardware read failed");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Hardware monitor read failed");
      return;
    }
    backend_->update_axis_test(sample);

    pi_arm_interfaces::msg::HardwareState message;
    message.stamp = now();
    message.connected = sample.connected;
    message.driver_mode = sample.driver_mode;
    message.management_operation = sample.management_operation;
    message.joint_names = joint_names_;
    message.positions = sample.positions_rad;
    message.velocities = sample.velocities_rad_s;
    message.enabled = sample.enabled;
    message.errors = sample.has_error;
    message.moving = sample.in_motion;
    message.fresh = sample.fresh;
    message.error_codes = sample.error_codes;
    message.ages_sec = sample.ages_sec;
    for (const auto & actuator : config_.actuators) {
      message.motor_ids.push_back(static_cast<std::int32_t>(actuator.motor_id));
    }
    message.motor_enabled = sample.motor_enabled;
    message.motor_errors = sample.motor_has_error;
    message.motor_moving = sample.motor_in_motion;
    message.motor_fresh = sample.motor_fresh;
    message.motor_error_codes = sample.motor_error_codes;
    message.motor_ages_sec = sample.motor_ages_sec;
    message.calibrated = sample.calibrated;
    message.kinematics_valid = sample.kinematics_valid;
    message.control_permitted = false;
    message.motion_stop_pending = sample.motion_stop_pending;
    message.motion_stop_failed = sample.motion_stop_failed;
    message.calibration_id = sample.calibration_id;
    message.raw_protocol_positions_deg = sample.raw_protocol_positions_deg;
    message.raw_output_positions_rad = sample.raw_output_positions_rad;
    message.software_output_offsets_rad = sample.software_output_offsets_rad;
    message.model_output_positions_rad = sample.model_output_positions_rad;
    message.kinematics_error_code = sample.kinematics_error_code;
    message.kinematics_error_axis = sample.kinematics_error_axis;
    message.kinematics_error_value = sample.kinematics_error_value;
    message.kinematics_error_message = sample.kinematics_error_message;
    message.diagnostic = sample.error_message;
    message.commissioning = config_.commissioning;
    message.can_interface = config_.can_interface;
    message.motor_test_permitted = sample.motor_test_permitted;
    message.joint_test_permitted = sample.joint_test_permitted;
    message.axis_test_active = backend_->axis_test_active();
    message.axis_test_id = backend_->axis_test_id();
    message.axis_test_status = backend_->axis_test_status();
    for (const auto & actuator : config_.actuators) {
      message.protocol_angle_ratios.push_back(actuator.protocol_angle_ratio);
      message.motor_directions.push_back(actuator.motor_direction);
    }
    publisher_->publish(std::move(message));
  }

  BackendConfig config_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<RealBackend> backend_;
  rclcpp::Publisher<pi_arm_interfaces::msg::HardwareState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<pi_arm_interfaces::srv::AxisTest>::SharedPtr axis_test_service_;
  std::vector<rclcpp::Service<pi_arm_interfaces::srv::ManageMotor>::SharedPtr> management_services_;
};

}  // namespace pi_arm_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<pi_arm_hardware::HardwareMonitorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("pi_arm_hardware_monitor"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
