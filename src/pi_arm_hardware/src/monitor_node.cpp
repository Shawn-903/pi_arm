#include "pi_arm_hardware/backend.hpp"
#include "pi_arm_hardware/hardware_state_message.hpp"

#include "pi_arm_interfaces/msg/hardware_state.hpp"
#include "pi_arm_interfaces/srv/axis_test.hpp"
#include "pi_arm_interfaces/srv/manage_motor.hpp"
#include "pi_arm_interfaces/srv/set_zero_reference.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pi_arm_hardware
{
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
      using SetZeroReference = pi_arm_interfaces::srv::SetZeroReference;
      zero_reference_service_ = create_service<SetZeroReference>(
        "/pi_arm/commissioning/set_zero_reference",
        [this](const std::shared_ptr<SetZeroReference::Request> request,
          std::shared_ptr<SetZeroReference::Response> response) {
          HardwareSample applied;
          const auto result = backend_->set_zero_reference(
            request->reference_joint_positions_rad, request->reference_pose_confirmed, applied);
          response->success = result.success;
          response->code = result.code;
          response->message = result.message;
          if (result.success) {
            std::copy_n(applied.raw_protocol_positions_deg.begin(), 6,
              response->raw_protocol_positions_deg.begin());
            std::copy_n(applied.software_output_offsets_rad.begin(), 6,
              response->software_output_offsets_rad.begin());
          }
          RCLCPP_INFO(get_logger(), "software zero: success=%d %s",
            result.success, result.message.c_str());
        });
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

    auto message = make_hardware_state_message(sample, joint_names_, config_, now());
    message.control_permitted = false;
    message.axis_test_active = backend_->axis_test_active();
    message.axis_test_id = backend_->axis_test_id();
    message.axis_test_status = backend_->axis_test_status();
    publisher_->publish(std::move(message));
  }

  BackendConfig config_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<RealBackend> backend_;
  rclcpp::Publisher<pi_arm_interfaces::msg::HardwareState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<pi_arm_interfaces::srv::AxisTest>::SharedPtr axis_test_service_;
  rclcpp::Service<pi_arm_interfaces::srv::SetZeroReference>::SharedPtr zero_reference_service_;
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
