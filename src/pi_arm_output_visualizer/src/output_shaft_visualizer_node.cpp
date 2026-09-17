#include "pi_arm_output_visualizer/output_shaft_mapper.hpp"

#include "pi_arm_transmission/config_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace pi_arm_output_visualizer
{

class OutputShaftVisualizerNode final : public rclcpp::Node
{
public:
  OutputShaftVisualizerNode()
  : Node("pi_arm_output_shaft_visualizer")
  {
    const auto config_path = declare_parameter<std::string>("cable_kinematics_file", "");
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/pi_arm/model_output_shaft_states");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/joint_states");

    if (config_path.empty()) {
      throw std::invalid_argument("parameter 'cable_kinematics_file' must not be empty");
    }
    if (input_topic.empty() || output_topic.empty()) {
      throw std::invalid_argument("input_topic and output_topic must not be empty");
    }

    mapper_ = std::make_unique<OutputShaftMapper>(
      pi_arm_transmission::load_cable_kinematics_config(config_path));
    publisher_ = create_publisher<sensor_msgs::msg::JointState>(output_topic, rclcpp::QoS(10));
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        publish_joint_state(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Visualizing six model-output shaft angles in radians from '%s' on '%s'; no CAN hardware is used",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  void publish_joint_state(const sensor_msgs::msg::JointState & input)
  {
    sensor_msgs::msg::JointState output;
    std::string error;
    if (!mapper_->map(input, output, error)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignored output-shaft state: %s", error.c_str());
      return;
    }
    if (output.header.stamp.sec == 0 && output.header.stamp.nanosec == 0U) {
      output.header.stamp = now();
    }
    publisher_->publish(std::move(output));
  }

  std::unique_ptr<OutputShaftMapper> mapper_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

}  // namespace pi_arm_output_visualizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<pi_arm_output_visualizer::OutputShaftVisualizerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("pi_arm_output_shaft_visualizer"),
      "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
