#include "pi_arm_hardware/pi_arm_system.hpp"
#include "pi_arm_hardware/hardware_state_message.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace pi_arm_hardware
{
namespace
{

template<typename T>
T required_joint_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & joint_name, const std::string & name)
{
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    throw std::invalid_argument(
            "joint '" + joint_name + "' is missing required ros2_control parameter '" + name +
            "'");
  }
  std::istringstream stream(found->second);
  T value{};
  if constexpr (std::is_same_v<T, bool>) {
    stream >> std::boolalpha >> value;
    if (!stream) {
      stream.clear();
      stream.str(found->second);
      int numeric = -1;
      stream >> numeric;
      if (numeric != 0 && numeric != 1) {
        throw std::invalid_argument("invalid boolean hardware parameter '" + name + "'");
      }
      value = numeric == 1;
    }
  } else {
    stream >> value;
  }
  if (!stream || !(stream >> std::ws).eof()) {
    throw std::invalid_argument(
            "joint '" + joint_name + "' has an invalid ros2_control parameter '" + name + "'");
  }
  return value;
}

template<typename T>
T parameter_or(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const T & fallback)
{
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    return fallback;
  }
  std::istringstream stream(found->second);
  T value{};
  if constexpr (std::is_same_v<T, bool>) {
    stream >> std::boolalpha >> value;
    if (!stream) {
      stream.clear();
      stream.str(found->second);
      int numeric = -1;
      stream >> numeric;
      if (numeric != 0 && numeric != 1) {
        throw std::invalid_argument("invalid boolean hardware parameter '" + name + "'");
      }
      value = numeric == 1;
    }
  } else {
    stream >> value;
  }
  if (!stream || !(stream >> std::ws).eof()) {
    throw std::invalid_argument("invalid hardware parameter '" + name + "'");
  }
  return value;
}

std::string string_parameter_or(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const std::string & fallback)
{
  const auto found = parameters.find(name);
  return found == parameters.end() ? fallback : found->second;
}

bool has_exact_interfaces(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces)
{
  if (interfaces.size() != 2U) {
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto & interface : interfaces) {
    names.insert(interface.name);
  }
  return names == std::unordered_set<std::string>{
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
}

}  // namespace

PiArmSystem::~PiArmSystem()
{
  // Stopping the auxiliary node first joins the executor thread, so no service
  // callback can reach the backend once cleanup starts.
  stop_auxiliary_node();
  if (backend_) {
    backend_->cleanup();
  }
}

hardware_interface::CallbackReturn PiArmSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    if (info_.joints.size() != backend_config_.joint_feedback_tolerances_rad.size()) {
      throw std::invalid_argument("pi_arm_hardware requires exactly six active joints");
    }
    joint_names_.clear();
    for (const auto & joint : info_.joints) {
      if (!has_exact_interfaces(joint.command_interfaces) ||
        !has_exact_interfaces(joint.state_interfaces))
      {
        throw std::invalid_argument(
                "every active joint requires position and velocity command/state interfaces");
      }
      joint_names_.push_back(joint.name);
    }
    parse_configuration();

    const std::size_t joint_count = joint_names_.size();
    state_positions_.assign(joint_count, 0.0);
    state_velocities_.assign(joint_count, 0.0);
    command_positions_.assign(joint_count, 0.0);
    command_velocities_.assign(joint_count, 0.0);
    latest_sample_.positions_rad = state_positions_;
    latest_sample_.velocities_rad_s = state_velocities_;
    latest_sample_.enabled.assign(joint_count, false);
    latest_sample_.has_error.assign(joint_count, false);
    latest_sample_.in_motion.assign(joint_count, false);
    latest_sample_.fresh.assign(joint_count, false);
    latest_sample_.error_codes.assign(joint_count, 0);
    latest_sample_.ages_sec.assign(joint_count, std::numeric_limits<double>::infinity());

    if (backend_name_ == "real") {
      backend_ = std::make_unique<RealBackend>(backend_config_);
    } else if (backend_name_ == "mock") {
      backend_ = std::make_unique<MockBackend>(backend_config_);
    } else {
      throw std::invalid_argument("hardware parameter backend must be 'mock' or 'real'");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("PiArmSystem"), "Initialization failed: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

void PiArmSystem::parse_configuration()
{
  const auto & parameters = info_.hardware_parameters;
  backend_name_ = string_parameter_or(parameters, "backend", "real");
  backend_config_.can_interface = string_parameter_or(parameters, "can_interface", "can0");
  backend_config_.cable_kinematics_file =
    string_parameter_or(parameters, "cable_kinematics_file", "");
  backend_config_.calibration_file =
    string_parameter_or(parameters, "calibration_file", "");
  backend_config_.software_zero_file =
    string_parameter_or(parameters, "software_zero_file", "");
  backend_config_.allow_real_control =
    parameter_or<bool>(parameters, "allow_real_control", false);
  if (backend_config_.cable_kinematics_file.empty()) {
    throw std::invalid_argument(
            "hardware parameter 'cable_kinematics_file' must name the installed YAML file");
  }
  backend_config_.poll_interval = std::chrono::milliseconds{
    parameter_or<int>(parameters, "state_poll_interval_ms", 20)};
  backend_config_.status1_poll_divider =
    parameter_or<std::size_t>(parameters, "status1_poll_divider", 5U);
  backend_config_.moving_speed_threshold_rad_s =
    parameter_or<double>(parameters, "moving_speed_threshold_rad_s", 0.00017453292519943296);
  backend_config_.state_stale_timeout_sec =
    parameter_or<double>(parameters, "state_stale_timeout_sec", 1.0);
  state_publish_rate_hz_ = parameter_or<double>(parameters, "state_publish_rate_hz", 10.0);
  node_name_ = string_parameter_or(parameters, "service_node_name", node_name_);

  if (backend_config_.poll_interval.count() <= 0 || backend_config_.status1_poll_divider == 0U ||
    backend_config_.moving_speed_threshold_rad_s < 0.0 ||
    backend_config_.state_stale_timeout_sec <= 0.0 || state_publish_rate_hz_ <= 0.0)
  {
    throw std::invalid_argument("timing and speed hardware parameters are outside valid ranges");
  }

  // Joint limits and the actuator protocol mapping come from ros2_control.
  // Cross-axis cable geometry is loaded from cable_kinematics_file.
  backend_config_.actuators.clear();
  backend_config_.joint_lower_limits_rad.clear();
  backend_config_.joint_upper_limits_rad.clear();
  backend_config_.joint_velocity_limits_rad_s.clear();
  std::unordered_set<int> motor_ids;
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto & joint = info_.joints[index];
    const auto limits = info_.limits.find(joint.name);
    if (limits == info_.limits.end() || !limits->second.has_position_limits ||
      !limits->second.has_velocity_limits || limits->second.max_velocity <= 0.0)
    {
      throw std::invalid_argument(
              "joint '" + joint.name + "' requires position and velocity limits in the URDF");
    }
    backend_config_.joint_lower_limits_rad.push_back(limits->second.min_position);
    backend_config_.joint_upper_limits_rad.push_back(limits->second.max_position);
    backend_config_.joint_feedback_tolerances_rad[index] =
      parameter_or<double>(joint.parameters, "feedback_position_tolerance_rad", 0.0);
    backend_config_.joint_velocity_limits_rad_s.push_back(limits->second.max_velocity);

    const int motor_id = required_joint_parameter<int>(joint.parameters, joint.name, "motor_id");
    if (motor_id <= 0 || motor_id > 0xFF || !motor_ids.insert(motor_id).second) {
      throw std::invalid_argument(
              "joint '" + joint.name + "' needs a unique motor_id between 1 and 255");
    }
    backend_config_.actuators.push_back(
      ActuatorConfig{
        joint.name, static_cast<std::uint8_t>(motor_id),
        required_joint_parameter<double>(
          joint.parameters, joint.name, "protocol_angle_ratio"),
        required_joint_parameter<int>(joint.parameters, joint.name, "motor_direction")});
  }
}

std::vector<hardware_interface::StateInterface> PiArmSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 2U);
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION, &state_positions_[index]);
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_VELOCITY, &state_velocities_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> PiArmSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 2U);
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION, &command_positions_[index]);
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_VELOCITY, &command_velocities_[index]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn PiArmSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    backend_->configure();
    start_auxiliary_node();
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("PiArmSystem"), "Configuration failed: %s", error.what());
    stop_auxiliary_node();
    if (backend_) {
      backend_->cleanup();
    }
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn PiArmSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Join the service executor thread before touching the backend so no manage()
  // call can run concurrently with cleanup().
  stop_auxiliary_node();
  hardware_active_.store(false);
  if (backend_) {backend_->cleanup();}
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiArmSystem::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

hardware_interface::CallbackReturn PiArmSystem::on_error(const rclcpp_lifecycle::State & state)
{
  on_cleanup(state);
  // An initialization failure may leave no backend to configure again.
  return backend_ ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::FAILURE;
}

hardware_interface::CallbackReturn PiArmSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> service_lock(service_mutex_);
  try {
    backend_->activate();
    command_positions_ = state_positions_;
    // Velocity command interfaces start at rest. Initializing them to the
    // positive joint limits manufactures a non-zero idle command and is not a
    // valid controller state.
    command_velocities_.assign(joint_names_.size(), 0.0);
    {
      std::lock_guard<std::mutex> sample_lock(sample_mutex_);
      latest_sample_.connected = true;
    }
    hardware_active_.store(true);
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("PiArmSystem"), "Activation failed: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn PiArmSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> service_lock(service_mutex_);
  backend_->deactivate();
  hardware_active_.store(false);
  {
    std::lock_guard<std::mutex> sample_lock(sample_mutex_);
    // Control loop stops updating samples; clear stale motion so manager does
    // not keep seeing RUNNING from frozen in_motion flags.
    std::fill(
      latest_sample_.velocities_rad_s.begin(), latest_sample_.velocities_rad_s.end(), 0.0);
    std::fill(latest_sample_.in_motion.begin(), latest_sample_.in_motion.end(), false);
    std::fill(latest_sample_.fresh.begin(), latest_sample_.fresh.end(), false);
    latest_sample_.connected = false;
  }
  std::fill(state_velocities_.begin(), state_velocities_.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PiArmSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // The backend implementations are thread-safe against concurrent manage()
  // calls, so the control loop never contends with slow management commands.
  HardwareSample sample;
  if (!backend_->read(sample) || sample.positions_rad.size() != joint_names_.size() ||
    sample.velocities_rad_s.size() != joint_names_.size())
  {
    return hardware_interface::return_type::ERROR;
  }
  state_positions_ = sample.positions_rad;
  state_velocities_ = sample.velocities_rad_s;
  {
    std::lock_guard<std::mutex> sample_lock(sample_mutex_);
    latest_sample_ = std::move(sample);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiArmSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return backend_->write(command_positions_, command_velocities_) ?
         hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

void PiArmSystem::start_auxiliary_node()
{
  if (node_) {
    return;
  }
  node_ = std::make_unique<rclcpp::Node>(node_name_);
  state_publisher_ = node_->create_publisher<pi_arm_interfaces::msg::HardwareState>(
    "/pi_arm/hardware_state", rclcpp::SensorDataQoS());
  create_management_service(
    "/pi_arm/hardware/disable_motor", pi_arm_can::ManagementOperation::MOTOR_OFF);
  create_management_service(
    "/pi_arm/hardware/enable_motor", pi_arm_can::ManagementOperation::MOTOR_RUN);
  create_management_service(
    "/pi_arm/hardware/reset_motor", pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS);
  create_management_service(
    "/pi_arm/hardware/reboot_motor", pi_arm_can::ManagementOperation::MOTOR_REBOOT);
  teaching_service_ = node_->create_service<pi_arm_interfaces::srv::SetTeaching>(
    "/pi_arm/hardware/set_teaching",
    [this](const std::shared_ptr<pi_arm_interfaces::srv::SetTeaching::Request> request,
      std::shared_ptr<pi_arm_interfaces::srv::SetTeaching::Response> response) {
      std::lock_guard<std::mutex> service_lock(service_mutex_);
      const auto result = backend_->set_teaching(request->enabled);
      response->success = result.success; response->code = result.code; response->message = result.message;
    });
  motion_arm_service_ = node_->create_service<SetMotionArmed>(
    "/pi_arm/hardware/set_motion_armed",
    [this](
      const std::shared_ptr<SetMotionArmed::Request> request,
      std::shared_ptr<SetMotionArmed::Response> response)
    {
      std::lock_guard<std::mutex> service_lock(service_mutex_);
      const auto result = backend_->set_motion_armed(
        request->armed, request->lease_id, request->max_output_velocity, request->hold_position);
      response->success = result.success;
      response->armed = result.armed;
      response->message = result.message;
      if (result.success) {
        RCLCPP_INFO(
          node_->get_logger(), "position-write gate armed=%d lease=%llu: %s",
          result.armed ? 1 : 0,
          static_cast<unsigned long long>(request->lease_id), result.message.c_str());
      } else {
        RCLCPP_WARN(
          node_->get_logger(), "position-write gate rejected armed=%d lease=%llu: %s",
          request->armed ? 1 : 0,
          static_cast<unsigned long long>(request->lease_id), result.message.c_str());
      }
    });
  state_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / state_publish_rate_hz_),
    [this]() {publish_hardware_state();});
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_->get_node_base_interface());
  executor_thread_ = std::thread([this]() {executor_->spin();});
}

void PiArmSystem::stop_auxiliary_node() noexcept
{
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  management_services_.clear();
  teaching_service_.reset();
  motion_arm_service_.reset();
  state_timer_.reset();
  state_publisher_.reset();
  if (executor_ && node_) {
    executor_->remove_node(node_->get_node_base_interface());
  }
  executor_.reset();
  node_.reset();
}

void PiArmSystem::create_management_service(
  const std::string & name, const pi_arm_can::ManagementOperation operation)
{
  management_services_.push_back(
    node_->create_service<ManageMotor>(
      name,
      [this, operation](
        const std::shared_ptr<ManageMotor::Request> request,
        std::shared_ptr<ManageMotor::Response> response)
      {
        std::lock_guard<std::mutex> service_lock(service_mutex_);
        {
          std::lock_guard<std::mutex> sample_lock(sample_mutex_);
          latest_sample_.management_operation = pi_arm_can::to_string(operation);
        }
        const auto result = backend_->manage(operation, request->motor_id);
        {
          std::lock_guard<std::mutex> sample_lock(sample_mutex_);
          latest_sample_.management_operation = "NONE";
          if (!result.success) {
            latest_sample_.error_message = result.message;
          }
        }
        response->success = result.success;
        response->code = result.code;
        response->message = result.message;
      }));
}

void PiArmSystem::publish_hardware_state()
{
  HardwareSample sample;
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample = latest_sample_;
  }
  if (!hardware_active_.load()) {
    // Belt-and-suspenders: never advertise frozen motion while inactive.
    std::fill(sample.velocities_rad_s.begin(), sample.velocities_rad_s.end(), 0.0);
    std::fill(sample.in_motion.begin(), sample.in_motion.end(), false);
    std::fill(sample.fresh.begin(), sample.fresh.end(), false);
    std::fill(sample.motor_in_motion.begin(), sample.motor_in_motion.end(), false);
    std::fill(sample.motor_fresh.begin(), sample.motor_fresh.end(), false);
    sample.connected = false;
  }
  auto message = make_hardware_state_message(
    sample, joint_names_, backend_config_, node_->now());
  state_publisher_->publish(std::move(message));
}

}  // namespace pi_arm_hardware

PLUGINLIB_EXPORT_CLASS(pi_arm_hardware::PiArmSystem, hardware_interface::SystemInterface)
