// Exercise the production plugin's public lifecycle while keeping it alive
// afterwards, so resource release cannot be hidden by its destructor.
#include "pi_arm_hardware/pi_arm_system.hpp"
#include "hardware_interface/component_parser.hpp"

#include <sys/socket.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

int can_sockets()
{
  int count = 0;
  for (const auto & entry : std::filesystem::directory_iterator("/proc/self/fd")) {
    const int fd = std::stoi(entry.path().filename());
    int domain = 0;
    socklen_t length = sizeof(domain);
    if (::getsockopt(fd, SOL_SOCKET, SO_DOMAIN, &domain, &length) == 0 && domain == AF_CAN) {
      ++count;
    }
  }
  return count;
}

void require_success(hardware_interface::CallbackReturn result)
{
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    throw std::runtime_error("lifecycle callback failed");
  }
}

void run(const char * urdf_path)
{
  std::ifstream input(urdf_path);
  std::stringstream xml;
  xml << input.rdbuf();
  const auto info = hardware_interface::parse_control_resources_from_urdf(xml.str()).at(0);
  if (info.hardware_parameters.at("backend") != "real" ||
    info.hardware_parameters.at("can_interface") != "vcan0")
  {
    throw std::runtime_error("probe requires real backend on vcan0 only");
  }
  pi_arm_hardware::PiArmSystem system;
  const rclcpp_lifecycle::State state;
  require_success(system.on_init(info));
  require_success(system.on_configure(state));
  require_success(system.on_activate(state));
  for (int i = 0; i < 8; ++i) {
    std::this_thread::sleep_for(100ms);
    system.read(rclcpp::Time(0), rclcpp::Duration(0, 20000000));
  }
  system.write(rclcpp::Time(0), rclcpp::Duration(0, 20000000));
  auto node = std::make_shared<rclcpp::Node>("lifecycle_probe_client");
  using Arm = pi_arm_interfaces::srv::SetMotionArmed;
  auto client = node->create_client<Arm>("/pi_arm/hardware/set_motion_armed");
  if (!client->wait_for_service(3s)) {throw std::runtime_error("motion service missing");}
  const auto arm = [&](bool armed, bool hold) {
      auto request = std::make_shared<Arm::Request>();
      request->armed = armed; request->hold_position = hold;
      request->lease_id = armed ? 1U : 0U;
      auto future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, future, 3s) != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("motion service timed out");
      }
      return future.get()->success;
    };
  if (!arm(true, false)) {throw std::runtime_error("initial motion lease rejected");}
  auto commands = system.export_command_interfaces();
  for (auto & command : commands) {
    if (command.get_name() == "joint_01/position" && !command.set_value(.01)) {
      throw std::runtime_error("position command assignment failed");
    }
  }
  system.write(rclcpp::Time(0), rclcpp::Duration(0, 20000000));
  std::this_thread::sleep_for(100ms);
  std::cout << "READY" << std::endl;
  std::string operation;
  while (std::getline(std::cin, operation) && operation != "quit") {
    bool success = true;
    if (operation == "shutdown") {require_success(system.on_shutdown(state));}
    else if (operation == "error") {require_success(system.on_error(state));}
    else if (operation == "cleanup") {require_success(system.on_cleanup(state));}
    else if (operation == "deactivate") {require_success(system.on_deactivate(state));}
    else if (operation == "activate") {require_success(system.on_activate(state));}
    else if (operation == "arm") {success = arm(true, false);}
    else if (operation == "hold") {success = arm(false, true);}
    else {throw std::runtime_error("unknown lifecycle operation");}
    std::cout << "ok=" << success << " can_sockets=" << can_sockets() << std::endl;
  }
}

int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  rclcpp::init(argc, argv);
  int result = 0;
  try {run(argv[1]);}
  catch (const std::exception & error) {std::cerr << error.what() << std::endl; result = 1;}
  rclcpp::shutdown();
  return result;
}
