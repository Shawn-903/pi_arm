// Production backend recovery probe. Only virtual CAN is selectable.
#include "vcan_probe_config.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
int main(int argc, char **argv)
{
  if (argc != 2) {return 2;}
  auto c = vcan_probe_config(argv[1], "vcan0");
  c.read_only = false;
  c.allow_real_control = true;
  pi_arm_hardware::RealBackend backend(c);
  backend.configure(); backend.activate();
  std::this_thread::sleep_for(400ms);
  pi_arm_hardware::HardwareSample before,after;
  backend.read(before);
  const auto result=backend.manage(pi_arm_can::ManagementOperation::MOTOR_REBOOT,2);
  std::this_thread::sleep_for(150ms); backend.read(after);
  const auto arm=backend.set_motion_armed(true,1);
  const bool enable_rejected = !result.success &&
    !backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 2).success;
  std::cout << "success=" << result.success << " enabled=" << after.motor_enabled.at(1)
            << " error=" << after.kinematics_error_code << " armed=" << arm.success
            << " enable_rejected=" << enable_rejected
            << " message=" << result.message << std::endl;
  backend.cleanup();
}
