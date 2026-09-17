// Production backend recovery probe. Only virtual CAN is selectable.
#include "pi_arm_hardware/backend.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
int main(int argc, char **argv)
{
  if (argc != 3) return 2;
  pi_arm_hardware::BackendConfig c;
  c.can_interface = "vcan0";
  c.cable_kinematics_file = std::string(argv[1]) + "/src/pi_arm_description/config/cable_kinematics.yaml";
  c.calibration_file = argv[2]; c.allow_real_control = true;
  const std::vector<std::string> names{"joint_01","joint_02","joint_04","joint_05","joint_06","joint_09"};
  const std::vector<int> signs{1,1,-1,-1,-1,1};
  for (int i=0;i<6;++i) c.actuators.push_back({names[i],static_cast<uint8_t>(i+1),i<3?36.0:6.0,signs[i]});
  c.joint_upper_limits_rad={3.141592654,1.757546556759,1.494001839708,.523598776,1.047197551,3.141592654};
  for (double v:c.joint_upper_limits_rad) c.joint_lower_limits_rad.push_back(-v);
  c.joint_velocity_limits_rad_s.assign(6,.1745329);
  c.joint_feedback_tolerances_rad={0,.05235987756,.05235987756,0,0,0};
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
