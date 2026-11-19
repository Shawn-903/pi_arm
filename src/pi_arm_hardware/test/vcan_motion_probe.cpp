// Feedback-only production backend probe. Never selects physical CAN.
#include "vcan_probe_config.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  auto config = vcan_probe_config(argv[1], "vcan_motion");
  pi_arm_hardware::RealBackend backend(config);
  backend.configure();
  backend.activate();
  std::cout << "READY" << std::endl;
  std::string command;
  while (std::getline(std::cin, command) && command != "quit") {
    unsigned motor_mask = 0, joint_mask = 0;
    int fresh_frames = 0, kinematics_frames = 0, raw_speed_frames = 0;
    for (int frame = 0; frame < 120; ++frame) {
      pi_arm_hardware::HardwareSample sample;
      if (!backend.read(sample)) {return 3;}
      if (frame >= 60) {  // Allow transition and the position window to settle.
        for (int i = 0; i < 6; ++i) {
          if (sample.motor_in_motion[i]) {motor_mask |= 1U << i;}
          if (sample.in_motion[i]) {joint_mask |= 1U << i;}
        }
        fresh_frames += std::all_of(sample.motor_fresh.begin(), sample.motor_fresh.end(),
          [](bool value) {return value;});
        kinematics_frames += sample.kinematics_valid;
        raw_speed_frames += std::any_of(sample.velocities_rad_s.begin(), sample.velocities_rad_s.end(),
          [](double value) {return std::abs(value) > 0.00017453292519943296;});
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::cout << "{\"motor_mask\":" << motor_mask << ",\"joint_mask\":" << joint_mask <<
      ",\"fresh_frames\":" << fresh_frames << ",\"kinematics_frames\":" << kinematics_frames <<
      ",\"raw_speed_frames\":" << raw_speed_frames << "}" << std::endl;
  }
}
