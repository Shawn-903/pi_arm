#pragma once

#include "pi_arm_hardware/backend.hpp"

#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

inline pi_arm_hardware::BackendConfig vcan_probe_config(
  const char * path, const std::string & expected_interface)
{
  std::ifstream file(path);
  if (!file) {throw std::runtime_error("cannot open probe URDF");}
  const std::string xml{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
  auto config = pi_arm_hardware::backend_config_from_robot_description(xml);
  if (config.can_interface != expected_interface) {
    throw std::runtime_error("probe requires " + expected_interface);
  }
  return config;
}
