#pragma once

#include "pi_arm_transmission/cable_kinematics_config.hpp"

#include <string>

namespace pi_arm_transmission
{

CableKinematicsParameters load_cable_kinematics_config(const std::string & path);

}  // namespace pi_arm_transmission
