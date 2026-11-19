#include "pi_arm_transmission/config_loader.hpp"

#include <exception>
#include <iomanip>
#include <iostream>

int main(int argc, char ** argv)
{
  if (argc != 2) {std::cerr << "usage: pi_arm_output_limits CABLE_YAML\n"; return 2;}
  try {
    const auto model = pi_arm_transmission::load_cable_kinematics_config(argv[1]);
    std::cout << std::setprecision(17);
    for (const auto & limit : model.output_limits) {
      std::cout << limit.lower_position_rad << ' ' << limit.upper_position_rad << '\n';
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
