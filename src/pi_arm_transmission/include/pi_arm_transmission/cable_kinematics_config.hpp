#pragma once

#include <array>

namespace pi_arm_transmission
{

struct LinearAxisParameters
{
  double motor_radius_mm{0.0};
  double joint_radius_mm{0.0};
};

struct CoupledAxis3Parameters
{
  double motor_radius_mm{0.0};
  double joint3_radius_mm{0.0};
  double joint4_radius_mm{0.0};
  double joint2_guide_radius_mm{0.0};
  int coupling_sign{1};
};

struct CoupledChordAxisParameters
{
  double motor_radius_mm{0.0};
  double joint_radius_mm{0.0};
  double joint2_guide_radius_mm{0.0};
  int coupling_sign{1};
};

struct Axis6BranchParameters
{
  double motor_radius_mm{0.0};
  double joint_radius_mm{0.0};
  double joint2_guide_radius_mm{0.0};
  int coupling_sign{1};
};

struct OutputPositionLimits
{
  double lower_position_rad{0.0};
  double upper_position_rad{0.0};
};

struct CableKinematicsParameters
{
  LinearAxisParameters axis1;
  LinearAxisParameters axis2;
  CoupledAxis3Parameters axis3;
  CoupledChordAxisParameters axis4;
  CoupledChordAxisParameters axis5;
  Axis6BranchParameters axis6_branch_a;
  Axis6BranchParameters axis6_branch_b;
  std::array<OutputPositionLimits, 6> output_limits;
  double asin_domain_tolerance{1.0e-12};
  double cosine_singularity_threshold{1.0e-6};
  double compatibility_tolerance{1.0e-9};
  double output_limit_tolerance_rad{1.0e-6};
};

}  // namespace pi_arm_transmission
