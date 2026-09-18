#pragma once

#include "pi_arm_transmission/cable_kinematics_config.hpp"
#include "pi_arm_transmission/result.hpp"

#include <array>
#include <cstddef>

namespace pi_arm_transmission
{

using Vector6 = std::array<double, 6>;
using Matrix6 = std::array<std::array<double, 6>, 6>;

// Public vectors use the six ROS active-joint coordinates in this order:
// joint_01, joint_02, joint_04, joint_05, joint_06, joint_09.
// For the synchronized mechanisms, the equivalent mechanical coordinates are
// S3 = 2 * joint_04, S4 = 2 * joint_05, and S5 = 2 * joint_06. Those factors
// are already substituted into the formulas implemented by this class.

struct Axis6Residual
{
  double branch_a_mm{0.0};
  double branch_b_mm{0.0};
};

class CableKinematics
{
public:
  explicit CableKinematics(CableKinematicsParameters parameters);

  const CableKinematicsParameters & parameters() const noexcept {return parameters_;}

  KinematicsStatus joint_to_output_position(
    const Vector6 & joint_position_rad,
    Vector6 & output_position_rad) const noexcept;

  KinematicsStatus output_to_joint_position(
    const Vector6 & output_position_rad,
    Vector6 & joint_position_rad) const noexcept;

  KinematicsStatus check_output_position_limits(
    const Vector6 & output_position_rad) const noexcept;

  // Feedback allowance propagated through the cable coupling. Command callers
  // must use the strict check above. Joint bounds must still be checked after IK.
  KinematicsStatus check_output_feedback_limits(
    const Vector6 & output_position_rad,
    const Vector6 & joint_tolerances_rad) const noexcept;

  KinematicsStatus joint_to_output_velocity(
    const Vector6 & joint_position_rad,
    const Vector6 & joint_velocity_rad_s,
    Vector6 & output_velocity_rad_s) const noexcept;

  KinematicsStatus output_to_joint_velocity(
    const Vector6 & joint_position_rad,
    const Vector6 & output_velocity_rad_s,
    Vector6 & joint_velocity_rad_s) const noexcept;

  KinematicsStatus jacobian(
    const Vector6 & joint_position_rad,
    Matrix6 & jacobian) const noexcept;

  Axis6Residual axis6_residual(
    double joint2_position_rad,
    double joint6_position_rad,
    double output6_position_rad) const noexcept;

private:
  CableKinematicsParameters parameters_;
  std::size_t motor4_joint_{3U};
  std::size_t motor5_joint_{4U};
  double alpha1_{0.0};
  double alpha2_{0.0};
  double axis3_self_gain_{0.0};
  double axis4_self_gain_{0.0};
  double axis5_self_gain_{0.0};
  double axis3_coupling_gain_{0.0};
  double axis4_coupling_gain_{0.0};
  double axis5_coupling_gain_{0.0};
  double alpha6_{0.0};
  double axis6_coupling_gain_{0.0};
};

}  // namespace pi_arm_transmission
