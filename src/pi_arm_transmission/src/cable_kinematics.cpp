#include "pi_arm_transmission/cable_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace pi_arm_transmission
{
namespace
{

void require_positive_finite(const double value, const char * const name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
}

void require_direction(const int value, const char * const name)
{
  if (value != -1 && value != 1) {
    throw std::invalid_argument(std::string(name) + " must be -1 or 1");
  }
}

bool approximately_equal(
  const double lhs, const double rhs, const double relative_tolerance) noexcept
{
  const double scale = std::max({1.0, std::abs(lhs), std::abs(rhs)});
  return std::abs(lhs - rhs) <= relative_tolerance * scale;
}

KinematicsStatus require_finite_vector(const Vector6 & values) noexcept
{
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (!std::isfinite(values[index])) {
      return {KinematicsError::NON_FINITE_INPUT, index + 1U, values[index]};
    }
  }
  return KinematicsStatus::success();
}

KinematicsStatus asin_argument(
  const double raw, const double tolerance, const std::size_t axis,
  double & clamped) noexcept
{
  if (!std::isfinite(raw)) {
    return {KinematicsError::NON_FINITE_INPUT, axis, raw};
  }
  if (raw < -1.0 - tolerance || raw > 1.0 + tolerance) {
    return {KinematicsError::ASIN_DOMAIN, axis, raw};
  }
  clamped = std::clamp(raw, -1.0, 1.0);
  return KinematicsStatus::success();
}

}  // namespace

const char * to_string(const KinematicsError error) noexcept
{
  switch (error) {
    case KinematicsError::NONE:
      return "NONE";
    case KinematicsError::NON_FINITE_INPUT:
      return "NON_FINITE_INPUT";
    case KinematicsError::OUTPUT_POSITION_LIMIT:
      return "OUTPUT_POSITION_LIMIT";
    case KinematicsError::ASIN_DOMAIN:
      return "ASIN_DOMAIN";
    case KinematicsError::VELOCITY_SINGULARITY:
      return "VELOCITY_SINGULARITY";
  }
  return "UNKNOWN";
}

CableKinematics::CableKinematics(CableKinematicsParameters parameters)
: parameters_(parameters)
{
  for (const int direction : parameters_.output_directions) {
    require_direction(direction, "output_directions");
  }
  require_positive_finite(parameters_.axis1.motor_radius_mm, "axis1.motor_radius_mm");
  require_positive_finite(parameters_.axis1.joint_radius_mm, "axis1.joint_radius_mm");
  require_positive_finite(parameters_.axis2.motor_radius_mm, "axis2.motor_radius_mm");
  require_positive_finite(parameters_.axis2.joint_radius_mm, "axis2.joint_radius_mm");

  require_positive_finite(parameters_.axis3.motor_radius_mm, "axis3.motor_radius_mm");
  require_positive_finite(parameters_.axis3.joint3_radius_mm, "axis3.joint3_radius_mm");
  require_positive_finite(parameters_.axis3.joint4_radius_mm, "axis3.joint4_radius_mm");
  require_positive_finite(
    parameters_.axis3.joint2_guide_radius_mm, "axis3.joint2_guide_radius_mm");
  require_direction(parameters_.axis3.coupling_sign, "axis3.coupling_sign");

  require_positive_finite(parameters_.axis4.motor_radius_mm, "axis4.motor_radius_mm");
  require_positive_finite(parameters_.axis4.joint_radius_mm, "axis4.joint_radius_mm");
  require_positive_finite(
    parameters_.axis4.joint2_guide_radius_mm, "axis4.joint2_guide_radius_mm");
  require_direction(parameters_.axis4.coupling_sign, "axis4.coupling_sign");

  require_positive_finite(parameters_.axis5.motor_radius_mm, "axis5.motor_radius_mm");
  require_positive_finite(parameters_.axis5.joint_radius_mm, "axis5.joint_radius_mm");
  require_positive_finite(
    parameters_.axis5.joint2_guide_radius_mm, "axis5.joint2_guide_radius_mm");
  require_direction(parameters_.axis5.coupling_sign, "axis5.coupling_sign");

  require_positive_finite(
    parameters_.axis6_branch_a.motor_radius_mm, "axis6.branch_a.motor_radius_mm");
  require_positive_finite(
    parameters_.axis6_branch_a.joint_radius_mm, "axis6.branch_a.joint_radius_mm");
  require_positive_finite(
    parameters_.axis6_branch_a.joint2_guide_radius_mm,
    "axis6.branch_a.joint2_guide_radius_mm");
  require_direction(
    parameters_.axis6_branch_a.coupling_sign, "axis6.branch_a.coupling_sign");

  require_positive_finite(
    parameters_.axis6_branch_b.motor_radius_mm, "axis6.branch_b.motor_radius_mm");
  require_positive_finite(
    parameters_.axis6_branch_b.joint_radius_mm, "axis6.branch_b.joint_radius_mm");
  require_positive_finite(
    parameters_.axis6_branch_b.joint2_guide_radius_mm,
    "axis6.branch_b.joint2_guide_radius_mm");
  require_direction(
    parameters_.axis6_branch_b.coupling_sign, "axis6.branch_b.coupling_sign");

  require_positive_finite(parameters_.asin_domain_tolerance, "asin_domain_tolerance");
  require_positive_finite(
    parameters_.cosine_singularity_threshold, "cosine_singularity_threshold");
  require_positive_finite(
    parameters_.compatibility_tolerance, "compatibility_tolerance");
  require_positive_finite(
    parameters_.output_limit_tolerance_rad, "output_limit_tolerance_rad");

  for (std::size_t index = 0; index < parameters_.output_limits.size(); ++index) {
    const auto & limit = parameters_.output_limits[index];
    if (!std::isfinite(limit.lower_position_rad) ||
      !std::isfinite(limit.upper_position_rad) ||
      limit.lower_position_rad >= limit.upper_position_rad)
    {
      throw std::invalid_argument(
              "output_limits[" + std::to_string(index) + "] must be finite and ordered");
    }
  }

  alpha1_ = parameters_.axis1.joint_radius_mm / parameters_.axis1.motor_radius_mm;
  alpha2_ = parameters_.axis2.joint_radius_mm / parameters_.axis2.motor_radius_mm;
  axis3_self_gain_ =
    (parameters_.axis3.joint3_radius_mm + parameters_.axis3.joint4_radius_mm) /
    parameters_.axis3.motor_radius_mm;
  axis4_self_gain_ =
    parameters_.axis4.joint_radius_mm / parameters_.axis4.motor_radius_mm;
  axis5_self_gain_ =
    parameters_.axis5.joint_radius_mm / parameters_.axis5.motor_radius_mm;
  axis3_coupling_gain_ =
    static_cast<double>(parameters_.axis3.coupling_sign) *
    parameters_.axis3.joint2_guide_radius_mm / parameters_.axis3.motor_radius_mm;
  axis4_coupling_gain_ =
    static_cast<double>(parameters_.axis4.coupling_sign) *
    parameters_.axis4.joint2_guide_radius_mm / parameters_.axis4.motor_radius_mm;
  axis5_coupling_gain_ =
    static_cast<double>(parameters_.axis5.coupling_sign) *
    parameters_.axis5.joint2_guide_radius_mm / parameters_.axis5.motor_radius_mm;

  const double alpha6a =
    parameters_.axis6_branch_a.joint_radius_mm /
    parameters_.axis6_branch_a.motor_radius_mm;
  const double alpha6b =
    parameters_.axis6_branch_b.joint_radius_mm /
    parameters_.axis6_branch_b.motor_radius_mm;
  const double kappa6a =
    static_cast<double>(parameters_.axis6_branch_a.coupling_sign) *
    parameters_.axis6_branch_a.joint2_guide_radius_mm /
    parameters_.axis6_branch_a.motor_radius_mm;
  const double kappa6b =
    static_cast<double>(parameters_.axis6_branch_b.coupling_sign) *
    parameters_.axis6_branch_b.joint2_guide_radius_mm /
    parameters_.axis6_branch_b.motor_radius_mm;

  if (!approximately_equal(alpha6a, alpha6b, parameters_.compatibility_tolerance)) {
    throw std::invalid_argument(
            "axis6 branches have incompatible joint-to-motor radius ratios");
  }
  if (!approximately_equal(kappa6a, kappa6b, parameters_.compatibility_tolerance)) {
    throw std::invalid_argument(
            "axis6 branches have incompatible joint2 coupling ratios");
  }
  alpha6_ = 0.5 * (alpha6a + alpha6b);
  axis6_coupling_gain_ = 0.5 * (kappa6a + kappa6b);

  // Apply to the whole row, including J2 compensation. Signed gains are shared
  // by forward/inverse position, velocity and Jacobian calculations.
  const auto & direction = parameters_.output_directions;
  alpha1_ *= direction[0];
  alpha2_ *= direction[1];
  axis3_self_gain_ *= direction[2];
  axis3_coupling_gain_ *= direction[2];
  axis4_self_gain_ *= direction[3];
  axis4_coupling_gain_ *= direction[3];
  axis5_self_gain_ *= direction[4];
  axis5_coupling_gain_ *= direction[4];
  alpha6_ *= direction[5];
  axis6_coupling_gain_ *= direction[5];
}

KinematicsStatus CableKinematics::joint_to_output_position(
  const Vector6 & joint_position_rad, Vector6 & output_position_rad) const noexcept
{
  const auto finite = require_finite_vector(joint_position_rad);
  if (!finite) {
    return finite;
  }

  output_position_rad[0] = alpha1_ * joint_position_rad[0];
  output_position_rad[1] = alpha2_ * joint_position_rad[1];
  output_position_rad[2] =
    axis3_self_gain_ * joint_position_rad[2] +
    axis3_coupling_gain_ * joint_position_rad[1];
  output_position_rad[3] =
    axis4_self_gain_ * std::sin(joint_position_rad[3]) +
    axis4_coupling_gain_ * joint_position_rad[1];
  output_position_rad[4] =
    axis5_self_gain_ * std::sin(joint_position_rad[4]) +
    axis5_coupling_gain_ * joint_position_rad[1];
  output_position_rad[5] =
    alpha6_ * joint_position_rad[5] + axis6_coupling_gain_ * joint_position_rad[1];
  return require_finite_vector(output_position_rad);
}

KinematicsStatus CableKinematics::output_to_joint_position(
  const Vector6 & output_position_rad, Vector6 & joint_position_rad) const noexcept
{
  const auto finite = require_finite_vector(output_position_rad);
  if (!finite) {
    return finite;
  }

  joint_position_rad[0] = output_position_rad[0] / alpha1_;
  joint_position_rad[1] = output_position_rad[1] / alpha2_;
  joint_position_rad[2] =
    (output_position_rad[2] - axis3_coupling_gain_ * joint_position_rad[1]) /
    axis3_self_gain_;

  double argument4 = 0.0;
  auto status = asin_argument(
    (output_position_rad[3] - axis4_coupling_gain_ * joint_position_rad[1]) /
    axis4_self_gain_,
    parameters_.asin_domain_tolerance, 4U, argument4);
  if (!status) {
    return status;
  }
  joint_position_rad[3] = std::asin(argument4);

  double argument5 = 0.0;
  status = asin_argument(
    (output_position_rad[4] - axis5_coupling_gain_ * joint_position_rad[1]) /
    axis5_self_gain_,
    parameters_.asin_domain_tolerance, 5U, argument5);
  if (!status) {
    return status;
  }
  joint_position_rad[4] = std::asin(argument5);
  joint_position_rad[5] =
    (output_position_rad[5] - axis6_coupling_gain_ * joint_position_rad[1]) / alpha6_;
  return require_finite_vector(joint_position_rad);
}

KinematicsStatus CableKinematics::check_output_position_limits(
  const Vector6 & output_position_rad) const noexcept
{
  const auto finite = require_finite_vector(output_position_rad);
  if (!finite) {
    return finite;
  }

  for (std::size_t index = 0; index < output_position_rad.size(); ++index) {
    const auto & limit = parameters_.output_limits[index];
    const double value = output_position_rad[index];
    if (value < limit.lower_position_rad - parameters_.output_limit_tolerance_rad ||
      value > limit.upper_position_rad + parameters_.output_limit_tolerance_rad)
    {
      return {KinematicsError::OUTPUT_POSITION_LIMIT, index + 1U, value};
    }
  }
  return KinematicsStatus::success();
}

KinematicsStatus CableKinematics::check_output_feedback_limits(
  const Vector6 & output_position_rad,
  const Vector6 & joint_tolerances_rad) const noexcept
{
  const auto finite = require_finite_vector(output_position_rad);
  if (!finite) {return finite;}
  for (std::size_t i = 0; i < joint_tolerances_rad.size(); ++i) {
    if (!std::isfinite(joint_tolerances_rad[i]) || joint_tolerances_rad[i] < 0.0) {
      return {KinematicsError::NON_FINITE_INPUT, i + 1U, joint_tolerances_rad[i]};
    }
  }
  // All linear gains are constant and |d(sin q)/dq| <= 1. The absolute
  // Jacobian at zero therefore bounds each output's change over the entire
  // joint allowance, including coupled J2 motion and either coupling sign.
  Matrix6 gain{};
  jacobian(Vector6{}, gain);
  for (std::size_t i = 0; i < output_position_rad.size(); ++i) {
    double margin = parameters_.output_limit_tolerance_rad;
    for (std::size_t j = 0; j < joint_tolerances_rad.size(); ++j) {
      margin += std::abs(gain[i][j]) * joint_tolerances_rad[j];
    }
    const auto & limit = parameters_.output_limits[i];
    if (output_position_rad[i] < limit.lower_position_rad - margin ||
      output_position_rad[i] > limit.upper_position_rad + margin)
    {
      return {KinematicsError::OUTPUT_POSITION_LIMIT, i + 1U, output_position_rad[i]};
    }
  }
  return KinematicsStatus::success();
}

KinematicsStatus CableKinematics::jacobian(
  const Vector6 & joint_position_rad, Matrix6 & jacobian_value) const noexcept
{
  const auto finite = require_finite_vector(joint_position_rad);
  if (!finite) {
    return finite;
  }

  jacobian_value = {};
  jacobian_value[0][0] = alpha1_;
  jacobian_value[1][1] = alpha2_;
  jacobian_value[2][1] = axis3_coupling_gain_;
  jacobian_value[2][2] = axis3_self_gain_;
  jacobian_value[3][1] = axis4_coupling_gain_;
  jacobian_value[3][3] =
    axis4_self_gain_ * std::cos(joint_position_rad[3]);
  jacobian_value[4][1] = axis5_coupling_gain_;
  jacobian_value[4][4] =
    axis5_self_gain_ * std::cos(joint_position_rad[4]);
  jacobian_value[5][1] = axis6_coupling_gain_;
  jacobian_value[5][5] = alpha6_;
  return KinematicsStatus::success();
}

KinematicsStatus CableKinematics::joint_to_output_velocity(
  const Vector6 & joint_position_rad, const Vector6 & joint_velocity_rad_s,
  Vector6 & output_velocity_rad_s) const noexcept
{
  auto status = require_finite_vector(joint_velocity_rad_s);
  if (!status) {
    return status;
  }
  Matrix6 jacobian_value{};
  status = jacobian(joint_position_rad, jacobian_value);
  if (!status) {
    return status;
  }

  output_velocity_rad_s = {};
  for (std::size_t row = 0; row < output_velocity_rad_s.size(); ++row) {
    for (std::size_t column = 0; column < joint_velocity_rad_s.size(); ++column) {
      output_velocity_rad_s[row] +=
        jacobian_value[row][column] * joint_velocity_rad_s[column];
    }
  }
  return require_finite_vector(output_velocity_rad_s);
}

KinematicsStatus CableKinematics::output_to_joint_velocity(
  const Vector6 & joint_position_rad, const Vector6 & output_velocity_rad_s,
  Vector6 & joint_velocity_rad_s) const noexcept
{
  auto status = require_finite_vector(joint_position_rad);
  if (!status) {
    return status;
  }
  status = require_finite_vector(output_velocity_rad_s);
  if (!status) {
    return status;
  }

  const double cosine4 = std::cos(joint_position_rad[3]);
  const double cosine5 = std::cos(joint_position_rad[4]);
  if (std::abs(cosine4) <= parameters_.cosine_singularity_threshold) {
    return {KinematicsError::VELOCITY_SINGULARITY, 4U, cosine4};
  }
  if (std::abs(cosine5) <= parameters_.cosine_singularity_threshold) {
    return {KinematicsError::VELOCITY_SINGULARITY, 5U, cosine5};
  }

  joint_velocity_rad_s[0] = output_velocity_rad_s[0] / alpha1_;
  joint_velocity_rad_s[1] = output_velocity_rad_s[1] / alpha2_;
  joint_velocity_rad_s[2] =
    (output_velocity_rad_s[2] - axis3_coupling_gain_ * joint_velocity_rad_s[1]) /
    axis3_self_gain_;
  joint_velocity_rad_s[3] =
    (output_velocity_rad_s[3] - axis4_coupling_gain_ * joint_velocity_rad_s[1]) /
    (axis4_self_gain_ * cosine4);
  joint_velocity_rad_s[4] =
    (output_velocity_rad_s[4] - axis5_coupling_gain_ * joint_velocity_rad_s[1]) /
    (axis5_self_gain_ * cosine5);
  joint_velocity_rad_s[5] =
    (output_velocity_rad_s[5] - axis6_coupling_gain_ * joint_velocity_rad_s[1]) /
    alpha6_;
  return require_finite_vector(joint_velocity_rad_s);
}

Axis6Residual CableKinematics::axis6_residual(
  const double joint2_position_rad, const double joint6_position_rad,
  const double output6_position_rad) const noexcept
{
  const auto & branch_a = parameters_.axis6_branch_a;
  const auto & branch_b = parameters_.axis6_branch_b;
  return {
    branch_a.motor_radius_mm * parameters_.output_directions[5] * output6_position_rad -
    branch_a.joint_radius_mm * joint6_position_rad -
    static_cast<double>(branch_a.coupling_sign) *
    branch_a.joint2_guide_radius_mm * joint2_position_rad,
    branch_b.motor_radius_mm * parameters_.output_directions[5] * output6_position_rad -
    branch_b.joint_radius_mm * joint6_position_rad -
    static_cast<double>(branch_b.coupling_sign) *
    branch_b.joint2_guide_radius_mm * joint2_position_rad};
}

}  // namespace pi_arm_transmission
