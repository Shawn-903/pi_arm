#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "pi_arm_manager/direct_move_validation.hpp"

using pi_arm_manager::JointLimits;
using pi_arm_manager::validate_direct_positions;

TEST(DirectMoveValidation, ValidatesUnselectedHoldJointAtFullPrecision)
{
  const std::map<std::string, JointLimits> limits{
    {"joint_01", {-1.0, 1.0}}, {"joint_02", {-1.75754655675829, 1.75754655675829}}};
  const double boundary = limits.at("joint_02").upper;
  EXPECT_NO_THROW(validate_direct_positions(
    limits, {"joint_01", "joint_02"}, {0.0, boundary}, "start/hold"));
  const double outside = std::nextafter(boundary, std::numeric_limits<double>::infinity());
  try {
    validate_direct_positions(limits, {"joint_01", "joint_02"}, {0.0, outside}, "start/hold");
    FAIL() << "Out-of-limit hold position must fail before arming";
  } catch (const std::runtime_error & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("start/hold"), std::string::npos);
    EXPECT_NE(message.find("joint_02"), std::string::npos);
    EXPECT_NE(message.find("1.757546556758"), std::string::npos);
  }
}

TEST(DirectMoveValidation, RejectsIncompleteDuplicateUnknownOrNonfiniteFeedback)
{
  const std::map<std::string, JointLimits> limits{
    {"joint_01", {-1.0, 1.0}}, {"joint_02", {-1.0, 1.0}}};
  EXPECT_THROW(validate_direct_positions(
    limits, {"joint_01"}, {0.0}, "start/hold"), std::runtime_error);
  EXPECT_THROW(validate_direct_positions(
    limits, {"joint_01", "joint_01"}, {0.0, 0.0}, "start/hold"), std::runtime_error);
  EXPECT_THROW(validate_direct_positions(
    limits, {"joint_01", "unknown"}, {0.0, 0.0}, "start/hold"), std::runtime_error);
  EXPECT_THROW(validate_direct_positions(
    limits, {"joint_01", "joint_02"}, {NAN, 0.0}, "start/hold"), std::runtime_error);
  EXPECT_THROW(validate_direct_positions(
    limits, {"joint_01", "joint_02"}, {0.0, 1.1}, "target/hold"), std::runtime_error);
}
