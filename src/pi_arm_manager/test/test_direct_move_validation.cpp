#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "pi_arm_manager/direct_move_validation.hpp"

using pi_arm_manager::JointLimits;
using pi_arm_manager::validate_direct_positions;

TEST(DirectMoveValidation, FeedbackAllowanceNeverExpandsTargets)
{
  const std::map<std::string, JointLimits> limits{
    {"joint_02", {-1.0, 1.0, 0.2, 0.1, 0.05}}};
  EXPECT_NO_THROW(validate_direct_positions(limits, {"joint_02"}, {1.04}, "start", true));
  EXPECT_THROW(validate_direct_positions(limits, {"joint_02"}, {1.04}, "target"), std::runtime_error);
  EXPECT_THROW(validate_direct_positions(limits, {"joint_02"}, {-1.06}, "start", true), std::runtime_error);
}

TEST(DirectMoveValidation, SlowPackingMoveOutlivesDefaultTimeout)
{
  using pi_arm_manager::direct_result_timeout;
  // 100.7 degrees at 0.2 degrees/s needs 503.5 seconds before settling.
  EXPECT_DOUBLE_EQ(direct_result_timeout(100.7 / 0.2, 120.0, 5.0), 508.5);
  EXPECT_DOUBLE_EQ(direct_result_timeout(2.0, 120.0, 5.0), 120.0);
  EXPECT_THROW(direct_result_timeout(NAN, 120.0, 5.0), std::invalid_argument);
  EXPECT_THROW(direct_result_timeout(-1.0, 120.0, 5.0), std::invalid_argument);
}

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
