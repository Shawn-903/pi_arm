#include "pi_arm_transmission/config_loader.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace
{

std::filesystem::path write_config(const std::string & contents, const std::string & name)
{
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream stream(path);
  stream << contents;
  stream.close();
  return path;
}

const char * valid_config()
{
  return R"(
cable_kinematics:
  model_version: 3
  angle_unit: rad
  length_unit: mm
  validation:
    asin_domain_tolerance: 1.0e-12
    cosine_singularity_threshold: 1.0e-6
    compatibility_tolerance: 1.0e-3
    output_limit_tolerance_rad: 1.0e-6
  output_limits:
    coordinate: reducer_output_shaft
    unit: rad
    motor_01: {lower_position_rad: -3.819, upper_position_rad: 3.819}
    motor_02: {lower_position_rad: -1.746, upper_position_rad: 1.746}
    motor_03: {lower_position_rad: -7.786, upper_position_rad: 7.786}
    motor_04: {lower_position_rad: -4.106, upper_position_rad: 4.106}
    motor_05: {lower_position_rad: -4.870, upper_position_rad: 4.870}
    motor_06: {lower_position_rad: -6.027, upper_position_rad: 6.027}
  axes:
    joint_01:
      model: linear
      motor_radius_mm: 16.25
      joint_radius_mm: 19.75
    joint_02:
      model: linear
      motor_radius_mm: 16.0
      joint_radius_mm: 16.0
    joint_04:
      model: synchronized_pair_coupled
      motor_radius_mm: 16.25
      joint3_radius_mm: 28.5
      joint4_radius_mm: 23.25
      joint2_guide_radius_mm: 28.5
      coupling_sign: 1
    joint_05:
      model: chord_coupled
      motor_radius_mm: 16.25
      joint_radius_mm: 33.94
      joint2_guide_radius_mm: 28.5
      coupling_sign: 1
    joint_06:
      model: chord_coupled
      motor_radius_mm: 16.25
      joint_radius_mm: 33.94
      joint2_guide_radius_mm: 28.5
      coupling_sign: 1
    joint_09:
      model: dual_branch_coupled
      branch_a:
        motor_radius_mm: 12.25
        joint_radius_mm: 12.25
        joint2_guide_radius_mm: 20.25
        coupling_sign: 1
      branch_b:
        motor_radius_mm: 17.25
        joint_radius_mm: 17.25
        joint2_guide_radius_mm: 28.5
        coupling_sign: 1
)";
}

TEST(ConfigLoader, LoadsStrictValidConfiguration)
{
  const auto path = write_config(valid_config(), "pi_arm_valid_cable_kinematics.yaml");
  const auto parameters =
    pi_arm_transmission::load_cable_kinematics_config(path.string());
  EXPECT_DOUBLE_EQ(parameters.axis3.joint3_radius_mm, 28.5);
  EXPECT_DOUBLE_EQ(parameters.axis3.joint4_radius_mm, 23.25);
  EXPECT_DOUBLE_EQ(parameters.axis4.joint_radius_mm, 33.94);
  EXPECT_DOUBLE_EQ(parameters.axis5.joint_radius_mm, 33.94);
  EXPECT_DOUBLE_EQ(parameters.axis6_branch_a.joint2_guide_radius_mm, 20.25);
  EXPECT_DOUBLE_EQ(parameters.output_limits[5].upper_position_rad, 6.027);
  std::filesystem::remove(path);
}

TEST(ConfigLoader, LoadsDirectionsAndRejectsInvalidOrIncompleteMapping)
{
  const std::string prefix = "  output_directions:\n    motor_01: 1\n    motor_02: 1\n";
  const std::string suffix = "    motor_04: -1\n    motor_05: 1\n    motor_06: 1\n";
  for (const std::string value : {"-1", "0", "2", "1.5", "true", "missing"}) {
    std::string contents = valid_config();
    contents.insert(contents.find("  axes:"), prefix +
      (value == "missing" ? "" : "    motor_03: " + value + "\n") + suffix);
    const auto path = write_config(contents, "pi_arm_output_directions.yaml");
    if (value == "-1") {
      const auto config = pi_arm_transmission::load_cable_kinematics_config(path.string());
      EXPECT_EQ(config.output_directions, (std::array<int, 6>{{1, 1, -1, -1, 1, 1}}));
    } else {
      EXPECT_THROW(pi_arm_transmission::load_cable_kinematics_config(path.string()),
        std::invalid_argument);
    }
    std::filesystem::remove(path);
  }
}

TEST(ConfigLoader, RejectsOldModelVersion)
{
  std::string contents = valid_config();
  const std::string current = "  model_version: 3";
  const auto marker = contents.find(current);
  ASSERT_NE(marker, std::string::npos);
  contents.replace(marker, current.size(), "  model_version: 1");
  const auto path = write_config(contents, "pi_arm_old_model_version.yaml");
  EXPECT_THROW(
    pi_arm_transmission::load_cable_kinematics_config(path.string()),
    std::invalid_argument);
  std::filesystem::remove(path);
}

TEST(ConfigLoader, RejectsMissingAxis)
{
  std::string contents = valid_config();
  const auto begin = contents.find("    joint_04:");
  const auto end = contents.find("    joint_05:");
  ASSERT_NE(begin, std::string::npos);
  ASSERT_NE(end, std::string::npos);
  contents.erase(begin, end - begin);
  const auto path = write_config(contents, "pi_arm_missing_axis_cable_kinematics.yaml");
  EXPECT_THROW(
    pi_arm_transmission::load_cable_kinematics_config(path.string()),
    std::invalid_argument);
  std::filesystem::remove(path);
}

TEST(ConfigLoader, RejectsUnknownField)
{
  std::string contents = valid_config();
  const auto marker = contents.find("      joint_radius_mm: 19.75");
  ASSERT_NE(marker, std::string::npos);
  contents.insert(marker, "      obsolete_ratio: 1.0\n");
  const auto path = write_config(contents, "pi_arm_unknown_field_cable_kinematics.yaml");
  EXPECT_THROW(
    pi_arm_transmission::load_cable_kinematics_config(path.string()),
    std::invalid_argument);
  std::filesystem::remove(path);
}

TEST(ConfigLoader, RejectsIncompatibleAxis6Branches)
{
  std::string contents = valid_config();
  const std::string correct = "        joint2_guide_radius_mm: 20.25";
  const auto marker = contents.find(correct);
  ASSERT_NE(marker, std::string::npos);
  contents.replace(marker, correct.size(), "        joint2_guide_radius_mm: 21.0");
  const auto path = write_config(contents, "pi_arm_bad_axis6_cable_kinematics.yaml");
  EXPECT_THROW(
    pi_arm_transmission::load_cable_kinematics_config(path.string()),
    std::invalid_argument);
  std::filesystem::remove(path);
}

TEST(ConfigLoader, RejectsMissingOutputLimit)
{
  std::string contents = valid_config();
  const auto begin = contents.find("    motor_04:");
  const auto end = contents.find("    motor_05:");
  ASSERT_NE(begin, std::string::npos);
  ASSERT_NE(end, std::string::npos);
  contents.erase(begin, end - begin);
  const auto path = write_config(contents, "pi_arm_missing_output_limit.yaml");
  EXPECT_THROW(
    pi_arm_transmission::load_cable_kinematics_config(path.string()),
    std::invalid_argument);
  std::filesystem::remove(path);
}

TEST(ConfigLoader, RejectsInvalidOutputLimitOrder)
{
  std::string contents = valid_config();
  const std::string current =
    "    motor_02: {lower_position_rad: -1.746, upper_position_rad: 1.746}";
  const auto marker = contents.find(current);
  ASSERT_NE(marker, std::string::npos);
  contents.replace(
    marker, current.size(),
    "    motor_02: {lower_position_rad: 2.0, upper_position_rad: 1.0}");
  const auto path = write_config(contents, "pi_arm_bad_output_limit.yaml");
  EXPECT_THROW(
    pi_arm_transmission::load_cable_kinematics_config(path.string()),
    std::invalid_argument);
  std::filesystem::remove(path);
}

}  // namespace
