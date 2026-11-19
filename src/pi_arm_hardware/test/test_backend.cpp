#include "pi_arm_hardware/backend.hpp"
#include "pi_arm_hardware/actuator_coordinates.hpp"
#include "pi_arm_hardware/calibration.hpp"
#include "pi_arm_hardware/software_zero.hpp"
#include "pi_arm_transmission/config_loader.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>

#include <string>
#include <vector>

namespace
{

pi_arm_hardware::BackendConfig make_config()
{
  pi_arm_hardware::BackendConfig config;
  config.cable_kinematics_file = PI_ARM_TEST_CABLE_CONFIG;
  const std::vector<std::uint8_t> motor_ids{1U, 2U, 3U, 4U, 5U, 6U};
  const std::vector<std::string> joint_names{
    "joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"};
  for (std::size_t index = 0; index < motor_ids.size(); ++index) {
    config.actuators.push_back(
      pi_arm_hardware::ActuatorConfig{
        joint_names[index], motor_ids[index], index < 3U ? 36.0 : 6.0, 1});
  }
  // In production these limits are filled from the URDF by the hardware plugin.
  config.joint_lower_limits_rad.assign(motor_ids.size(), -3.14);
  config.joint_upper_limits_rad.assign(motor_ids.size(), 3.14);
  config.joint_velocity_limits_rad_s.assign(motor_ids.size(), 0.17453292519943295);
  return config;
}

TEST(ActuatorCoordinates, RoundTripsProtocolAndOutputCoordinates)
{
  const pi_arm_hardware::ActuatorConfig actuator{"joint_04", 3U, 36.0, -1};
  pi_arm_hardware::validate_actuator_config(actuator);
  const double protocol =
    pi_arm_hardware::output_position_to_protocol_deg(0.25, actuator);
  EXPECT_LT(protocol, 0.0);
  EXPECT_NEAR(
    pi_arm_hardware::protocol_position_to_output_rad(protocol, actuator),
    0.25, 1.0e-12);
  EXPECT_NEAR(
    pi_arm_hardware::protocol_velocity_to_output_rad_s(
      pi_arm_hardware::output_velocity_to_protocol_dps(-0.4, actuator), actuator),
    -0.4, 1.0e-12);
}

TEST(MockBackend, FollowsPositionAndVelocityCommands)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  const std::vector<double> positions{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  const std::vector<double> velocities{0.0, -0.02, 0.03, 0.0, 0.05, 0.06};
  ASSERT_TRUE(backend.write(positions, velocities));

  ASSERT_TRUE(backend.read(sample));
  ASSERT_EQ(sample.positions_rad.size(), positions.size());
  ASSERT_EQ(sample.velocities_rad_s.size(), velocities.size());
  for (std::size_t index = 0; index < positions.size(); ++index) {
    EXPECT_NEAR(sample.positions_rad[index], positions[index], 1.0e-12);
    EXPECT_NEAR(sample.velocities_rad_s[index], velocities[index], 1.0e-12);
  }
  EXPECT_FALSE(sample.in_motion[0]);
  EXPECT_TRUE(sample.in_motion[1]);
}

TEST(MockBackend, ClampsVelocityAboveLimit)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  const std::vector<double> positions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> velocities{0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
  ASSERT_TRUE(backend.write(positions, velocities));

  ASSERT_TRUE(backend.read(sample));
  EXPECT_NEAR(sample.velocities_rad_s[0], 0.17453292519943295, 1e-9);
  EXPECT_TRUE(sample.in_motion[0]);
}

TEST(MockBackend, SoftStopHoldsCurrentPoseAgainstOldControllerWrites)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int i = 0; i < 3; ++i) {ASSERT_TRUE(backend.read(sample));}
  ASSERT_TRUE(backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  std::vector<double> q(6, 0.0), v(6, 0.0);
  q[0] = .1; v[0] = .01;
  ASSERT_TRUE(backend.write(q, v));
  ASSERT_TRUE(backend.set_motion_armed(false, 0, 0.0, true).success);
  q[0] = .2;
  ASSERT_TRUE(backend.write(q, v));
  ASSERT_TRUE(backend.read(sample));
  EXPECT_DOUBLE_EQ(sample.positions_rad[0], .1);
  EXPECT_DOUBLE_EQ(sample.velocities_rad_s[0], 0.0);
  EXPECT_FALSE(sample.in_motion[0]);
  ASSERT_TRUE(backend.set_motion_armed(true, 2).success);
  ASSERT_TRUE(backend.write(q, v));
  ASSERT_TRUE(backend.read(sample));
  EXPECT_DOUBLE_EQ(sample.positions_rad[0], .2);
}

TEST(MockBackend, DeactivateClearsMotionAndDisconnects)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  const std::vector<double> positions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> velocities{0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
  ASSERT_TRUE(backend.write(positions, velocities));

  backend.deactivate();
  ASSERT_TRUE(backend.read(sample));
  EXPECT_FALSE(sample.connected);
  for (const double velocity : sample.velocities_rad_s) {
    EXPECT_DOUBLE_EQ(velocity, 0.0);
  }
  for (const bool moving : sample.in_motion) {
    EXPECT_FALSE(moving);
  }
  for (const bool fresh : sample.fresh) {
    EXPECT_FALSE(fresh);
  }
  EXPECT_FALSE(backend.set_motion_armed(true, 1U).success);
  EXPECT_FALSE(backend.set_motion_armed(false, 0U, 0.0, true).success);
  EXPECT_TRUE(backend.set_motion_armed(false, 0U).success);
  backend.activate();
  EXPECT_TRUE(backend.set_motion_armed(true, 2U).success);
}

TEST(MockBackend, ReactivationKeepsTeachingMotionInhibited)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int i = 0; i < 3; ++i) {ASSERT_TRUE(backend.read(sample));}
  ASSERT_TRUE(backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  ASSERT_TRUE(backend.set_teaching(true).success);
  backend.deactivate();
  backend.activate();
  ASSERT_TRUE(backend.read(sample));
  EXPECT_TRUE(sample.teaching_active);
  EXPECT_FALSE(sample.control_permitted);
  EXPECT_FALSE(backend.set_motion_armed(true, 1U).success);
}

TEST(MockBackend, RejectsWholeCommandWhenMappedOutputExceedsLimit)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);

  // This is inside the deliberately broad test joint limit, but axis 3 maps
  // beyond motor_03's configured reducer-output limit.
  const std::vector<double> positions{0.0, 0.0, 3.0, 0.0, 0.0, 0.0};
  const std::vector<double> velocities(6U, 0.0);
  EXPECT_FALSE(backend.write(positions, velocities));
  ASSERT_TRUE(backend.read(sample));
  EXPECT_NE(sample.error_message.find("motor_03"), std::string::npos);
  for (const double position : sample.positions_rad) {
    EXPECT_DOUBLE_EQ(position, 0.0);
  }
}

TEST(MockBackend, SeparatesRawMotorAndJointDependencyState)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 4).success);
  ASSERT_TRUE(backend.read(sample));
  EXPECT_TRUE(sample.motor_enabled[3]);
  EXPECT_FALSE(sample.enabled[3]);  // joint_05 also depends on motor 2.
  EXPECT_EQ(sample.driver_mode, "MONITORING");

  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  ASSERT_TRUE(backend.read(sample));
  EXPECT_TRUE(std::all_of(sample.motor_enabled.begin(), sample.motor_enabled.end(), [](bool v) {
    return v;
  }));
  EXPECT_TRUE(std::all_of(sample.enabled.begin(), sample.enabled.end(), [](bool v) {return v;}));
  EXPECT_EQ(sample.driver_mode, "CONTROL");

  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_OFF, 2).success);
  ASSERT_TRUE(backend.read(sample));
  const std::vector<bool> expected_motors{true, false, true, true, true, true};
  const std::vector<bool> expected_joints{true, false, false, false, false, false};
  EXPECT_EQ(sample.motor_enabled, expected_motors);
  EXPECT_EQ(sample.enabled, expected_joints);
  EXPECT_EQ(sample.driver_mode, "MONITORING");

  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_OFF, 0).success);
  ASSERT_TRUE(backend.read(sample));
  EXPECT_TRUE(std::none_of(sample.motor_enabled.begin(), sample.motor_enabled.end(), [](bool v) {
    return v;
  }));

  EXPECT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_OFF, 2).success);
  EXPECT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 2).success);
  EXPECT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 2).success);
  EXPECT_FALSE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_OFF, 7).success);
  EXPECT_FALSE(
    backend.manage(
      pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO, 4).success);
}


TEST(ActuatorCoordinates, AppliesSoftwareOffsetOnlyInModelCoordinate)
{
  const pi_arm_hardware::ActuatorConfig actuator{"joint_01", 1U, 36.0, 1};
  constexpr double model_output = 0.7;
  constexpr double offset = 0.2;
  const double protocol = pi_arm_hardware::model_output_position_to_protocol_deg(
    model_output, offset, actuator);
  EXPECT_NEAR(
    pi_arm_hardware::protocol_position_to_model_output_rad(protocol, offset, actuator),
    model_output, 1.0e-12);
  EXPECT_NEAR(
    pi_arm_hardware::model_to_rom_relative_output_rad(model_output, offset), 0.5, 1.0e-12);
  EXPECT_NEAR(
    pi_arm_hardware::rom_relative_to_model_output_rad(0.5, offset), model_output, 1.0e-12);
}

TEST(MockBackend, PublishesExplicitMockCalibrationState)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  ASSERT_TRUE(backend.read(sample));
  EXPECT_TRUE(sample.calibrated);
  EXPECT_TRUE(sample.kinematics_valid);
  EXPECT_EQ(sample.calibration_id, "MOCK");
  EXPECT_FALSE(sample.control_permitted);
  EXPECT_EQ(sample.raw_protocol_positions_deg.size(), 6U);
  EXPECT_EQ(sample.model_output_positions_rad.size(), 6U);
}

TEST(Calibration, UnapprovedCandidateFailsClosedAndApprovedCandidateComputesOffsets)
{
  const auto config = make_config();
  const auto parameters =
    pi_arm_transmission::load_cable_kinematics_config(config.cable_kinematics_file);
  const pi_arm_transmission::CableKinematics kinematics(parameters);
  const auto fingerprint = pi_arm_hardware::calibration_configuration_fingerprint(
    config.cable_kinematics_file, config.actuators);
  const auto path = std::filesystem::temp_directory_path() /
    "pi_arm_pack_reference_calibration_test.yaml";

  const auto write_candidate = [&](const bool approved) {
    std::ofstream stream(path);
    stream << "calibration:\n"
           << "  schema_version: 1\n"
           << "  robot_id: test-robot\n"
           << "  calibration_id: test-calibration\n"
           << "  approved: " << (approved ? "true" : "false") << "\n"
           << "  model_version: 3\n"
           << "  created_at_utc: 2026-09-14T00:00:00Z\n"
           << "  configuration_fingerprint: " << fingerprint << "\n"
           << "  joint_names: [joint_01, joint_02, joint_04, joint_05, joint_06, joint_09]\n"
           << "  motor_ids: [1, 2, 3, 4, 5, 6]\n"
           << "  packing_pose_joint_rad: [0, 0, 0, 0, 0, 0]\n"
           << "  raw_output_at_packing_rad: [0, 0, 0, 0, 0, 0]\n";
  };

  write_candidate(false);
  auto result = pi_arm_hardware::load_pack_reference_calibration(
    path.string(), config.cable_kinematics_file, config.actuators, kinematics);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.message.find("approved is false"), std::string::npos);

  write_candidate(true);
  result = pi_arm_hardware::load_pack_reference_calibration(
    path.string(), config.cable_kinematics_file, config.actuators, kinematics);
  EXPECT_TRUE(result.valid) << result.message;
  for (const double offset : result.software_output_offsets_rad) {
    EXPECT_NEAR(offset, 0.0, 1.0e-12);
  }
  std::filesystem::remove(path);
}

class SoftwareZero : public ::testing::Test
{
protected:
  SoftwareZero()
  : config(make_config()), kinematics(
      pi_arm_transmission::load_cable_kinematics_config(config.cable_kinematics_file))
  {
    config.actuators[2].motor_direction = -1;
    config.actuators[3].motor_direction = -1;
    config.actuators[4].motor_direction = -1;
    base.valid = true;
    base.data.approved = true;
    base.data.schema_version = 1;
    base.data.model_version = 3;
    base.data.created_at_utc = "2026-11-01T00:00:00Z";
    for (std::size_t i = 0; i < config.actuators.size(); ++i) {
      base.data.joint_names[i] = config.actuators[i].joint_name;
    }
    base.data.robot_id = "zero-test-robot";
    base.data.calibration_id = "base-reference";
    base.data.configuration_fingerprint = pi_arm_hardware::calibration_configuration_fingerprint(
      config.cable_kinematics_file, config.actuators);
    base.data.motor_ids = {1, 2, 3, 4, 5, 6};
    std::string pattern = (std::filesystem::temp_directory_path() / "pi-arm-zero-XXXXXX").string();
    if (!::mkdtemp(pattern.data())) {throw std::runtime_error("cannot create test directory");}
    directory = pattern;
    path = directory / "zero.yaml";
  }
  void TearDown() override {std::filesystem::remove_all(directory);}
  std::string contents() const
  {
    std::ifstream file(path);
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
  }
  pi_arm_hardware::CalibrationResult save()
  {
    return pi_arm_hardware::save_software_zero_reference(path.string(), base,
      reference, protocol, config.actuators, kinematics, "2026-11-16T00:00:00Z");
  }
  pi_arm_hardware::BackendConfig config;
  pi_arm_transmission::CableKinematics kinematics;
  pi_arm_hardware::CalibrationResult base;
  pi_arm_hardware::CalibrationVector6 reference{0.2, 0.1, -0.05, 0.02, -0.02, 0.1};
  pi_arm_hardware::CalibrationVector6 protocol{1800, -3600, 500, 420, -600, 360};
  std::filesystem::path directory, path;
};

TEST_F(SoftwareZero, PersistsReferenceAndRoundTripsCoupledTargets)
{
  const auto saved = save();
  const auto loaded = pi_arm_hardware::load_software_zero_reference(
    path.string(), base, config.actuators, kinematics);
  EXPECT_FALSE(YAML::LoadFile(path.string())["software_zero"]["reference_id"]);
  EXPECT_EQ(loaded.software_output_offsets_rad, saved.software_output_offsets_rad);
  EXPECT_EQ(loaded.data.raw_output_at_packing_rad, base.data.raw_output_at_packing_rad);
  pi_arm_transmission::Vector6 output{}, recovered{};
  for (std::size_t i = 0; i < 6; ++i) {
    output[i] = pi_arm_hardware::protocol_position_to_model_output_rad(
      protocol[i], loaded.software_output_offsets_rad[i], config.actuators[i]);
    EXPECT_NEAR(pi_arm_hardware::model_output_position_to_protocol_deg(
      output[i], loaded.software_output_offsets_rad[i], config.actuators[i]), protocol[i], 1e-9);
  }
  ASSERT_TRUE(kinematics.output_to_joint_position(output, recovered));
  for (std::size_t i = 0; i < 6; ++i) {EXPECT_NEAR(recovered[i], reference[i], 1e-9);}
  protocol[0] += 36.0;
  const auto replaced = save();
  EXPECT_NEAR(replaced.software_output_offsets_rad[0] - saved.software_output_offsets_rad[0],
    -3.14159265358979323846 / 180.0, 1e-12);
  EXPECT_EQ(pi_arm_hardware::load_software_zero_reference(
    path.string(), base, config.actuators, kinematics).software_output_offsets_rad, replaced.software_output_offsets_rad);
}

TEST_F(SoftwareZero, InvalidCandidateLeavesCommittedReferenceUnchanged)
{
  save();
  const auto before = contents();
  protocol[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(save(), std::exception);
  EXPECT_EQ(contents(), before);
  auto mismatched = base;
  mismatched.data.configuration_fingerprint = "different";
  EXPECT_THROW(pi_arm_hardware::load_software_zero_reference(
    path.string(), mismatched, config.actuators, kinematics), std::exception);
  auto reversed = config.actuators;
  std::swap(reversed[0], reversed[1]);
  EXPECT_THROW(pi_arm_hardware::load_software_zero_reference(
    path.string(), base, reversed, kinematics), std::exception);
}

TEST_F(SoftwareZero, SingleFileLoadsAloneAndLegacyIsUpgradedOnlyOnCapture)
{
  const auto saved = save();
  const auto single = pi_arm_hardware::load_pack_reference_calibration(
    path.string(), config.cable_kinematics_file, config.actuators, kinematics);
  ASSERT_TRUE(single.valid) << single.message;
  EXPECT_EQ(single.software_output_offsets_rad, saved.software_output_offsets_rad);

  auto document = YAML::LoadFile(path.string());
  auto zero = document["software_zero"];
  zero["schema_version"] = 2;
  zero["configuration_fingerprint"] = base.data.configuration_fingerprint;
  zero["reference_id"] = "legacy-zero";
  zero.remove("joint_names");
  zero["robot_id"] = base.data.robot_id;
  zero["base_calibration_id"] = base.data.calibration_id;
  {std::ofstream out(path); out << document;}
  EXPECT_EQ(pi_arm_hardware::load_pack_reference_calibration(
    path.string(), config.cable_kinematics_file, config.actuators, kinematics).software_output_offsets_rad,
    saved.software_output_offsets_rad);
  document.remove("calibration");
  {std::ofstream out(path); out << document;}
  EXPECT_THROW(pi_arm_hardware::load_software_zero_reference(
    path.string(), base, config.actuators, kinematics), std::exception);
  document["software_zero"]["schema_version"] = 1;
  {std::ofstream out(path); out << document;}
  const auto old_bytes = contents();
  const auto legacy = pi_arm_hardware::load_software_zero_reference(
    path.string(), base, config.actuators, kinematics);
  EXPECT_EQ(legacy.software_output_offsets_rad, saved.software_output_offsets_rad);
  EXPECT_EQ(contents(), old_bytes);
  save();
  EXPECT_EQ(pi_arm_hardware::load_pack_reference_calibration(
    path.string(), config.cable_kinematics_file, config.actuators, kinematics).software_output_offsets_rad,
    saved.software_output_offsets_rad);
  EXPECT_FALSE(YAML::LoadFile(path.string())["software_zero"]["reference_id"]);
}

TEST_F(SoftwareZero, LostFileCanBeRebuiltFromThePoseAndNewEncoderReadings)
{
  const auto fingerprint = base.data.configuration_fingerprint;
  auto first = pi_arm_hardware::capture_pose_zero(path.string(), fingerprint, reference, protocol,
    config.actuators, kinematics, "2026-11-19T00:00:00Z");
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(std::filesystem::remove(path));
  protocol[0] += 72.0;
  const auto next = pi_arm_hardware::capture_pose_zero(path.string(), fingerprint, reference,
    protocol, config.actuators, kinematics, "2026-11-19T00:01:00Z");
  const auto loaded = pi_arm_hardware::load_pack_reference_calibration(
    path.string(), config.cable_kinematics_file, config.actuators, kinematics);
  ASSERT_TRUE(loaded.valid) << loaded.message;
  EXPECT_EQ(loaded.software_output_offsets_rad, next.software_output_offsets_rad);
  EXPECT_NEAR(next.software_output_offsets_rad[0] - first.software_output_offsets_rad[0],
    -2.0 * 3.14159265358979323846 / 180.0, 1.0e-12);
}

TEST_F(SoftwareZero, ConfirmedRecaptureAfterDirectionChangeReplacesTheSingleFile)
{
  const auto capture = [&]() {
      const auto fingerprint = pi_arm_hardware::calibration_configuration_fingerprint(
        config.cable_kinematics_file, config.actuators);
      return pi_arm_hardware::capture_pose_zero(path.string(), fingerprint, reference, protocol,
        config.actuators, kinematics, "2026-11-19T00:00:00Z");
    };
  const auto load = [&]() {
      return pi_arm_hardware::load_pack_reference_calibration(
        path.string(), config.cable_kinematics_file, config.actuators, kinematics);
    };
  ASSERT_TRUE(capture().valid);
  const auto before = contents();
  config.actuators[1].motor_direction *= -1;
  EXPECT_TRUE(load().valid);
  protocol[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(capture(), std::exception);
  EXPECT_EQ(contents(), before);
  protocol[1] = 5832.0;
  ASSERT_TRUE(capture().valid);
  const auto current = load();
  ASSERT_TRUE(current.valid) << current.message;
  EXPECT_FALSE(YAML::LoadFile(path.string())["software_zero"]["configuration_fingerprint"]);
  pi_arm_transmission::Vector6 output{};
  ASSERT_TRUE(kinematics.joint_to_output_position(reference, output));
  for (std::size_t i = 0; i < protocol.size(); ++i) {
    EXPECT_NEAR(pi_arm_hardware::protocol_position_to_model_output_rad(
      protocol[i], current.software_output_offsets_rad[i], config.actuators[i]), output[i], 1e-9);
  }
  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    EXPECT_EQ(entry.path(), path);
  }
}

TEST_F(SoftwareZero, ConfirmedCaptureReplacesEarlierPackingMetadata)
{
  save();
  auto previous = YAML::LoadFile(path.string());
  previous["calibration"]["calibration_id"] = "earlier-packing-reference";
  previous["calibration"]["robot_id"] = "earlier-installation";
  {std::ofstream out(path); out << previous;}
  ASSERT_TRUE(save().valid);
  const auto current = YAML::LoadFile(path.string());
  EXPECT_EQ(current["calibration"]["calibration_id"].as<std::string>(), base.data.calibration_id);
  EXPECT_EQ(current["calibration"]["robot_id"].as<std::string>(), base.data.robot_id);
}

TEST_F(SoftwareZero, MissingCorruptOrUnapprovedReferenceIsRejected)
{
  EXPECT_THROW(pi_arm_hardware::load_software_zero_reference(
    path.string(), base, config.actuators, kinematics), std::exception);
  base.valid = false;
  EXPECT_THROW(save(), std::exception);
  EXPECT_FALSE(std::filesystem::exists(path));
  base.valid = true;
  {std::ofstream file(path); file << "unrelated: file\n";}
  EXPECT_THROW(save(), std::exception);
  EXPECT_EQ(contents(), "unrelated: file\n");
  const auto blocker = directory / "not-a-directory";
  {std::ofstream file(blocker); file << "keep\n";}
  path = blocker / "zero.yaml";
  EXPECT_THROW(save(), std::exception);
}

}  // namespace

TEST(MockBackend, TeachingBlocksMotionAndExitsWithoutReplayingControllerTarget)
{
  pi_arm_hardware::MockBackend b(make_config()); b.configure(); b.activate();
  pi_arm_hardware::HardwareSample s;
  for (int i=0; i<3; ++i) {b.read(s);}
  ASSERT_TRUE(b.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  ASSERT_TRUE(b.set_teaching(true).success);
  ASSERT_FALSE(b.set_motion_armed(true, 1).success);
  ASSERT_FALSE(b.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  ASSERT_TRUE(b.set_teaching(false).success);
  ASSERT_TRUE(b.write(std::vector<double>(6, 0.1), std::vector<double>(6, 0.0)));
  b.read(s);
  EXPECT_EQ(s.positions_rad, std::vector<double>(6, 0.0));
  ASSERT_TRUE(b.set_motion_armed(true, 2).success);
  ASSERT_TRUE(b.write(std::vector<double>(6, 0.1), std::vector<double>(6, 0.0)));
  b.read(s);
  EXPECT_NEAR(s.positions_rad[0], 0.1, 1e-10);
}

TEST(MockBackend, WristSwapRoutesMotorAvailabilityToCorrectJoint)
{
  const auto path=std::filesystem::temp_directory_path()/"pi_arm_test_swapped_wrist.yaml";
  std::ifstream source(PI_ARM_TEST_CABLE_CONFIG);
  std::string text((std::istreambuf_iterator<char>(source)),std::istreambuf_iterator<char>());
  text.insert(text.find("cable_kinematics:")+std::string("cable_kinematics:").size(),
    "\n  swap_wrist_channels: true");
  {std::ofstream stream(path); stream<<text;}
  auto config=make_config(); config.cable_kinematics_file=path.string();
  pi_arm_hardware::MockBackend b(config); b.configure(); b.activate();
  pi_arm_hardware::HardwareSample sample;
  for(int i=0;i<3;++i) {b.read(sample);}
  ASSERT_TRUE(b.manage(pi_arm_can::ManagementOperation::MOTOR_RUN,0).success);
  ASSERT_TRUE(b.manage(pi_arm_can::ManagementOperation::MOTOR_OFF,4).success);
  b.read(sample);
  EXPECT_TRUE(sample.enabled[3]); EXPECT_FALSE(sample.enabled[4]);
  EXPECT_FALSE(sample.motor_enabled[3]); EXPECT_TRUE(sample.motor_enabled[4]);
  std::filesystem::remove(path);
}
