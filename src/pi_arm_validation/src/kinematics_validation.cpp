#include "pi_arm_transmission/cable_kinematics.hpp"
#include "pi_arm_transmission/config_loader.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{

constexpr std::size_t kAxisCount = 6U;
constexpr double kFiniteDifferenceStep = 1.0e-7;

struct Options
{
  std::filesystem::path config_path;
  std::filesystem::path output_directory{"validation_results"};
  std::size_t samples{5000U};
  std::size_t sweep_points{101U};
  std::uint64_t seed{20260730U};
  double joint_range_rad{1.4};
  double velocity_range_rad_s{1.0};
};

struct Summary
{
  double max_position_error_rad{0.0};
  double max_velocity_error_rad_s{0.0};
  double max_jacobian_error{0.0};
  double max_axis6_residual_mm{0.0};
};

void print_usage(const char * const program)
{
  std::cout
    << "Usage: " << program << " [options]\n"
    << "  --config PATH          Cable kinematics YAML. Defaults to installed "
       "pi_arm_description config.\n"
    << "  --output-dir PATH      Output directory (default: validation_results).\n"
    << "  --samples N            Random round-trip samples (default: 5000).\n"
    << "  --sweep-points N       Points per sweep dimension, minimum 3 (default: 101).\n"
    << "  --seed N               Deterministic random seed (default: 20260730).\n"
    << "  --joint-range RAD      Symmetric active-joint sampling range, below pi/2 "
       "(default: 1.4).\n"
    << "  --velocity-range RAD_S Symmetric velocity sampling range (default: 1.0).\n"
    << "  --help                 Show this help.\n";
}

std::string require_argument(
  const int argc, char ** const argv, int & index, const std::string & option)
{
  if (index + 1 >= argc) {
    throw std::invalid_argument(option + " requires a value");
  }
  ++index;
  return argv[index];
}

std::size_t parse_size(const std::string & text, const std::string & option)
{
  std::size_t parsed = 0U;
  const auto value = std::stoull(text, &parsed);
  if (parsed != text.size() || value == 0U) {
    throw std::invalid_argument(option + " must be a positive integer");
  }
  return static_cast<std::size_t>(value);
}

std::uint64_t parse_seed(const std::string & text)
{
  std::size_t parsed = 0U;
  const auto value = std::stoull(text, &parsed);
  if (parsed != text.size()) {
    throw std::invalid_argument("--seed must be an unsigned integer");
  }
  return static_cast<std::uint64_t>(value);
}

double parse_positive_double(const std::string & text, const std::string & option)
{
  std::size_t parsed = 0U;
  const double value = std::stod(text, &parsed);
  if (parsed != text.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(option + " must be a finite positive number");
  }
  return value;
}

Options parse_options(const int argc, char ** const argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--help") {
      print_usage(argv[0]);
      std::exit(0);
    }
    if (option == "--config") {
      options.config_path = require_argument(argc, argv, index, option);
    } else if (option == "--output-dir") {
      options.output_directory = require_argument(argc, argv, index, option);
    } else if (option == "--samples") {
      options.samples = parse_size(require_argument(argc, argv, index, option), option);
    } else if (option == "--sweep-points") {
      options.sweep_points =
        parse_size(require_argument(argc, argv, index, option), option);
    } else if (option == "--seed") {
      options.seed = parse_seed(require_argument(argc, argv, index, option));
    } else if (option == "--joint-range") {
      options.joint_range_rad =
        parse_positive_double(require_argument(argc, argv, index, option), option);
    } else if (option == "--velocity-range") {
      options.velocity_range_rad_s =
        parse_positive_double(require_argument(argc, argv, index, option), option);
    } else {
      throw std::invalid_argument("unknown option '" + option + "'");
    }
  }

  if (options.sweep_points < 3U) {
    throw std::invalid_argument("--sweep-points must be at least 3");
  }
  constexpr double kHalfPi = 1.57079632679489661923;
  if (options.joint_range_rad >= kHalfPi) {
    throw std::invalid_argument("--joint-range must be smaller than pi/2");
  }
  if (options.config_path.empty()) {
    options.config_path =
      std::filesystem::path(
      ament_index_cpp::get_package_share_directory("pi_arm_description")) /
      "config" / "cable_kinematics.yaml";
  }
  return options;
}

std::ofstream open_output(
  const std::filesystem::path & directory, const std::string & filename)
{
  const auto path = directory / filename;
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open output file '" + path.string() + "'");
  }
  stream << std::setprecision(17);
  return stream;
}

void require_status(
  const pi_arm_transmission::KinematicsStatus & status, const std::string & context)
{
  if (status) {
    return;
  }
  std::ostringstream message;
  message
    << context << " failed: "
    << pi_arm_transmission::to_string(status.error)
    << " axis=" << status.axis
    << " value=" << status.value;
  throw std::runtime_error(message.str());
}

double sweep_value(
  const std::size_t index, const std::size_t count, const double magnitude)
{
  return -magnitude +
         2.0 * magnitude * static_cast<double>(index) /
         static_cast<double>(count - 1U);
}

std::string json_escape(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += character;
        break;
    }
  }
  return escaped;
}

Summary generate_random_validation(
  const pi_arm_transmission::CableKinematics & kinematics,
  const Options & options,
  std::array<std::array<double, kAxisCount>, kAxisCount> & max_jacobian_error)
{
  auto position_stream =
    open_output(options.output_directory, "position_roundtrip.csv");
  auto velocity_stream =
    open_output(options.output_directory, "velocity_roundtrip.csv");

  position_stream << "sample";
  velocity_stream << "sample";
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    position_stream << ",joint_" << axis + 1U << "_abs_error_rad";
    velocity_stream << ",joint_" << axis + 1U << "_abs_error_rad_s";
  }
  position_stream << '\n';
  velocity_stream << '\n';

  std::mt19937_64 random(options.seed);
  std::uniform_real_distribution<double> joint_distribution(
    -options.joint_range_rad, options.joint_range_rad);
  std::uniform_real_distribution<double> velocity_distribution(
    -options.velocity_range_rad_s, options.velocity_range_rad_s);

  Summary summary;
  max_jacobian_error = {};
  for (std::size_t sample = 0; sample < options.samples; ++sample) {
    pi_arm_transmission::Vector6 joint{};
    pi_arm_transmission::Vector6 joint_velocity{};
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      joint[axis] = joint_distribution(random);
      joint_velocity[axis] = velocity_distribution(random);
    }

    pi_arm_transmission::Vector6 output{};
    pi_arm_transmission::Vector6 recovered_joint{};
    require_status(
      kinematics.joint_to_output_position(joint, output),
      "joint_to_output_position");
    require_status(
      kinematics.output_to_joint_position(output, recovered_joint),
      "output_to_joint_position");

    position_stream << sample;
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      const double error = std::abs(recovered_joint[axis] - joint[axis]);
      summary.max_position_error_rad =
        std::max(summary.max_position_error_rad, error);
      position_stream << ',' << error;
    }
    position_stream << '\n';

    pi_arm_transmission::Vector6 output_velocity{};
    pi_arm_transmission::Vector6 recovered_velocity{};
    require_status(
      kinematics.joint_to_output_velocity(joint, joint_velocity, output_velocity),
      "joint_to_output_velocity");
    require_status(
      kinematics.output_to_joint_velocity(
        recovered_joint, output_velocity, recovered_velocity),
      "output_to_joint_velocity");

    velocity_stream << sample;
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      const double error = std::abs(recovered_velocity[axis] - joint_velocity[axis]);
      summary.max_velocity_error_rad_s =
        std::max(summary.max_velocity_error_rad_s, error);
      velocity_stream << ',' << error;
    }
    velocity_stream << '\n';

    pi_arm_transmission::Matrix6 analytic{};
    require_status(kinematics.jacobian(joint, analytic), "jacobian");
    for (std::size_t column = 0; column < kAxisCount; ++column) {
      auto plus = joint;
      auto minus = joint;
      plus[column] += kFiniteDifferenceStep;
      minus[column] -= kFiniteDifferenceStep;
      pi_arm_transmission::Vector6 plus_output{};
      pi_arm_transmission::Vector6 minus_output{};
      require_status(
        kinematics.joint_to_output_position(plus, plus_output),
        "finite-difference plus position");
      require_status(
        kinematics.joint_to_output_position(minus, minus_output),
        "finite-difference minus position");
      for (std::size_t row = 0; row < kAxisCount; ++row) {
        const double numerical =
          (plus_output[row] - minus_output[row]) /
          (2.0 * kFiniteDifferenceStep);
        const double error = std::abs(analytic[row][column] - numerical);
        max_jacobian_error[row][column] =
          std::max(max_jacobian_error[row][column], error);
        summary.max_jacobian_error =
          std::max(summary.max_jacobian_error, error);
      }
    }

    const auto residual =
      kinematics.axis6_residual(joint[1], joint[5], output[5]);
    summary.max_axis6_residual_mm =
      std::max(
      summary.max_axis6_residual_mm,
      std::max(std::abs(residual.branch_a_mm), std::abs(residual.branch_b_mm)));
  }
  return summary;
}

void write_jacobian_errors(
  const Options & options,
  const std::array<std::array<double, kAxisCount>, kAxisCount> & errors)
{
  auto stream = open_output(options.output_directory, "jacobian_error.csv");
  stream << "row,column,max_abs_error\n";
  for (std::size_t row = 0; row < kAxisCount; ++row) {
    for (std::size_t column = 0; column < kAxisCount; ++column) {
      stream << row + 1U << ',' << column + 1U << ',' << errors[row][column] << '\n';
    }
  }
}

void write_joint2_coupling(
  const pi_arm_transmission::CableKinematics & kinematics, const Options & options)
{
  auto stream = open_output(options.output_directory, "joint2_coupling.csv");
  stream << "joint_02_rad";
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    stream << ",output_" << axis + 1U << "_rad";
  }
  stream << '\n';

  for (std::size_t index = 0; index < options.sweep_points; ++index) {
    pi_arm_transmission::Vector6 joint{};
    joint[1] = sweep_value(index, options.sweep_points, 1.0);
    pi_arm_transmission::Vector6 output{};
    require_status(
      kinematics.joint_to_output_position(joint, output),
      "joint2 coupling sweep");
    stream << joint[1];
    for (const double value : output) {
      stream << ',' << value;
    }
    stream << '\n';
  }
}

void write_axis45_nonlinearity(
  const pi_arm_transmission::CableKinematics & kinematics, const Options & options)
{
  auto stream = open_output(options.output_directory, "axis45_nonlinear.csv");
  stream
    << "active_joint_rad,output_4_self_rad,jacobian_4,"
       "output_5_self_rad,jacobian_5\n";

  for (std::size_t index = 0; index < options.sweep_points; ++index) {
    const double active_joint =
      sweep_value(index, options.sweep_points, options.joint_range_rad);
    pi_arm_transmission::Vector6 joint{};
    joint[3] = active_joint;
    joint[4] = active_joint;
    pi_arm_transmission::Vector6 output{};
    pi_arm_transmission::Matrix6 jacobian{};
    require_status(
      kinematics.joint_to_output_position(joint, output),
      "axis 4/5 nonlinearity sweep");
    require_status(kinematics.jacobian(joint, jacobian), "axis 4/5 sweep jacobian");
    stream
      << active_joint << ','
      << output[3] << ',' << jacobian[3][3] << ','
      << output[4] << ',' << jacobian[4][4] << '\n';
  }
}

double write_axis6_residuals(
  const pi_arm_transmission::CableKinematics & kinematics, const Options & options)
{
  auto stream = open_output(options.output_directory, "axis6_residual.csv");
  stream << "joint_02_rad,joint_09_rad,branch_a_residual_mm,branch_b_residual_mm\n";
  double maximum = 0.0;
  for (std::size_t row = 0; row < options.sweep_points; ++row) {
    const double joint2 = sweep_value(row, options.sweep_points, 1.0);
    for (std::size_t column = 0; column < options.sweep_points; ++column) {
      const double joint6 = sweep_value(column, options.sweep_points, 1.0);
      pi_arm_transmission::Vector6 joint{};
      joint[1] = joint2;
      joint[5] = joint6;
      pi_arm_transmission::Vector6 output{};
      require_status(
        kinematics.joint_to_output_position(joint, output),
        "axis 6 residual sweep");
      const auto residual =
        kinematics.axis6_residual(joint2, joint6, output[5]);
      maximum = std::max(
        maximum,
        std::max(std::abs(residual.branch_a_mm), std::abs(residual.branch_b_mm)));
      stream
        << joint2 << ',' << joint6 << ','
        << residual.branch_a_mm << ',' << residual.branch_b_mm << '\n';
    }
  }
  return maximum;
}

void write_summary(
  const Options & options, const Summary & summary)
{
  auto stream = open_output(options.output_directory, "summary.json");
  stream
    << "{\n"
    << "  \"model_version\": 2,\n"
    << "  \"config_path\": \""
    << json_escape(options.config_path.generic_string()) << "\",\n"
    << "  \"samples\": " << options.samples << ",\n"
    << "  \"sweep_points\": " << options.sweep_points << ",\n"
    << "  \"seed\": " << options.seed << ",\n"
    << "  \"joint_range_rad\": " << options.joint_range_rad << ",\n"
    << "  \"velocity_range_rad_s\": " << options.velocity_range_rad_s << ",\n"
    << "  \"finite_difference_step\": " << kFiniteDifferenceStep << ",\n"
    << "  \"max_position_error_rad\": " << summary.max_position_error_rad << ",\n"
    << "  \"max_velocity_error_rad_s\": " << summary.max_velocity_error_rad_s << ",\n"
    << "  \"max_jacobian_error\": " << summary.max_jacobian_error << ",\n"
    << "  \"max_axis6_residual_mm\": " << summary.max_axis6_residual_mm << "\n"
    << "}\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    std::filesystem::create_directories(options.output_directory);

    const pi_arm_transmission::CableKinematics kinematics(
      pi_arm_transmission::load_cable_kinematics_config(
        options.config_path.string()));

    std::array<std::array<double, kAxisCount>, kAxisCount> jacobian_errors{};
    auto summary =
      generate_random_validation(kinematics, options, jacobian_errors);
    write_jacobian_errors(options, jacobian_errors);
    write_joint2_coupling(kinematics, options);
    write_axis45_nonlinearity(kinematics, options);
    summary.max_axis6_residual_mm =
      std::max(
      summary.max_axis6_residual_mm,
      write_axis6_residuals(kinematics, options));
    write_summary(options, summary);

    std::cout
      << std::setprecision(6) << std::scientific
      << "PI_ARM_VALIDATION_OK"
      << " output=" << options.output_directory.string()
      << " position_error=" << summary.max_position_error_rad
      << " velocity_error=" << summary.max_velocity_error_rad_s
      << " jacobian_error=" << summary.max_jacobian_error
      << " axis6_residual_mm=" << summary.max_axis6_residual_mm
      << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "pi_arm_validation: " << error.what() << '\n';
    return 1;
  }
}
