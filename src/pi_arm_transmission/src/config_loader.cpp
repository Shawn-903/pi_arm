#include "pi_arm_transmission/config_loader.hpp"

#include "pi_arm_transmission/cable_kinematics.hpp"

#include <yaml-cpp/yaml.h>

#include <set>
#include <stdexcept>
#include <string>

namespace pi_arm_transmission
{
namespace
{

YAML::Node require_map(const YAML::Node & parent, const char * const key)
{
  const auto value = parent[key];
  if (!value || !value.IsMap()) {
    throw std::invalid_argument(std::string("missing map '") + key + "'");
  }
  return value;
}

template<typename T>
T require_value(const YAML::Node & parent, const char * const key)
{
  const auto value = parent[key];
  if (!value || !value.IsScalar()) {
    throw std::invalid_argument(std::string("missing scalar '") + key + "'");
  }
  try {
    return value.as<T>();
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            std::string("invalid value for '") + key + "': " + error.what());
  }
}

template<typename T>
T optional_value(const YAML::Node & parent, const char * const key, const T default_value)
{
  const auto value = parent[key];
  if (!value) {
    return default_value;
  }
  if (!value.IsScalar()) {
    throw std::invalid_argument(std::string("value '") + key + "' must be scalar");
  }
  try {
    return value.as<T>();
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            std::string("invalid value for '") + key + "': " + error.what());
  }
}

void reject_unknown_keys(
  const YAML::Node & map, const std::set<std::string> & allowed,
  const std::string & context)
{
  for (const auto & entry : map) {
    if (!entry.first.IsScalar()) {
      throw std::invalid_argument(context + " contains a non-scalar key");
    }
    const auto key = entry.first.as<std::string>();
    if (allowed.count(key) == 0U) {
      throw std::invalid_argument(context + " contains unknown key '" + key + "'");
    }
  }
}

void require_model(const YAML::Node & node, const std::string & expected)
{
  const auto model = require_value<std::string>(node, "model");
  if (model != expected) {
    throw std::invalid_argument(
            "expected model '" + expected + "' but got '" + model + "'");
  }
}

LinearAxisParameters parse_linear(const YAML::Node & node)
{
  reject_unknown_keys(
    node, {"model", "motor_radius_mm", "joint_radius_mm"}, "linear axis");
  require_model(node, "linear");
  return {
    require_value<double>(node, "motor_radius_mm"),
    require_value<double>(node, "joint_radius_mm")};
}

CoupledAxis3Parameters parse_coupled_axis3(const YAML::Node & node)
{
  reject_unknown_keys(
    node,
    {"model", "motor_radius_mm", "joint3_radius_mm", "joint4_radius_mm",
      "joint2_guide_radius_mm", "coupling_sign"},
    "coupled synchronized axis 3");
  require_model(node, "synchronized_pair_coupled");
  return {
    require_value<double>(node, "motor_radius_mm"),
    require_value<double>(node, "joint3_radius_mm"),
    require_value<double>(node, "joint4_radius_mm"),
    require_value<double>(node, "joint2_guide_radius_mm"),
    require_value<int>(node, "coupling_sign")};
}

CoupledChordAxisParameters parse_coupled_chord(const YAML::Node & node)
{
  reject_unknown_keys(
    node,
    {"model", "motor_radius_mm", "joint_radius_mm", "joint2_guide_radius_mm",
      "coupling_sign"},
    "coupled chord axis");
  require_model(node, "chord_coupled");
  return {
    require_value<double>(node, "motor_radius_mm"),
    require_value<double>(node, "joint_radius_mm"),
    require_value<double>(node, "joint2_guide_radius_mm"),
    require_value<int>(node, "coupling_sign")};
}

Axis6BranchParameters parse_axis6_branch(const YAML::Node & node)
{
  reject_unknown_keys(
    node,
    {"motor_radius_mm", "joint_radius_mm", "joint2_guide_radius_mm", "coupling_sign"},
    "axis6 branch");
  return {
    require_value<double>(node, "motor_radius_mm"),
    require_value<double>(node, "joint_radius_mm"),
    require_value<double>(node, "joint2_guide_radius_mm"),
    require_value<int>(node, "coupling_sign")};
}

OutputPositionLimits parse_output_limit(const YAML::Node & node, const std::string & name)
{
  reject_unknown_keys(
    node, {"lower_position_rad", "upper_position_rad"}, name + " output limit");
  return {
    require_value<double>(node, "lower_position_rad"),
    require_value<double>(node, "upper_position_rad")};
}

}  // namespace

CableKinematicsParameters load_cable_kinematics_config(const std::string & path)
{
  if (path.empty()) {
    throw std::invalid_argument("cable kinematics configuration path is empty");
  }

  YAML::Node document;
  try {
    document = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "failed to load cable kinematics configuration '" + path + "': " + error.what());
  }
  if (!document || !document.IsMap()) {
    throw std::invalid_argument("cable kinematics document must be a map");
  }
  reject_unknown_keys(document, {"cable_kinematics"}, "configuration root");

  const auto root = require_map(document, "cable_kinematics");
  reject_unknown_keys(
    root,
    {"model_version", "angle_unit", "length_unit", "axes", "output_limits", "validation",
      "output_directions", "swap_wrist_channels"},
    "cable_kinematics");
  const int model_version = require_value<int>(root, "model_version");
  if (model_version != 3) {
    throw std::invalid_argument("cable kinematics model_version must be 3");
  }
  if (optional_value<std::string>(root, "angle_unit", "rad") != "rad") {
    throw std::invalid_argument("cable kinematics angle_unit must be 'rad'");
  }
  if (optional_value<std::string>(root, "length_unit", "mm") != "mm") {
    throw std::invalid_argument("cable kinematics length_unit must be 'mm'");
  }

  const auto axes = require_map(root, "axes");
  reject_unknown_keys(
    axes, {"joint_01", "joint_02", "joint_04", "joint_05", "joint_06", "joint_09"},
    "axes");

  CableKinematicsParameters parameters;
  parameters.swap_wrist_channels = optional_value<bool>(root, "swap_wrist_channels", false);
  if (root["output_directions"]) {
    const auto directions = require_map(root, "output_directions");
    reject_unknown_keys(directions,
      {"motor_01", "motor_02", "motor_03", "motor_04", "motor_05", "motor_06"},
      "output_directions");
    for (std::size_t i = 0; i < parameters.output_directions.size(); ++i) {
      const auto name = "motor_0" + std::to_string(i + 1);
      parameters.output_directions[i] = require_value<int>(directions, name.c_str());
    }
  }
  parameters.axis1 = parse_linear(require_map(axes, "joint_01"));
  parameters.axis2 = parse_linear(require_map(axes, "joint_02"));
  parameters.axis3 = parse_coupled_axis3(require_map(axes, "joint_04"));
  parameters.axis4 = parse_coupled_chord(require_map(axes, "joint_05"));
  parameters.axis5 = parse_coupled_chord(require_map(axes, "joint_06"));

  const auto axis6 = require_map(axes, "joint_09");
  reject_unknown_keys(axis6, {"model", "branch_a", "branch_b"}, "joint_09");
  require_model(axis6, "dual_branch_coupled");
  parameters.axis6_branch_a = parse_axis6_branch(require_map(axis6, "branch_a"));
  parameters.axis6_branch_b = parse_axis6_branch(require_map(axis6, "branch_b"));

  const auto output_limits = require_map(root, "output_limits");
  reject_unknown_keys(
    output_limits,
    {"coordinate", "unit", "motor_01", "motor_02", "motor_03", "motor_04", "motor_05",
      "motor_06"},
    "output_limits");
  if (require_value<std::string>(output_limits, "coordinate") != "reducer_output_shaft") {
    throw std::invalid_argument(
            "output_limits coordinate must be 'reducer_output_shaft'");
  }
  if (require_value<std::string>(output_limits, "unit") != "rad") {
    throw std::invalid_argument("output_limits unit must be 'rad'");
  }
  for (std::size_t index = 0; index < parameters.output_limits.size(); ++index) {
    const std::string name = "motor_0" + std::to_string(index + 1U);
    parameters.output_limits[index] =
      parse_output_limit(require_map(output_limits, name.c_str()), name);
  }

  const auto validation = root["validation"];
  if (validation) {
    if (!validation.IsMap()) {
      throw std::invalid_argument("validation must be a map");
    }
    reject_unknown_keys(
      validation,
      {"asin_domain_tolerance", "cosine_singularity_threshold", "compatibility_tolerance",
        "output_limit_tolerance_rad"},
      "validation");
    parameters.asin_domain_tolerance =
      optional_value<double>(
      validation, "asin_domain_tolerance", parameters.asin_domain_tolerance);
    parameters.cosine_singularity_threshold =
      optional_value<double>(
      validation, "cosine_singularity_threshold",
      parameters.cosine_singularity_threshold);
    parameters.compatibility_tolerance =
      optional_value<double>(
      validation, "compatibility_tolerance", parameters.compatibility_tolerance);
    parameters.output_limit_tolerance_rad =
      optional_value<double>(
      validation, "output_limit_tolerance_rad", parameters.output_limit_tolerance_rad);
  }

  const CableKinematics validated(parameters);
  return validated.parameters();
}

}  // namespace pi_arm_transmission
