#pragma once

#include <cstddef>

namespace pi_arm_transmission
{

enum class KinematicsError
{
  NONE,
  NON_FINITE_INPUT,
  ASIN_DOMAIN,
  VELOCITY_SINGULARITY,
};

struct KinematicsStatus
{
  KinematicsError error{KinematicsError::NONE};
  std::size_t axis{0U};
  double value{0.0};

  constexpr bool ok() const noexcept {return error == KinematicsError::NONE;}
  constexpr explicit operator bool() const noexcept {return ok();}

  static constexpr KinematicsStatus success() noexcept {return {};}
};

const char * to_string(KinematicsError error) noexcept;

}  // namespace pi_arm_transmission
