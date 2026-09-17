#include "pi_arm_can/can_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

struct Options
{
  std::string interface_name{"can0"};
  std::string audit_file{"pi_arm_rom_zero_audit.jsonl"};
  bool execute{false};
  bool yes{false};
};

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--interface" && index + 1 < argc) {
      options.interface_name = argv[++index];
    } else if (argument == "--audit-file" && index + 1 < argc) {
      options.audit_file = argv[++index];
    } else if (argument == "--execute") {
      options.execute = true;
    } else if (argument == "--yes") {
      options.yes = true;
    } else if (argument == "--help") {
      std::cout
        << "Usage: pi_arm_rom_zero_cli [--interface can0] [--audit-file PATH] "
        << "[--execute] [--yes]\n\n"
        << "Default is dry-run. --yes is accepted only for vcan interfaces.\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown or incomplete argument: " + argument);
    }
  }
  if (options.audit_file.empty()) {
    throw std::invalid_argument("--audit-file must not be empty");
  }
  return options;
}

std::string utc_now()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t value = std::chrono::system_clock::to_time_t(now);
  std::tm result{};
  gmtime_r(&value, &result);
  std::ostringstream text;
  text << std::put_time(&result, "%Y-%m-%dT%H:%M:%SZ");
  return text.str();
}

std::string json_escape(const std::string & value)
{
  std::string result;
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      result.push_back('\\');
    }
    result.push_back(character);
  }
  return result;
}

void append_audit(
  const Options & options, const std::string & result, const std::string & message)
{
  std::ofstream stream(options.audit_file, std::ios::app);
  if (!stream) {
    throw std::runtime_error("cannot append audit file: " + options.audit_file);
  }
  stream << "{\"timestamp_utc\":\"" << utc_now()
         << "\",\"interface\":\"" << json_escape(options.interface_name)
         << "\",\"execute\":" << (options.execute ? "true" : "false")
         << ",\"result\":\"" << json_escape(result)
         << "\",\"message\":\"" << json_escape(message) << "\"}\n";
}

bool is_vcan(const std::string & interface_name)
{
  return interface_name.rfind("vcan", 0U) == 0U;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  try {
    options = parse_options(argc, argv);
    if (options.yes && !is_vcan(options.interface_name)) {
      throw std::invalid_argument("--yes is forbidden on a non-vcan interface");
    }

    pi_arm_can::DriverConfig config;
    config.interface_name = options.interface_name;
    config.motor_ids = {1U, 2U, 3U, 4U, 5U, 6U};
    config.state_poll_interval = std::chrono::milliseconds{50};
    config.status1_poll_divider = 2U;
    pi_arm_can::CanDriver driver(config);
    driver.start();

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{8};
    bool healthy = false;
    std::string failure = "timed out waiting for six healthy stationary motors";
    while (std::chrono::steady_clock::now() < deadline) {
      healthy = true;
      const auto snapshot = driver.snapshot();
      const auto now = std::chrono::steady_clock::now();
      for (const auto id : config.motor_ids) {
        const auto & state = snapshot.at(id);
        const auto age = std::chrono::duration<double>(now - state.last_update).count();
        if (!state.angle_valid || !state.status1_valid || !state.status2_valid || age > 1.0) {
          healthy = false;
          failure = "motor feedback is missing or stale";
          break;
        }
        if (state.error_state != 0U) {
          healthy = false;
          failure = "a motor reports non-zero error flags";
          break;
        }
        if (state.protocol_speed_dps != 0.0 || state.stationary_samples < 3U) {
          healthy = false;
          failure = "all motors must report zero speed for three consecutive samples";
          break;
        }
      }
      if (healthy) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{50});
    }
    if (!healthy) {
      driver.stop();
      append_audit(options, "rejected", failure);
      std::cerr << "Safety checks failed: " << failure << '\n';
      return 2;
    }

    if (!options.execute) {
      driver.stop();
      append_audit(options, "dry-run-ok", "six motors healthy and stationary; no 0x19 sent");
      std::cout << "Dry-run passed. No ROM write command was sent.\n";
      return 0;
    }

    if (!options.yes) {
      std::cout
        << "DANGER: this writes the current position to ROM on CAN IDs 1..6.\n"
        << "Type exactly WRITE ROM ZERO to continue: " << std::flush;
      std::string confirmation;
      std::getline(std::cin, confirmation);
      if (confirmation != "WRITE ROM ZERO") {
        driver.stop();
        append_audit(options, "cancelled", "interactive confirmation did not match");
        std::cerr << "Cancelled. No ROM write command was sent.\n";
        return 3;
      }
    }

    const bool success = driver.execute_management(
      pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO,
      config.motor_ids, std::chrono::seconds{3});
    const std::string detail = success ?
      "six 0x19 acknowledgements received; power-cycle is required" :
      (driver.last_error().empty() ? "ROM write was not confirmed" : driver.last_error());
    driver.stop();
    append_audit(options, success ? "executed" : "failed", detail);
    if (!success) {
      std::cerr << detail << '\n';
      return 4;
    }
    std::cout << detail << '\n';
    return 0;
  } catch (const std::exception & error) {
    try {
      append_audit(options, "error", error.what());
    } catch (...) {
    }
    std::cerr << "pi_arm_rom_zero_cli: " << error.what() << '\n';
    return 1;
  }
}
