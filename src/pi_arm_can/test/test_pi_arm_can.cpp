#include "pi_arm_can/can_driver.hpp"
#include "pi_arm_can/protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace
{

TEST(Protocol, EncodesPositionControlPayloadLittleEndian)
{
  const auto frame = pi_arm_can::make_position_command(5U, -123.45, 321.0);
  EXPECT_EQ(frame.id, 0x145U);
  const std::array<std::uint8_t, 8> expected{
    0xA4, 0x00, 0x41, 0x01, 0xC7, 0xCF, 0xFF, 0xFF};
  EXPECT_EQ(frame.data, expected);
}

TEST(Protocol, DecodesSigned56BitLimits)
{
  const std::array<std::uint8_t, 7> minus_one{
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  const std::array<std::uint8_t, 7> minimum{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80};
  EXPECT_EQ(pi_arm_can::decode_signed_56_le(minus_one.data(), minus_one.size()), -1);
  EXPECT_EQ(
    pi_arm_can::decode_signed_56_le(minimum.data(), minimum.size()),
    -(std::int64_t{1} << 55));
}

TEST(Protocol, NeverRaisesRequestedSpeedLimit)
{
  const auto frame = pi_arm_can::make_position_command(4U, 0.6, 1.8);
  EXPECT_EQ(pi_arm_can::read_le<std::uint16_t>(&frame.data[2]), 1U);
  EXPECT_THROW(pi_arm_can::make_position_command(4U, 0.6, 0.3), std::out_of_range);
  EXPECT_THROW(pi_arm_can::make_position_command(1U, 1e300, 1.0), std::out_of_range);
  EXPECT_THROW(pi_arm_can::make_position_command(1U, 0.0, 1e300), std::out_of_range);
}

TEST(Protocol, RejectsPositionCommandOutsideProtocolRanges)
{
  EXPECT_THROW(
    pi_arm_can::make_position_command(1U, 0.0, 0.0),
    std::out_of_range);
  EXPECT_THROW(
    pi_arm_can::make_position_command(1U, 0.0, 65536.0),
    std::out_of_range);
  EXPECT_THROW(
    pi_arm_can::make_position_command(
      1U, static_cast<double>(std::numeric_limits<std::int32_t>::max()) / 100.0 + 1.0,
      1.0),
    std::out_of_range);
}

TEST(DriverCache, ParsesStatusAndAngleFrames)
{
  pi_arm_can::DriverConfig config;
  config.motor_ids = {1U};
  pi_arm_can::CanDriver driver(config);

  pi_arm_can::CanFrame status;
  status.id = 0x141U;
  status.data = {0x9A, 0xFB, 0xE8, 0x03, 0x0C, 0xFE, 0x30, 0x02};
  driver.process_received_frame(status);
  auto state = driver.motor_state(1U);
  EXPECT_TRUE(state.status1_valid);
  EXPECT_EQ(state.temperature_c, -5);
  EXPECT_DOUBLE_EQ(state.voltage_v, 10.0);
  EXPECT_DOUBLE_EQ(state.current_a, -5.0);
  EXPECT_EQ(state.motor_state, 0x30);
  EXPECT_EQ(state.error_state, 0x02);

  pi_arm_can::CanFrame speed;
  speed.id = 0x141U;
  speed.data = {0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  driver.process_received_frame(speed);
  driver.process_received_frame(speed);
  driver.process_received_frame(speed);
  state = driver.motor_state(1U);
  EXPECT_EQ(state.stationary_samples, 3U);

  pi_arm_can::CanFrame angle;
  angle.id = 0x141U;
  angle.data = {0x92, 0xC7, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  driver.process_received_frame(angle);
  state = driver.motor_state(1U);
  EXPECT_TRUE(state.angle_valid);
  EXPECT_DOUBLE_EQ(state.protocol_angle_deg, -123.45);
}

TEST(DriverCache, RecordsSetZeroAcknowledgementWithoutChangingLiveAngle)
{
  pi_arm_can::DriverConfig config;
  config.motor_ids = {1U};
  pi_arm_can::CanDriver driver(config);

  pi_arm_can::CanFrame angle;
  angle.id = 0x141U;
  angle.data = {0x92, 0x39, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00};
  driver.process_received_frame(angle);
  auto state = driver.motor_state(1U);
  ASSERT_DOUBLE_EQ(state.protocol_angle_deg, 123.45);
  ASSERT_FALSE(state.zero_ack_valid);

  pi_arm_can::CanFrame acknowledgement;
  acknowledgement.id = 0x141U;
  acknowledgement.data = {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  driver.process_received_frame(acknowledgement);
  state = driver.motor_state(1U);
  EXPECT_TRUE(state.zero_ack_valid);
  EXPECT_DOUBLE_EQ(state.protocol_angle_deg, 123.45);
}

TEST(DriverConfiguration, RejectsUnsafeInterfaceNames)
{
  pi_arm_can::DriverConfig config;
  config.interface_name = "../can0";
  config.motor_ids = {1U};
  EXPECT_THROW(pi_arm_can::CanDriver driver(config), std::invalid_argument);
}

TEST(DriverSafety, ReadOnlyModeRejectsControlAndManagement)
{
  pi_arm_can::DriverConfig config;
  config.interface_name = "vcan0";
  config.motor_ids = {1U};
  config.read_only = true;
  pi_arm_can::CanDriver driver(config);

  EXPECT_FALSE(driver.set_mode(pi_arm_can::DriverMode::CONTROL));
  EXPECT_FALSE(
    driver.execute_management(
      pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO, {1U}));
}

}  // namespace
