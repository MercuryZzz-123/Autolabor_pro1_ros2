#include <array>

#include <gtest/gtest.h>

#include "autolabor_driver/chassis_protocol.hpp"

namespace
{

TEST(ChassisProtocolTest, XorChecksumEmptyBufferIsZero)
{
  EXPECT_EQ(autolabor_driver::protocol::xor_checksum(nullptr, 0), 0x00);
}

TEST(ChassisProtocolTest, XorChecksumMatchesChassisFrame)
{
  const std::array<uint8_t, 6> request = {0x55, 0xAA, 0x02, 0x01, 0x07, 0x00};

  EXPECT_EQ(autolabor_driver::protocol::xor_checksum(request.data(), request.size()), 0xFB);
}

TEST(ChassisProtocolTest, EncoderDeltaWithoutOverflow)
{
  EXPECT_EQ(autolabor_driver::protocol::encoder_delta(100, 140), 40);
  EXPECT_EQ(autolabor_driver::protocol::encoder_delta(140, 100), -40);
  EXPECT_EQ(autolabor_driver::protocol::encoder_delta(1234, 1234), 0);
}

TEST(ChassisProtocolTest, EncoderDeltaWithForwardOverflow)
{
  EXPECT_EQ(autolabor_driver::protocol::encoder_delta(65530, 5), 10);
}

TEST(ChassisProtocolTest, EncoderDeltaWithReverseOverflow)
{
  EXPECT_EQ(autolabor_driver::protocol::encoder_delta(5, 65530), -10);
}

TEST(ChassisProtocolTest, UpdateEncoderPulseStoresReceivedValue)
{
  int current = 65530;
  int delta = 0;

  autolabor_driver::protocol::update_encoder_pulse(current, 5, delta);

  EXPECT_EQ(delta, 10);
  EXPECT_EQ(current, 5);
}

}  // namespace
