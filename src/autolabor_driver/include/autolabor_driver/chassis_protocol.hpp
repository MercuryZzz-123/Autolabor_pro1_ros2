#ifndef AUTOLABOR_DRIVER__CHASSIS_PROTOCOL_HPP_
#define AUTOLABOR_DRIVER__CHASSIS_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>

namespace autolabor_driver
{
namespace protocol
{

inline uint8_t xor_checksum(const uint8_t * data, std::size_t len)
{
  uint8_t checksum = 0x00;
  for (std::size_t i = 0; i < len; ++i) {
    checksum = checksum ^ *(data + i);
  }
  return checksum;
}

inline int encoder_delta(int current, int receive)
{
  if (receive > current) {
    return (receive - current) < (current - receive + 65535) ?
           (receive - current) : (receive - current - 65535);
  }
  return (current - receive) < (receive - current + 65535) ?
         (receive - current) : (receive - current + 65535);
}

inline void update_encoder_pulse(int & current, int receive, int & delta)
{
  delta = encoder_delta(current, receive);
  current = receive;
}

}  // namespace protocol
}  // namespace autolabor_driver

#endif  // AUTOLABOR_DRIVER__CHASSIS_PROTOCOL_HPP_
