#ifndef PANTHER_HARDWARE_INTERFACES__UTILS_HPP_
#define PANTHER_HARDWARE_INTERFACES__UTILS_HPP_

namespace panther_hardware_interfaces
{

inline uint8_t GetByte(uint32_t data, uint8_t byte_no) { return (data >> (byte_no * 8)) & 0xFF; }
inline bool BitSet(uint8_t flags, uint8_t bit_no) { return flags & (0b00000001 << bit_no); }

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__UTILS_HPP_