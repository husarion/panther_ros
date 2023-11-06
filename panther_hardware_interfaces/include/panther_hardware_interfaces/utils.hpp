#ifndef PANTHER_HARDWARE_INTERFACES__UTILS_HPP_
#define PANTHER_HARDWARE_INTERFACES__UTILS_HPP_

namespace panther_hardware_interfaces
{

/**
 * @brief Get byte byte_no from data (byte_no has to be in [0;3] range)
 * @exception std::runtime_error if byte_no is out of range
 */
inline uint8_t GetByte(uint32_t data, uint8_t byte_no)
{
  if (byte_no > 3) {
    throw std::runtime_error("byte_no out of range, allowed values: [0;3]");
  }

  return (data >> (byte_no * 8)) & 0xFF;
}

/**
 * @brief Check if bit bit_no is set (bit_no has to be in [0;7] range)
 * @exception std::runtime_error if bit_no is out of range
 */
inline bool IsBitSet(uint8_t flags, uint8_t bit_no)
{
  if (bit_no > 7) {
    throw std::runtime_error("bit_no out of range, allowed values: [0;7]");
  }

  return flags & (0b00000001 << bit_no);
}

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__UTILS_HPP_