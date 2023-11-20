#include <panther_hardware_interfaces/utils.hpp>

#include <iostream>
#include <stdexcept>

namespace panther_hardware_interfaces
{

uint8_t GetByte(uint32_t data, uint8_t byte_no)
{
  if (byte_no > 3) {
    throw std::runtime_error("byte_no out of range, allowed values: [0;3]");
  }

  return (data >> (byte_no * 8)) & 0xFF;
}

bool IsBitSet(uint8_t flags, uint8_t bit_no)
{
  if (bit_no > 7) {
    throw std::runtime_error("bit_no out of range, allowed values: [0;7]");
  }

  return flags & (0b00000001 << bit_no);
}

bool OperationWithAttempts(
  std::function<void()> operation, unsigned max_attempts, std::function<void()> on_error)
{
  for (unsigned attempts_counter = 0; attempts_counter < max_attempts; ++attempts_counter) {
    try {
      operation();
      return true;
    } catch (std::runtime_error & err) {
      std::cerr << "Operation failed: " << err.what() << ". Attempt " << attempts_counter + 1
                << " of " << max_attempts << std::endl;
      try {
        on_error();
      } catch (std::runtime_error & on_error_err) {
        std::cerr << "on_error function failed: " << on_error_err.what() << std::endl;
        return false;
      }
    }
  }
  return false;
}

}  // namespace panther_hardware_interfaces