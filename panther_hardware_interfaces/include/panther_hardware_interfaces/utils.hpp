#ifndef PANTHER_HARDWARE_INTERFACES__UTILS_HPP_
#define PANTHER_HARDWARE_INTERFACES__UTILS_HPP_

#include <cstdint>
#include <functional>

namespace panther_hardware_interfaces
{

/**
 * @brief Get byte byte_no from data (byte_no has to be in [0;3] range)
 * @exception std::runtime_error if byte_no is out of range
 */
uint8_t GetByte(uint32_t data, uint8_t byte_no);

/**
 * @brief Check if bit bit_no is set (bit_no has to be in [0;7] range)
 * @exception std::runtime_error if bit_no is out of range
 */
bool IsBitSet(uint8_t flags, uint8_t bit_no);

/**
 * @brief Attempts to run operation for max_attempts number of times.
 * operation can throw std::runtime_error, which is caught, and on_error function
 * is executed (for example deinitialization or some other clean up in case of 
 * failure)
 * @return true if operation was successfully executed, false if it wasn't successfuly executed
 * and number of attempts exceeded maximum allowed or on_error function threw std::runtime_error
 */
bool OperationWithAttempts(
  std::function<void()> operation, unsigned max_attempts, std::function<void()> on_error);

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__UTILS_HPP_