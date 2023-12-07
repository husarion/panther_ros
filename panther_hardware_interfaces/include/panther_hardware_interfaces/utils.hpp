// Copyright 2023 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PANTHER_HARDWARE_INTERFACES_UTILS_HPP_
#define PANTHER_HARDWARE_INTERFACES_UTILS_HPP_

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
bool IsBitSet(uint8_t data, uint8_t bit_no);

/**
 * @brief Set bit_no (bit_no has to be in [0;7] range)
 * @exception std::runtime_error if bit_no is out of range
 */
uint8_t SetBit(uint8_t data, uint8_t bit_no);

/**
 * @brief Attempts to run operation for max_attempts number of times.
 * operation can throw std::runtime_error, which is caught, and on_error function
 * is executed (for example deinitialization or some other cleanup in case of
 * failure)
 * @return true if the operation was successfully executed, false if it wasn't successfully executed
 * and the number of attempts exceeded the maximum allowed or on_error function threw
 * std::runtime_error
 */
bool OperationWithAttempts(
  const std::function<void()> operation, const unsigned max_attempts,
  const std::function<void()> on_error);

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_UTILS_HPP_
