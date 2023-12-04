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

bool IsBitSet(uint8_t data, uint8_t bit_no)
{
  if (bit_no > 7) {
    throw std::runtime_error("bit_no out of range, allowed values: [0;7]");
  }

  return data & (0b00000001 << bit_no);
}

uint8_t SetBit(uint8_t data, uint8_t bit_no)
{
  if (bit_no > 7) {
    throw std::runtime_error("bit_no out of range, allowed values: [0;7]");
  }

  return data | (0b00000001 << bit_no);
}

bool OperationWithAttempts(
  std::function<void()> operation, unsigned max_attempts, std::function<void()> on_error)
{
  for (unsigned attempts_counter = 0; attempts_counter < max_attempts; ++attempts_counter) {
    try {
      operation();
      return true;
    } catch (const std::runtime_error & e) {
      std::cerr << "Operation failed: " << e.what() << ". Attempt " << attempts_counter + 1
                << " of " << max_attempts << std::endl;
      try {
        on_error();
      } catch (const std::runtime_error & on_error_e) {
        std::cerr << "on_error function failed: " << on_error_e.what() << std::endl;
        return false;
      }
    }
  }
  return false;
}

}  // namespace panther_hardware_interfaces
