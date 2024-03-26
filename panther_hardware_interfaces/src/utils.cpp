// Copyright 2024 Husarion sp. z o.o.
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

#include "panther_hardware_interfaces/utils.hpp"

#include <iostream>
#include <stdexcept>

namespace panther_hardware_interfaces
{

bool OperationWithAttempts(
  const std::function<void()> operation, const unsigned max_attempts,
  const std::function<void()> on_error)
{
  for (unsigned i = 0; i < max_attempts; ++i) {
    try {
      operation();
      return true;
    } catch (const std::runtime_error & e) {
      std::cerr << "Operation failed: " << e.what() << ". Attempt " << i + 1 << " of "
                << max_attempts << std::endl;
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

bool CheckIfJointNameContainValidSequence(const std::string & name, const std::string & sequence)
{
  const std::size_t pos = name.find(sequence);
  if (pos == std::string::npos) {
    return false;
  }

  if (pos >= 1) {
    const std::size_t id_before_sequence = pos - 1;
    if (name[id_before_sequence] != '_' && name[id_before_sequence] != '/') {
      return false;
    }
  }

  const std::size_t id_after_sequence = pos + sequence.length();
  if (id_after_sequence < name.length() && name[id_after_sequence] != '_') {
    return false;
  }

  return true;
}

}  // namespace panther_hardware_interfaces
