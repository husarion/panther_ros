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

#ifndef PANTHER_DIAGNOSTICS__TYPES_HPP_
#define PANTHER_DIAGNOSTICS__TYPES_HPP_

#include <vector>

namespace panther_diagnostics
{

/**
 * @brief Represents the system status information.
 *
 * This struct contains various metrics related to the system status, such as CPU usage, CPU
 * temperature, memory usage, and disk usage.
 */
struct SystemStatus
{
  std::vector<float> core_usages;
  float cpu_mean_usage;
  float cpu_temperature;
  float memory_usage;
  float disk_usage;
};

}  // namespace panther_diagnostics

#endif  // PANTHER_DIAGNOSTICS__TYPES_HPP_
