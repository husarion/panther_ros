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

#ifndef PANTHER_UTILS_COMMON_UTILITIES_HPP_
#define PANTHER_UTILS_COMMON_UTILITIES_HPP_

#include <fstream>
#include <map>
#include <string>

namespace panther_utils::common_utilities
{

/**
 * @brief Prefixes the keys of a given map with a specified prefix.
 *
 * @tparam T The value type of the map.
 * @param map The input map. Key of the map must be std::string type.
 * @param prefix The prefix to be added to each key.
 * @return A new map with the keys prefixed.
 */
template <typename T>
std::map<std::string, T> PrefixMapKeys(
  const std::map<std::string, T> & map, const std::string & prefix)
{
  std::map<std::string, T> prefixed_map;
  for (const auto & [key, value] : map) {
    prefixed_map[prefix + key] = value;
  }
  return prefixed_map;
}

/**
 * @brief Opens a file with the specified mode.
 *
 * This function opens a file with the specified mode and returns a std::fstream object.
 * If the file fails to open, it throws a std::runtime_error.
 *
 * @param file_path The path to the file to be opened.
 * @param mode The mode in which the file should be opened (e.g., std::ios_base::in |
 * std::ios_base::out).
 * @return std::fstream The opened file stream.
 *
 * @throws std::runtime_error if the file fails to open.
 */
std::fstream OpenFile(const std::string & file_path, const std::ios_base::openmode & mode)
{
  std::fstream file(file_path, mode);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + file_path);
  }
  return file;
}

}  // namespace panther_utils::common_utilities

#endif  // PANTHER_UTILS_COMMON_UTILITIES_HPP_
