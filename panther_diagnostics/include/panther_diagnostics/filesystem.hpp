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

#ifndef PANTHER_DIAGNOSTICS__FILESYSTEM_HPP_
#define PANTHER_DIAGNOSTICS__FILESYSTEM_HPP_

#include <filesystem>

namespace panther_diagnostics
{

/**
 * @brief Interface for interacting with the filesystem.
 *
 * This interface provides a way to interact with the filesystem, allowing users to retrieve space
 * information for a given path.
 */
class FilesystemInterface
{
public:
  /**
   * @brief Virtual destructor for the FilesystemInterface class.
   */
  virtual ~FilesystemInterface() = default;

  /**
   * @brief Retrieves space information for a given path.
   *
   * @param path The path for which to retrieve space information.
   * @return std::filesystem::space_info The space information for the given path.
   */
  virtual std::filesystem::space_info GetSpaceInfo(const std::filesystem::path & path) const = 0;

  /**
   * @brief Alias for a shared pointer to a FilesystemInterface object.
   *
   */
  using SharedPtr = std::shared_ptr<FilesystemInterface>;
};

/**
 * @brief The Filesystem class provides functionality related to filesystem operations.
 *
 * This class inherits from the FilesystemInterface and provides an implementation for
 * getting space information for a given path.
 */
class Filesystem : public FilesystemInterface
{
public:
  /**
   * @brief Get the space information for a given path.
   *
   * This function returns the space information (capacity, free space, and available space)
   * for the specified path.
   *
   * @param path The path for which to retrieve the space information.
   * @return The space information for the specified path.
   */
  inline std::filesystem::space_info GetSpaceInfo(const std::filesystem::path & path) const override
  {
    return std::filesystem::space(path);
  }
};

}  // namespace panther_diagnostics

#endif  // PANTHER_DIAGNOSTICS__FILESYSTEM_HPP_
