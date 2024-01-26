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

#ifndef PANTHER_HARDWARE_INTERFACES_ROBOTEQ_ERROR_FILTER_HPP_
#define PANTHER_HARDWARE_INTERFACES_ROBOTEQ_ERROR_FILTER_HPP_

#include <atomic>
#include <map>

namespace panther_hardware_interfaces
{

class ErrorFilter
{
public:
  ErrorFilter(const unsigned max_error_count) : max_error_count_(max_error_count) {}

  bool IsError() const { return error_; };

  /**
   * @brief Updates the error count, if the number of consecutive errors exceeds the max
   * threshold error is set
   */
  void UpdateError(const bool current_error);

  void ClearError();

private:
  const unsigned max_error_count_;
  unsigned current_error_count_ = 0;
  bool error_ = false;
};

enum class ErrorsFilterIds {
  READ_SDO = 0,
  WRITE_SDO,
  READ_PDO,
  ROBOTEQ_DRIVER,
};

/**
 * @brief Class that keeps track of different types of errors. In some rare cases Roboteq
 * controllers can miss for example the SDO response, or PDO can be received a bit later, which
 * results in a timeout. As they usually are rare and singular occurrences, it is better to filter
 * some of these errors, and escalate only when a certain number of errors happen.
 */
class RoboteqErrorFilter
{
public:
  RoboteqErrorFilter(
    const unsigned max_read_sdo_errors_count, const unsigned max_write_sdo_errors_count,
    const unsigned max_read_pdo_errors_count, const unsigned max_roboteq_driver_error_count);

  bool IsError() const;

  bool IsError(const ErrorsFilterIds id) const { return error_filters_.at(id).IsError(); };

  /**
   * @brief Updates error count, if the number of consecutive errors exceeds the max
   * threshold error is set
   */
  void UpdateError(const ErrorsFilterIds id, const bool current_error);
  /**
   * @brief Sets clear errors flag - errors will be cleared upon the next Update (any) method.
   * This makes sure that the operation is multithread-safe.
   */
  void SetClearErrorsFlag() { clear_errors_.store(true); }

private:
  void ClearErrorsIfFlagSet();

  std::atomic_bool clear_errors_ = false;

  std::map<ErrorsFilterIds, ErrorFilter> error_filters_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_ROBOTEQ_ERROR_FILTER_HPP_
