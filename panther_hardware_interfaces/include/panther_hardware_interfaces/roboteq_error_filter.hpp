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
  void UpdateError(const bool current_error)
  {
    if (current_error) {
      ++current_error_count_;
      if (current_error_count_ >= max_error_count_) {
        error_ = true;
      }
    } else {
      current_error_count_ = 0;
    }
  }

  void ClearError()
  {
    error_ = false;
    current_error_count_ = 0;
  }

private:
  const unsigned max_error_count_;
  unsigned current_error_count_ = 0;
  bool error_ = false;
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
  RoboteqErrorFilter(const std::vector<ErrorFilter> & error_filters) : error_filters_(error_filters)
  {
  }

  bool IsError() const
  {
    return std::any_of(error_filters_.begin(), error_filters_.end(), [](const auto & filter) {
      return filter.IsError();
    });
  };

  bool IsError(const std::size_t id) const { return error_filters_[id].IsError(); };

  /**
   * @brief Updates error count, if the number of consecutive errors exceeds the max
   * threshold error is set
   */
  void UpdateError(const std::size_t id, const bool current_error)
  {
    ClearErrorsIfFlagSet();
    error_filters_[id].UpdateError(current_error);
  }

  /**
   * @brief Sets clear errors flag - errors will be cleared upon the next Update (any) method.
   * This makes sure that the operation is multithread-safe.
   */
  void SetClearErrorsFlag() { clear_errors_.store(true); }

private:
  void ClearErrorsIfFlagSet()
  {
    if (clear_errors_) {
      std::for_each(
        error_filters_.begin(), error_filters_.end(), [](auto & filter) { filter.ClearError(); });
      clear_errors_.store(false);
    }
  }

  std::atomic_bool clear_errors_ = false;

  std::vector<ErrorFilter> error_filters_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_ROBOTEQ_ERROR_FILTER_HPP_
