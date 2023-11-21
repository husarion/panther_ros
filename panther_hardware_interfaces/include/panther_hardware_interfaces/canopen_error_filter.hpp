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

#ifndef PANTHER_HARDWARE_INTERFACES_CANOPEN_ERROR_FILTER_HPP_
#define PANTHER_HARDWARE_INTERFACES_CANOPEN_ERROR_FILTER_HPP_

#include <atomic>

namespace panther_hardware_interfaces
{

/**
 * @brief Class that keeps track of different types of errors. In some rare cases Roboteq
 * controllers can miss for example the SDO response, or PDO can be received a bit later, which
 * results in a timeout. As they usually are rare and singular occurrences, it is better to filter
 * some of these errors, and escalate only when a certain number of errors happen.
 */
class CanOpenErrorFilter
{
public:
  CanOpenErrorFilter(
    unsigned max_write_sdo_errors_count, unsigned max_read_sdo_errors_count,
    unsigned max_read_pdo_errors_count)
  : max_write_sdo_errors_count_(max_write_sdo_errors_count),
    max_read_sdo_errors_count_(max_read_sdo_errors_count),
    max_read_pdo_errors_count_(max_read_pdo_errors_count)
  {
  }

  bool IsError() const { return error_; };

  bool IsWriteSDOError() const { return write_sdo_error_; };
  bool IsReadSDOError() const { return read_sdo_error_; };
  bool IsReadPDOError() const { return read_pdo_error_; };

  /**
   * @brief Update read SDO error count, if the number of consecutive errors exceeds the max
   * threshold error is set
   */
  void UpdateReadSDOError(bool current_error)
  {
    UpdateError(
      current_read_sdo_error_count_, read_sdo_error_, max_read_sdo_errors_count_, current_error);
  }

  /**
   * @brief Update read PDO error count, if the number of consecutive errors exceeds the max
   * threshold error is set
   */
  void UpdateReadPDOError(bool current_error)
  {
    UpdateError(
      current_read_pdo_error_count_, read_pdo_error_, max_read_pdo_errors_count_, current_error);
  }

  /**
   * @brief Update write SDO error count, if the number of consecutive errors exceeds the max
   * threshold error is set
   */
  void UpdateWriteSDOError(bool current_error)
  {
    UpdateError(
      current_write_sdo_error_count_, write_sdo_error_, max_write_sdo_errors_count_, current_error);
  }

  /**
   * @brief Sets clear errors flag - errors will be cleared upon the next Update (any) method.
   * This makes sure that the operation is multithread-safe.
   */
  void SetClearErrorsFlag() { clear_errors_.store(true); }

private:
  void UpdateError(
    unsigned & current_error_count, bool & error_flag, unsigned max_errors_count,
    bool current_error)
  {
    if (clear_errors_) {
      ClearErrors();
      clear_errors_.store(false);
    }

    if (current_error) {
      ++current_error_count;
      if (current_error_count >= max_errors_count) {
        error_ = true;
        error_flag = true;
      }
    } else {
      current_error_count = 0;
    }
  }

  void ClearErrors()
  {
    error_ = false;

    read_sdo_error_ = false;
    read_pdo_error_ = false;
    write_sdo_error_ = false;

    current_write_sdo_error_count_ = 0;
    current_read_sdo_error_count_ = 0;
    current_read_pdo_error_count_ = 0;
  };

  const unsigned max_write_sdo_errors_count_;
  unsigned current_write_sdo_error_count_ = 0;

  const unsigned max_read_sdo_errors_count_;
  unsigned current_read_sdo_error_count_ = 0;

  const unsigned max_read_pdo_errors_count_;
  unsigned current_read_pdo_error_count_ = 0;

  bool error_ = false;

  bool read_sdo_error_ = false;
  bool read_pdo_error_ = false;
  bool write_sdo_error_ = false;

  std::atomic_bool clear_errors_ = false;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_CANOPEN_ERROR_FILTER_HPP_
