#ifndef PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_ERROR_HANDLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_ERROR_HANDLER_HPP_

#include <atomic>

namespace panther_hardware_interfaces
{
class PantherSystemErrorHandler
{
public:
  PantherSystemErrorHandler(
    unsigned max_write_sdo_errors_count, unsigned max_read_sdo_errors_count,
    unsigned max_read_pdo_errors_count)
  : max_write_sdo_errors_count_(max_write_sdo_errors_count),
    max_read_sdo_errors_count_(max_read_sdo_errors_count),
    max_read_pdo_errors_count_(max_read_pdo_errors_count)
  {
  }

  bool IsError() { return error_; };

  bool IsWriteSDOError() { return write_sdo_error_; };
  bool IsReadSDOError() { return read_sdo_error_; };
  bool IsReadPDOError() { return read_pdo_error_; };

  void UpdateReadSDOErrors(bool current_error)
  {
    UpdateError(
      current_read_sdo_error_count_, read_sdo_error_, max_read_sdo_errors_count_, current_error);
  }

  void UpdateReadPDOErrors(bool current_error)
  {
    UpdateError(
      current_read_pdo_error_count_, read_pdo_error_, max_read_pdo_errors_count_, current_error);
  }

  void UpdateWriteSDOErrors(bool current_error)
  {
    UpdateError(
      current_write_sdo_error_count_, write_sdo_error_, max_write_sdo_errors_count_, current_error);
  }

  // Not clearing errors here to make it multithread safe
  void SetClearErrorFlag() { clear_errors_.store(true); }

private:
  void UpdateError(
    unsigned & current_error_count, bool & error_flag, unsigned max_errors_count,
    bool current_error)
  {
    if (clear_errors_) {
      ClearErrors();
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

  // Sometimes there's a single SDO write error, which is better to filter out
  // If more consecutive errors happen, action should be taken
  const unsigned max_write_sdo_errors_count_;
  unsigned current_write_sdo_error_count_ = 0;

  const unsigned max_read_sdo_errors_count_;
  unsigned current_read_sdo_error_count_ = 0;

  // PDO errors doesn't happen so it should be set to 1
  const unsigned max_read_pdo_errors_count_;
  unsigned current_read_pdo_error_count_ = 0;

  bool error_ = false;

  bool read_sdo_error_ = false;
  bool read_pdo_error_ = false;
  bool write_sdo_error_ = false;

  std::atomic_bool clear_errors_ = false;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_ERROR_HANDLER_HPP_