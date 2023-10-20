#ifndef PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_ERROR_HANDLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_ERROR_HANDLER_HPP_

namespace panther_hardware_interfaces
{
class PantherSystemErrorHandler
{
public:
  PantherSystemErrorHandler(
    int8_t max_write_sdo_errors_count, int8_t max_read_sdo_errors_count,
    int8_t max_read_pdo_errors_count)
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

private:
  void UpdateError(
    int8_t & current_error_count, bool & error_flag, int8_t max_errors_count, bool current_error)
  {
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

  // Sometimes there's a single SDO write error, which is better to filter out
  // If more consecutive errors happen, action should be taken
  const int8_t max_write_sdo_errors_count_;
  int8_t current_write_sdo_error_count_ = 0;

  const int8_t max_read_sdo_errors_count_;
  int8_t current_read_sdo_error_count_ = 0;

  // PDO errors doesn't happen so it should be set to 1
  const int8_t max_read_pdo_errors_count_;
  int8_t current_read_pdo_error_count_ = 0;

  bool error_ = false;

  bool read_sdo_error_ = false;
  bool read_pdo_error_ = false;
  bool write_sdo_error_ = false;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_ERROR_HANDLER_HPP_