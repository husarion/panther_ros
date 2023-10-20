#ifndef PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_ERROR_HANDLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_ERROR_HANDLER_HPP_

namespace panther_hardware_interfaces
{
class PantherSystemErrorHandler
{
public:
  PantherSystemErrorHandler() {}

  bool IsError() { return error_; };
  bool IsWriteError() { return write_error_; };
  bool IsSDOReadError() { return read_sdo_error_; };
  bool IsPDOReadError() { return read_pdo_error_; };

  void UpdateSDOReadErrors(bool current_error)
  {
    if (current_error) {
      ++current_read_sdo_error_count_;
      if (current_read_sdo_error_count_ >= max_read_sdo_errors_count_) {
        error_ = true;
        read_sdo_error_ = true;
        // RCLCPP_ERROR_STREAM(
        //   rclcpp::get_logger("PantherSystem"),
        //   "Read error count exceeded max value, entering error state");
      }
    } else {
      current_read_sdo_error_count_ = 0;
    }
  }

  void UpdateReadPDOErrors(bool current_error)
  {
    if (current_error) {
      // it is necessary to set error immiediately - can be caused by disconnected encoder
      error_ = true;
      read_pdo_error_ = true;
    }
  }

  void UpdateWriteErrors(bool current_error)
  {
    if (current_error) {
      ++current_write_error_count_;
      if (current_write_error_count_ >= max_write_errors_count_) {
        error_ = true;
        write_error_ = true;
        // RCLCPP_ERROR_STREAM(
        //   rclcpp::get_logger("PantherSystem"),
        //   "Error count exceeded max value, entering error state");
      }
    } else {
      current_write_error_count_ = 0;
    }
  }

  // TODO add service for clearing errors
  void ClearErrors()
  {
    error_ = false;
    read_sdo_error_ = false;
    read_pdo_error_ = false;
    write_error_ = false;

    // todo current_write_error_count_
  };

private:
  // TODO: parameter

  // Sometimes there's a single SDO write error, which is better to filter out
  // If more consecutive errors happen, action should be taken
  const int8_t max_write_errors_count_ = 2;
  int8_t current_write_error_count_ = 0;

  const int8_t max_read_sdo_errors_count_ = 2;
  int8_t current_read_sdo_error_count_ = 0;

  bool error_ = false;
  bool read_sdo_error_ = false;
  bool read_pdo_error_ = false;
  bool write_error_ = false;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_ERROR_HANDLER_HPP_