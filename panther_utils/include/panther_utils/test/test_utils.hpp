#ifndef PANTHER_UTILS_TEST_UTILS_HPP_
#define PANTHER_UTILS_TEST_UTILS_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

namespace panther_utils::test_utils
{

template <typename NodeT, typename MsgT>
bool WaitForMsg(
  const std::shared_ptr<NodeT> & node, std::shared_ptr<MsgT> & msg,
  const std::chrono::nanoseconds & timeout)
{
  msg = nullptr;
  rclcpp::Time start_time = node->now();

  while (rclcpp::ok() && node->now() - start_time <= rclcpp::Duration(timeout)) {
    if (msg) {
      return true;
    }
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

template <typename T>
bool CheckNaNVector(const std::vector<T> & vector)
{
  if (!std::numeric_limits<T>::has_quiet_NaN) {
    throw std::runtime_error(
      "Invalid method typename. Valid are: 'float', 'double', 'long double'.");
  }
  return std::all_of(vector.begin(), vector.end(), [](const T value) { return std::isnan(value); });
}

}  // namespace panther_utils::test_utils

#endif  // PANTHER_UTILS_TEST_UTILS_HPP_
