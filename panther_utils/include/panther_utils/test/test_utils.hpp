#ifndef PANTHER_UTILS_TEST_UTILS_HPP_
#define PANTHER_UTILS_TEST_UTILS_HPP_

#include <chrono>
#include <memory>

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

}  // namespace panther_utils::test_utils

#endif  // PANTHER_UTILS_TEST_UTILS_HPP_