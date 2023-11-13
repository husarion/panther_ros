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

#ifndef PANTHER_UTILS_ROS_SYNC_CLIENT_HPP_
#define PANTHER_UTILS_ROS_SYNC_CLIENT_HPP_

#include <chrono>
#include <future>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

namespace panther_utils
{

/**
 * @brief ROS service client that mimics synchronous behavior. When using inside a rclcpp::Node
 * class, you must spin the node using MultiThreadExecutor to avoid a deadlock.
 */
template <typename ServiceT, typename DurationT = std::chrono::milliseconds>
class RosSyncClient
{
  using SharedRequest = typename ServiceT::Request::SharedPtr;
  using SharedResponse = typename ServiceT::Response::SharedPtr;

public:
  RosSyncClient(
    const rclcpp::Node::SharedPtr & node, const std::string & service_name,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_services_default)
  : node_(node)
  {
    callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ = node->create_client<ServiceT>(service_name, qos_profile, callback_group_);
  }

  ~RosSyncClient()
  {
    client_.reset();
    callback_group_.reset();
    node_.reset();
  }

  /**
   * @brief call ROS serviece and wait for response
   *
   * @returns shared pointer to a response from the server
   *
   * @exception std::runtime_error if timeout is reached waiting for service or response, or when
   * request was deferred by the service
   */
  SharedResponse Call(
    const SharedRequest & request, const DurationT & service_timeout = DurationT::max(),
    const DurationT & response_timeout = DurationT::max())
  {
    if (!client_->wait_for_service(DurationT(service_timeout))) {
      throw std::runtime_error("Timeout exceeded waiting for service.");
    }

    auto future = client_->async_send_request(request);
    auto status = future.wait_for(DurationT(response_timeout));

    if (status == std::future_status::timeout) {
      throw std::runtime_error("Timeout exceeded waiting for response from the service.");
    }

    if (status == std::future_status::deferred) {
      throw std::runtime_error("The service request was deferred.");
    }

    return future.get();
  }

  const char * GetServiceName() const { return client_->get_service_name(); }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
};

}  // namespace panther_utils

#endif  // PANTHER_UTILS_ROS_SYNC_CLIENT_HPP_
