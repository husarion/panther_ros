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

#include "panther_manager/plugins/action/call_set_bool_service_node.hpp"

namespace panther_manager
{

bool CallSetBoolService::setRequest(typename Request::SharedPtr & request)
{
  if (!getInput<bool>("data", request->data)) {
    RCLCPP_ERROR_STREAM(this->node_->get_logger(), "Failed to get input [data]");
    return false;
  }
  return true;
}

BT::NodeStatus CallSetBoolService::onResponseReceived(const typename Response::SharedPtr & response)
{
  if (!response->success) {
    RCLCPP_ERROR_STREAM(
      this->node_->get_logger(),
      "Failed to call " << this->prev_service_name_ << " service, message: " << response->message);
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_DEBUG_STREAM(
    this->node_->get_logger(), "Successfully called "
                                 << this->prev_service_name_
                                 << " service, message: " << response->message);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace panther_manager

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(panther_manager::CallSetBoolService, "CallSetBoolService");
