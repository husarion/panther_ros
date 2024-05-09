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

#ifndef PANTHER_MANAGER_MANAGER_BT_TEST_UTILS_
#define PANTHER_MANAGER_MANAGER_BT_TEST_UTILS_

#include <chrono>
#include <functional>
#include <thread>

#include <gtest/gtest.h>

#include "behaviortree_cpp/basic_types.h"

#include "rclcpp/rclcpp.hpp"

namespace test_behavior_tree_utils
{

void SpinWhileRunning(const rclcpp::Node::SharedPtr node, std::function<BT::NodeStatus()> GetStatus)
{
  auto status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING || status == BT::NodeStatus::IDLE ||
         status == BT::NodeStatus::SKIPPED) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    status = GetStatus();
  }

  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
}

}  // namespace test_behavior_tree_utils

#endif  // PANTHER_MANAGER_MANAGER_BT_TEST_UTILS_
