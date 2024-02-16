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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>

#include <panther_utils/test/ros_test_utils.hpp>

TEST(TestTestUtils, WaitForMessage)
{
  auto node = std::make_shared<rclcpp::Node>("node");
  auto pub = node->create_publisher<std_msgs::msg::Empty>("topic", 10);
  std_msgs::msg::Empty::SharedPtr empty_msg;
  auto sub = node->create_subscription<std_msgs::msg::Empty>(
    "topic", 10, [&](const std_msgs::msg::Empty::SharedPtr msg) { empty_msg = msg; });

  EXPECT_FALSE(
    panther_utils::test_utils::WaitForMsg(node, empty_msg, std::chrono::milliseconds(1000)));

  pub->publish(std_msgs::msg::Empty());
  EXPECT_TRUE(
    panther_utils::test_utils::WaitForMsg(node, empty_msg, std::chrono::milliseconds(1000)));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
