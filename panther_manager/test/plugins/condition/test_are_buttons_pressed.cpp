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

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include "panther_manager/plugins/condition/are_buttons_pressed.hpp"
#include "utils/plugin_test_utils.hpp"

class TestAreButtonsPressed : public panther_manager::plugin_test_utils::PluginTestUtils
{
public:
  TestAreButtonsPressed();
  void PublishJoyMessage(const std::vector<int> & buttons);

protected:
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  std::map<std::string, std::string> params_ = {{"topic_name", "joy"}, {"buttons", "0;1;0"}};
};

TestAreButtonsPressed::TestAreButtonsPressed()
{
  RegisterNodeWithParams<panther_manager::AreButtonsPressed>("AreButtonsPressed");
  joy_publisher_ = bt_node_->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
}

void TestAreButtonsPressed::PublishJoyMessage(const std::vector<int> & buttons)
{
  sensor_msgs::msg::Joy msg;
  msg.buttons = buttons;
  joy_publisher_->publish(msg);
}

TEST_F(TestAreButtonsPressed, LoadingAreButtonsPressedPlugin)
{
  ASSERT_NO_THROW({ CreateTree("AreButtonsPressed", params_); });
}

TEST_F(TestAreButtonsPressed, NoMessage)
{
  ASSERT_NO_THROW({ CreateTree("AreButtonsPressed", params_); });

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestAreButtonsPressed, WrongMessageTooFewButtons)
{
  ASSERT_NO_THROW({ CreateTree("AreButtonsPressed", params_); });

  PublishJoyMessage({0, 1});

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestAreButtonsPressed, GoodMessageWrongButtonsState)
{
  ASSERT_NO_THROW({ CreateTree("AreButtonsPressed", params_); });

  PublishJoyMessage({0, 0, 0});

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestAreButtonsPressed, GoodMessageWithTooMuchButtonsAndGoodButtonsState)
{
  ASSERT_NO_THROW({ CreateTree("AreButtonsPressed", params_); });

  PublishJoyMessage({0, 1, 0, 0, 0, 1});

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestAreButtonsPressed, GoodMessageGoodButtonsState)
{
  ASSERT_NO_THROW({ CreateTree("AreButtonsPressed", params_); });

  PublishJoyMessage({0, 1, 0});

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
