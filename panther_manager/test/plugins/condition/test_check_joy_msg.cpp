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

#include "panther_manager/plugins/condition/check_joy_msg.hpp"
#include "utils/plugin_test_utils.hpp"

class TestCheckJoyMsg : public panther_manager::plugin_test_utils::PluginTestUtils
{
public:
  TestCheckJoyMsg();
  void PublishJoyMessage(const std::vector<int> & buttons);

protected:
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  std::map<std::string, std::string> params_ = {{"topic_name", "joy"}, {"buttons", "0;1;0"}};
};

TestCheckJoyMsg::TestCheckJoyMsg()
{
  RegisterNodeWithParams<panther_manager::CheckJoyMsg>("CheckJoyMsg");
  joy_publisher_ = bt_node_->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
}

void TestCheckJoyMsg::PublishJoyMessage(const std::vector<int> & buttons)
{
  sensor_msgs::msg::Joy msg;
  msg.buttons = buttons;
  joy_publisher_->publish(msg);
}

TEST_F(TestCheckJoyMsg, LoadingCheckJoyMsgPlugin)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", params_); });
}

TEST_F(TestCheckJoyMsg, NoMessage)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", params_); });

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCheckJoyMsg, WrongMessageTooFewButtons)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", params_); });

  PublishJoyMessage({0, 1});

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCheckJoyMsg, GoodMessageWrongButtonsState)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", params_); });

  PublishJoyMessage({0, 0, 0});

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCheckJoyMsg, GoodMessageWithTooMuchButtonsAndGoodButtonsState)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", params_); });

  PublishJoyMessage({0, 1, 0, 0, 0, 1});

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCheckJoyMsg, GoodMessageGoodButtonsState)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", params_); });

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
