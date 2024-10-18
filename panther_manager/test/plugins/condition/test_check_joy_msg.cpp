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
#include <std_msgs/msg/header.hpp>

#include "panther_manager/plugins/condition/check_joy_msg.hpp"
#include "utils/plugin_test_utils.hpp"

using JoyMsg = sensor_msgs::msg::Joy;
using HeaderMsg = std_msgs::msg::Header;

class TestCheckJoyMsg : public panther_manager::plugin_test_utils::PluginTestUtils
{
public:
  TestCheckJoyMsg();
  JoyMsg CreateJoyMsg(
    const HeaderMsg & header = HeaderMsg(), const std::vector<float> & axes = {},
    const std::vector<int> & buttons = {});
  void PublishJoyMsg(JoyMsg msg);

protected:
  rclcpp::Publisher<JoyMsg>::SharedPtr joy_publisher_;
  std::map<std::string, std::string> bb_ports_ = {
    {"topic_name", "joy"}, {"axes", ""}, {"buttons", "0;1;0"}, {"timeout", "0.0"}};
};

TestCheckJoyMsg::TestCheckJoyMsg()
{
  RegisterNodeWithParams<panther_manager::CheckJoyMsg>("CheckJoyMsg");
  joy_publisher_ = bt_node_->create_publisher<JoyMsg>("joy", 10);
}

JoyMsg TestCheckJoyMsg::CreateJoyMsg(
  const HeaderMsg & header, const std::vector<float> & axes, const std::vector<int> & buttons)
{
  JoyMsg msg;
  msg.header = header;
  msg.axes = axes;
  msg.buttons = buttons;
  return msg;
}

void TestCheckJoyMsg::PublishJoyMsg(JoyMsg msg) { joy_publisher_->publish(msg); }

TEST_F(TestCheckJoyMsg, LoadingCheckJoyMsgPlugin)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", bb_ports_); });
}

TEST_F(TestCheckJoyMsg, NoMessage)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", bb_ports_); });

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCheckJoyMsg, WrongMessageTooFewButtons)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", bb_ports_); });

  auto msg = CreateJoyMsg(HeaderMsg(), {}, {0, 1});
  PublishJoyMsg(msg);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCheckJoyMsg, GoodMessageWrongButtonsState)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", bb_ports_); });

  auto msg = CreateJoyMsg(HeaderMsg(), {}, {0, 0, 0});
  PublishJoyMsg(msg);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCheckJoyMsg, GoodMessageWithTooMuchButtonsAndGoodButtonsState)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", bb_ports_); });

  auto msg = CreateJoyMsg(HeaderMsg(), {}, {0, 1, 0, 0, 0, 1});
  PublishJoyMsg(msg);

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCheckJoyMsg, GoodMessageGoodButtonsState)
{
  ASSERT_NO_THROW({ CreateTree("CheckJoyMsg", bb_ports_); });

  auto msg = CreateJoyMsg(HeaderMsg(), {}, {0, 1, 0});
  PublishJoyMsg(msg);

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
