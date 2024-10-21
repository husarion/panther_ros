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
#include <memory>
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
using bt_ports = std::map<std::string, std::string>;

struct TestCase
{
  BT::NodeStatus result;
  bt_ports input;
  JoyMsg msg;
};

constexpr auto TOPIC = "joy";
constexpr auto PLUGIN = "CheckJoyMsg";
class TestCheckJoyMsg : public panther_manager::plugin_test_utils::PluginTestUtils
{
public:
  TestCheckJoyMsg();
  void PublishMsg(JoyMsg msg) { joy_publisher_->publish(msg); };
  JoyMsg CreateMsg(
    const std::vector<float> & axes = {}, const std::vector<int> & buttons = {},
    const HeaderMsg & header = HeaderMsg());
  void SetCurrentMsgTime(JoyMsg & header);

protected:
  rclcpp::Publisher<JoyMsg>::SharedPtr joy_publisher_;
};

TestCheckJoyMsg::TestCheckJoyMsg()
{
  RegisterNodeWithParams<panther_manager::CheckJoyMsg>(PLUGIN);
  joy_publisher_ = bt_node_->create_publisher<JoyMsg>(TOPIC, 10);
}

JoyMsg TestCheckJoyMsg::CreateMsg(
  const std::vector<float> & axes, const std::vector<int> & buttons, const HeaderMsg & header)
{
  JoyMsg msg;
  msg.header = header;
  msg.axes = axes;
  msg.buttons = buttons;
  return msg;
}

void TestCheckJoyMsg::SetCurrentMsgTime(JoyMsg & msg)
{
  msg.header.stamp.sec = bt_node_->now().seconds();
  msg.header.stamp.nanosec = bt_node_->now().nanoseconds();
}

TEST_F(TestCheckJoyMsg, NoTopicSet)
{
  bt_ports input = {{"topic_name", ""}, {"axes", "0;0"}, {"buttons", "0;0"}, {"timeout", "1.0"}};
  ASSERT_THROW(CreateTree(PLUGIN, input), std::logic_error);
}

TEST_F(TestCheckJoyMsg, NoMessage)
{
  bt_ports input = {{"topic_name", TOPIC}, {"axes", "0;0"}, {"buttons", "0;0"}, {"timeout", "1.0"}};
  ASSERT_NO_THROW({ CreateTree(PLUGIN, input); });

  auto & tree = GetTree();
  auto status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCheckJoyMsg, TimeoutTests)
{
  std::vector<TestCase> test_cases = {
    {BT::NodeStatus::SUCCESS,
     {{"topic_name", TOPIC}, {"axes", ""}, {"buttons", ""}, {"timeout", "1.0"}},
     CreateMsg()},
    {BT::NodeStatus::SUCCESS,
     {{"topic_name", TOPIC}, {"axes", ""}, {"buttons", ""}, {"timeout", "0.0"}},
     CreateMsg()},
    {BT::NodeStatus::SUCCESS,
     {{"topic_name", TOPIC}, {"axes", ""}, {"buttons", ""}, {"timeout", "-1.0"}},
     CreateMsg()},
    {BT::NodeStatus::FAILURE,
     {{"topic_name", TOPIC}, {"axes", ""}, {"buttons", ""}, {"timeout", "0.001"}},
     CreateMsg()}};

  for (auto & test_case : test_cases) {
    CreateTree(PLUGIN, test_case.input);
    SetCurrentMsgTime(test_case.msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    PublishMsg(test_case.msg);

    auto & tree = GetTree();
    auto status = tree.tickWhileRunning();

    EXPECT_EQ(status, test_case.result);
  }
}

TEST_F(TestCheckJoyMsg, OnTickBehavior)
{
  std::vector<TestCase> test_cases = {
    {BT::NodeStatus::SUCCESS,
     {{"topic_name", TOPIC}, {"axes", ""}, {"buttons", ""}, {"timeout", "1.0"}},
     CreateMsg()},
    {BT::NodeStatus::SUCCESS,
     {{"topic_name", TOPIC}, {"axes", "1"}, {"buttons", ""}, {"timeout", "1.0"}},
     CreateMsg({1})},
    {BT::NodeStatus::SUCCESS,
     {{"topic_name", TOPIC}, {"axes", ""}, {"buttons", "1"}, {"timeout", "1.0"}},
     CreateMsg({}, {1})},
    {BT::NodeStatus::SUCCESS,
     {{"topic_name", TOPIC}, {"axes", "0;0"}, {"buttons", "0;0"}, {"timeout", "1.0"}},
     CreateMsg({0, 0}, {0, 0})},
    {BT::NodeStatus::SUCCESS,
     {{"topic_name", TOPIC}, {"axes", "34;0;1"}, {"buttons", "0;0;10;0"}, {"timeout", "1.0"}},
     CreateMsg({34, 0, 1}, {0, 0, 10, 0})},
    {BT::NodeStatus::SUCCESS,
     {{"topic_name", TOPIC}, {"axes", "34;0;1"}, {"buttons", ""}, {"timeout", "1.0"}},
     CreateMsg({34, 0, 1}, {0, 0, 10, 0})},
    {BT::NodeStatus::SUCCESS,
     {{"topic_name", TOPIC}, {"axes", ""}, {"buttons", "0;0;10;0"}, {"timeout", "1.0"}},
     CreateMsg({34, 0, 1}, {0, 0, 10, 0})},
    {BT::NodeStatus::FAILURE,
     {{"topic_name", TOPIC}, {"axes", "34;0;1"}, {"buttons", "0;0;10;0"}, {"timeout", "1.0"}},
     CreateMsg({33, 0, 1}, {0, 0, 10, 0})},
    {BT::NodeStatus::FAILURE,
     {{"topic_name", TOPIC}, {"axes", "0;0;0"}, {"buttons", "0;0"}, {"timeout", "1.0"}},
     CreateMsg({0, 0}, {0, 0})},
    {BT::NodeStatus::FAILURE,
     {{"topic_name", TOPIC}, {"axes", "0;0"}, {"buttons", "0;0;0"}, {"timeout", "1.0"}},
     CreateMsg({0, 0}, {0, 0})},
    {BT::NodeStatus::FAILURE,
     {{"topic_name", TOPIC}, {"axes", "0;0"}, {"buttons", "0;0"}, {"timeout", "1.0"}},
     CreateMsg({0, 0, 0}, {0, 0})},
    {BT::NodeStatus::FAILURE,
     {{"topic_name", TOPIC}, {"axes", "0;0"}, {"buttons", "0;0"}, {"timeout", "1.0"}},
     CreateMsg({0, 0}, {0, 0, 0})},
    {BT::NodeStatus::FAILURE,
     {{"topic_name", TOPIC}, {"axes", "0;0"}, {"buttons", "0;0"}, {"timeout", "1.0"}},
     CreateMsg({0, 1}, {0, 0})},
    {BT::NodeStatus::FAILURE,
     {{"topic_name", TOPIC}, {"axes", "0;0"}, {"buttons", "0;0"}, {"timeout", "1.0"}},
     CreateMsg({0, 0}, {0, 1})},
    {BT::NodeStatus::FAILURE,
     {{"topic_name", TOPIC}, {"axes", "0;0"}, {"buttons", "0;0"}, {"timeout", "1.0"}},
     CreateMsg({1, 1}, {1, 1})},
    {BT::NodeStatus::FAILURE,
     {{"topic_name", TOPIC}, {"axes", "0;0"}, {"buttons", "0;0;0"}, {"timeout", "1.0"}},
     CreateMsg({1, 1}, {1, 1, 1})}};

  for (auto & test_case : test_cases) {
    CreateTree(PLUGIN, test_case.input);
    SetCurrentMsgTime(test_case.msg);
    PublishMsg(test_case.msg);

    auto & tree = GetTree();
    auto status = tree.tickWhileRunning();

    EXPECT_EQ(status, test_case.result);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
