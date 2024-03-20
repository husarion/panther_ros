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
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <panther_manager/plugins/action/call_set_led_animation_service_node.hpp>
#include <plugin_test_utils.hpp>

void ServiceFailedCallback(
  const panther_msgs::srv::SetLEDAnimation::Request::SharedPtr request,
  panther_msgs::srv::SetLEDAnimation::Response::SharedPtr response)
{
  response->message = "Failed callback pass!";
  response->success = false;
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("test_set_led_animation_plugin"),
    response->message << " success: " << response->success << " id: " << request->animation.id
                      << " param: " << request->animation.param
                      << " repeating: " << request->repeating);
}

void ServiceSuccessCallbackCheckRepeatingTrueValue(
  const panther_msgs::srv::SetLEDAnimation::Request::SharedPtr request,
  panther_msgs::srv::SetLEDAnimation::Response::SharedPtr response)
{
  response->message = "Successfully callback pass!";
  response->success = true;
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("test_set_led_animation_plugin"),
    response->message << " success: " << response->success << " id: " << request->animation.id
                      << " param: " << request->animation.param
                      << " repeating: " << request->repeating);

  EXPECT_EQ(request->repeating, true);
}

void ServiceSuccessCallbackCheckRepeatingFalseValue(
  const panther_msgs::srv::SetLEDAnimation::Request::SharedPtr request,
  panther_msgs::srv::SetLEDAnimation::Response::SharedPtr response)
{
  response->message = "Successfully callback pass!";
  response->success = true;
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("test_set_led_animation_plugin"),
    response->message << " success: " << response->success << " id: " << request->animation.id
                      << " param: " << request->animation.param
                      << " repeating: " << request->repeating);

  EXPECT_EQ(request->repeating, false);
}

void ServiceSuccessCallbackCheckId5(
  const panther_msgs::srv::SetLEDAnimation::Request::SharedPtr request,
  panther_msgs::srv::SetLEDAnimation::Response::SharedPtr response)
{
  response->message = "Successfully callback pass!";
  response->success = true;
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("test_set_led_animation_plugin"),
    response->message << " success: " << response->success << " id: " << request->animation.id
                      << " param: " << request->animation.param
                      << " repeating: " << request->repeating);

  EXPECT_EQ(request->animation.id, 5u);
}

TEST(TestCallSetLedAnimationService, GoodLoadingCallSetLedAnimationServicePlugin)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  ASSERT_NO_THROW({ test_utils.CreateTree("CallSetLedAnimationService", service); });
}

TEST(TestCallSetLedAnimationService, WrongPluginNameLoadingCallSetLedAnimationServicePlugin)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  EXPECT_THROW(
    { test_utils.CreateTree("WrongCallSetLedAnimationService", service); }, BT::RuntimeError);
}

TEST(TestCallSetLedAnimationService, WrongCallSetLedAnimationServiceServiceServerNotInitialized)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  test_utils.CreateTree("CallSetLedAnimationService", service);
  auto & tree = test_utils.GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(TestCallSetLedAnimationService, GoodSetLedAnimationCallServiceSuccessWithTrueRepeatingValue)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  test_utils.CreateTree("CallSetLedAnimationService", service);
  auto & tree = test_utils.GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  test_utils.CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "test_set_led_animation_service", ServiceSuccessCallbackCheckRepeatingTrueValue);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(TestCallSetLedAnimationService, GoodSetLedAnimationCallServiceSuccessWithFalseRepeatingValue)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "false"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  test_utils.CreateTree("CallSetLedAnimationService", service);
  auto & tree = test_utils.GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  test_utils.CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "test_set_led_animation_service", ServiceSuccessCallbackCheckRepeatingFalseValue);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(TestCallSetLedAnimationService, GoodSetLedAnimationCallServiceSuccessWith_5IdValue)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "5"}, {"param", ""}, {"repeating", "true"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  test_utils.CreateTree("CallSetLedAnimationService", service);
  auto & tree = test_utils.GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  test_utils.CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "test_set_led_animation_service", ServiceSuccessCallbackCheckId5);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(TestCallSetLedAnimationService, WrongSetLedAnimationCallServiceFailure)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  test_utils.CreateTree("CallSetLedAnimationService", service);
  auto & tree = test_utils.GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  test_utils.CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "test_set_led_animation_service", ServiceFailedCallback);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(TestCallSetLedAnimationService, WrongRepeatingServiceValueDefined)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "wrong_bool"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  test_utils.CreateTree("CallSetLedAnimationService", service);
  auto & tree = test_utils.GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  test_utils.CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "test_set_led_animation_service", ServiceFailedCallback);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(TestCallSetLedAnimationService, WrongIdServiceValueDefined)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "-5"}, {"param", ""}, {"repeating", "true"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");

  test_utils.CreateTree("CallSetLedAnimationService", service);
  auto & tree = test_utils.GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  test_utils.CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "test_set_led_animation_service", ServiceFailedCallback);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
