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

class TestCallSetLedAnimationService : public panther_manager::plugin_test_utils::PluginTestUtils
{
public:
  void ServiceFailedCallback(
    const panther_msgs::srv::SetLEDAnimation::Request::SharedPtr request,
    panther_msgs::srv::SetLEDAnimation::Response::SharedPtr response);

  void ServiceSuccessCallbackCheckRepeatingTrueValue(
    const panther_msgs::srv::SetLEDAnimation::Request::SharedPtr request,
    panther_msgs::srv::SetLEDAnimation::Response::SharedPtr response);

  void ServiceSuccessCallbackCheckRepeatingFalseValue(
    const panther_msgs::srv::SetLEDAnimation::Request::SharedPtr request,
    panther_msgs::srv::SetLEDAnimation::Response::SharedPtr response);

  void ServiceSuccessCallbackCheckId5(
    const panther_msgs::srv::SetLEDAnimation::Request::SharedPtr request,
    panther_msgs::srv::SetLEDAnimation::Response::SharedPtr response);
};

void TestCallSetLedAnimationService::ServiceFailedCallback(
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

void TestCallSetLedAnimationService::ServiceSuccessCallbackCheckRepeatingTrueValue(
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

void TestCallSetLedAnimationService::ServiceSuccessCallbackCheckRepeatingFalseValue(
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

void TestCallSetLedAnimationService::ServiceSuccessCallbackCheckId5(
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

TEST_F(TestCallSetLedAnimationService, GoodLoadingCallSetLedAnimationServicePlugin)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>("CallSetLedAnimationService");

  ASSERT_NO_THROW({ CreateTree("CallSetLedAnimationService", service); });
}

TEST_F(TestCallSetLedAnimationService, WrongPluginNameLoadingCallSetLedAnimationServicePlugin)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>("CallSetLedAnimationService");

  EXPECT_THROW({ CreateTree("WrongCallSetLedAnimationService", service); }, BT::RuntimeError);
}

TEST_F(TestCallSetLedAnimationService, WrongCallSetLedAnimationServiceServiceServerNotInitialized)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>("CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCallSetLedAnimationService, GoodSetLedAnimationCallServiceSuccessWithTrueRepeatingValue)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>("CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  using std::placeholders::_1;
  using std::placeholders::_2;
  CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "set_led_animation",
    std::bind(
      &TestCallSetLedAnimationService::ServiceSuccessCallbackCheckRepeatingTrueValue, this, _1,
      _2));

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCallSetLedAnimationService, GoodSetLedAnimationCallServiceSuccessWithFalseRepeatingValue)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "false"}};

  RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>("CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  using std::placeholders::_1;
  using std::placeholders::_2;
  CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "set_led_animation",
    std::bind(
      &TestCallSetLedAnimationService::ServiceSuccessCallbackCheckRepeatingFalseValue, this, _1,
      _2));

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCallSetLedAnimationService, GoodSetLedAnimationCallServiceSuccessWith_5IdValue)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "5"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>("CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  using std::placeholders::_1;
  using std::placeholders::_2;
  CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "set_led_animation",
    std::bind(&TestCallSetLedAnimationService::ServiceSuccessCallbackCheckId5, this, _1, _2));

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestCallSetLedAnimationService, WrongSetLedAnimationCallServiceFailure)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>("CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  using std::placeholders::_1;
  using std::placeholders::_2;
  CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "set_led_animation",
    std::bind(&TestCallSetLedAnimationService::ServiceFailedCallback, this, _1, _2));

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCallSetLedAnimationService, WrongRepeatingServiceValueDefined)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "0"}, {"param", ""}, {"repeating", "wrong_bool"}};

  RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>("CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  using std::placeholders::_1;
  using std::placeholders::_2;
  CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "set_led_animation",
    std::bind(&TestCallSetLedAnimationService::ServiceFailedCallback, this, _1, _2));

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestCallSetLedAnimationService, WrongIdServiceValueDefined)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_led_animation"}, {"id", "-5"}, {"param", ""}, {"repeating", "true"}};

  RegisterNodeWithParams<panther_manager::CallSetLedAnimationService>("CallSetLedAnimationService");

  CreateTree("CallSetLedAnimationService", service);
  auto & tree = GetTree();

  using panther_msgs::srv::SetLEDAnimation;
  using std::placeholders::_1;
  using std::placeholders::_2;
  CreateService<SetLEDAnimation, SetLEDAnimation::Request, SetLEDAnimation::Response>(
    "set_led_animation",
    std::bind(&TestCallSetLedAnimationService::ServiceFailedCallback, this, _1, _2));

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
