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

#include <algorithm>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <panther_utils/test/test_utils.hpp>

#include <panther_hardware_interfaces/panther_system_ros_interface.hpp>

#include <test_constants.hpp>

TEST(TestPantherSystemRosInterface, test_initialization)
{
  std::vector<std::string> node_names;
  const std::string panther_system_node_name = "/panther_system_node";

  rclcpp::init(0, nullptr);
  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_panther_system_node");

  panther_hardware_interfaces::PantherSystemRosInterface panther_system_ros_interface;

  panther_system_ros_interface.Initialize();
  node_names = test_node->get_node_names();
  ASSERT_TRUE(
    std::find(node_names.begin(), node_names.end(), panther_system_node_name) != node_names.end());

  panther_system_ros_interface.Deinitialize();
  node_names = test_node->get_node_names();
  ASSERT_FALSE(
    std::find(node_names.begin(), node_names.end(), panther_system_node_name) != node_names.end());

  // Check if it is possible to create a node once again (if everything was cleaned up properly)
  panther_system_ros_interface.Initialize();
  node_names = test_node->get_node_names();
  ASSERT_TRUE(
    std::find(node_names.begin(), node_names.end(), panther_system_node_name) != node_names.end());

  panther_system_ros_interface.Deinitialize();

  rclcpp::shutdown();
}

TEST(TestPantherSystemRosInterface, test_activation)
{
  using panther_hardware_interfaces_test::kClearErrorsService;
  using panther_hardware_interfaces_test::kMotorControllersStateTopic;

  std::map<std::string, std::vector<std::string>> service_names_and_types;
  std::map<std::string, std::vector<std::string>> topic_names_and_types;

  rclcpp::init(0, nullptr);
  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_panther_system_node");

  panther_hardware_interfaces::PantherSystemRosInterface panther_system_ros_interface;

  panther_system_ros_interface.Initialize();
  panther_system_ros_interface.Activate();

  // Necessary to add some waiting, so that topic lists are updated
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  service_names_and_types = test_node->get_service_names_and_types();
  topic_names_and_types = test_node->get_topic_names_and_types();

  ASSERT_TRUE(service_names_and_types.find(kClearErrorsService) != service_names_and_types.end());
  ASSERT_TRUE(
    topic_names_and_types.find(kMotorControllersStateTopic) != topic_names_and_types.end());

  panther_system_ros_interface.Deactivate();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  service_names_and_types = test_node->get_service_names_and_types();
  topic_names_and_types = test_node->get_topic_names_and_types();

  ASSERT_FALSE(service_names_and_types.find(kClearErrorsService) != service_names_and_types.end());
  ASSERT_FALSE(
    topic_names_and_types.find(kMotorControllersStateTopic) != topic_names_and_types.end());

  panther_system_ros_interface.Activate();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  service_names_and_types = test_node->get_service_names_and_types();
  topic_names_and_types = test_node->get_topic_names_and_types();

  ASSERT_TRUE(service_names_and_types.find(kClearErrorsService) != service_names_and_types.end());
  ASSERT_TRUE(
    topic_names_and_types.find(kMotorControllersStateTopic) != topic_names_and_types.end());

  panther_system_ros_interface.Deactivate();
  panther_system_ros_interface.Deinitialize();

  panther_system_ros_interface.Initialize();
  panther_system_ros_interface.Activate();

  service_names_and_types = test_node->get_service_names_and_types();
  topic_names_and_types = test_node->get_topic_names_and_types();

  ASSERT_TRUE(service_names_and_types.find(kClearErrorsService) != service_names_and_types.end());
  ASSERT_TRUE(
    topic_names_and_types.find(kMotorControllersStateTopic) != topic_names_and_types.end());

  panther_system_ros_interface.Deactivate();
  panther_system_ros_interface.Deinitialize();

  rclcpp::shutdown();
}

TEST(TestPantherSystemRosInterface, test_clear_errors_srv)
{
  rclcpp::init(0, nullptr);
  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_panther_system_node");

  panther_hardware_interfaces::PantherSystemRosInterface panther_system_ros_interface;

  bool clear_errors = false;

  panther_system_ros_interface.Initialize();
  panther_system_ros_interface.Activate();

  auto clear_errors_client = test_node->create_client<std_srvs::srv::Trigger>(
    panther_hardware_interfaces_test::kClearErrorsService);

  clear_errors_client->wait_for_service();
  auto result =
    clear_errors_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

  ASSERT_TRUE(
    rclcpp::spin_until_future_complete(test_node, result) == rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(clear_errors);

  panther_system_ros_interface.Deactivate();
  panther_system_ros_interface.Deinitialize();

  rclcpp::shutdown();
}

TEST(TestPantherSystemRosInterface, test_error_flags)
{
  rclcpp::init(0, nullptr);
  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_panther_system_node");

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = test_node->create_subscription<panther_msgs::msg::DriverState>(
    panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  panther_hardware_interfaces::PantherSystemRosInterface panther_system_ros_interface;

  panther_system_ros_interface.Initialize();
  panther_system_ros_interface.Activate();

  panther_hardware_interfaces::RoboteqData front(
    panther_hardware_interfaces_test::kDrivetrainSettings);
  panther_hardware_interfaces::RoboteqData rear(
    panther_hardware_interfaces_test::kDrivetrainSettings);

  front.SetFlags(0b00000001, 0b00000010, 0b00000100, 0b00001000);
  rear.SetFlags(0b00000010, 0b00000001, 0b00010000, 0b00100000);

  panther_system_ros_interface.UpdateMsgErrorFlags(front, rear);
  panther_system_ros_interface.PublishDriverState();

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(test_node, state_msg, std::chrono::seconds(5)));

  ASSERT_TRUE(state_msg->front.fault_flag.overheat);
  ASSERT_TRUE(state_msg->front.script_flag.encoder_disconected);
  ASSERT_TRUE(state_msg->front.left_motor_runtime_error.loop_error);
  ASSERT_TRUE(state_msg->front.right_motor_runtime_error.safety_stop_active);

  ASSERT_TRUE(state_msg->rear.fault_flag.overvoltage);
  ASSERT_TRUE(state_msg->rear.script_flag.loop_error);
  ASSERT_TRUE(state_msg->rear.left_motor_runtime_error.forward_limit_triggered);
  ASSERT_TRUE(state_msg->rear.right_motor_runtime_error.reverse_limit_triggered);

  panther_system_ros_interface.Deactivate();
  panther_system_ros_interface.Deinitialize();

  rclcpp::shutdown();
}

TEST(TestPantherSystemRosInterface, test_drivers_parameters)
{
  rclcpp::init(0, nullptr);
  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_panther_system_node");

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = test_node->create_subscription<panther_msgs::msg::DriverState>(
    panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  panther_hardware_interfaces::PantherSystemRosInterface panther_system_ros_interface;

  panther_system_ros_interface.Initialize();
  panther_system_ros_interface.Activate();

  panther_hardware_interfaces::DriverState front;
  panther_hardware_interfaces::DriverState rear;

  const std::int16_t f_temp = 36;
  const std::uint16_t f_volt = 405;
  const std::int16_t f_bat_amps_1 = 15;
  const std::int16_t f_bat_amps_2 = 12;
  const std::int16_t r_temp = 35;
  const std::uint16_t r_volt = 404;
  const std::int16_t r_bat_amps_1 = 14;
  const std::int16_t r_bat_amps_2 = 11;

  front.SetTemperature(f_temp);
  front.SetVoltage(f_volt);
  front.SetBatAmps1(f_bat_amps_1);
  front.SetBatAmps2(f_bat_amps_2);

  rear.SetTemperature(r_temp);
  rear.SetVoltage(r_volt);
  rear.SetBatAmps1(r_bat_amps_1);
  rear.SetBatAmps2(r_bat_amps_2);

  panther_system_ros_interface.UpdateMsgDriversParameters(front, rear);
  panther_system_ros_interface.PublishDriverState();

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(test_node, state_msg, std::chrono::seconds(5)));

  ASSERT_FLOAT_EQ(static_cast<std::int16_t>(state_msg->front.temperature), f_temp);
  ASSERT_FLOAT_EQ(static_cast<std::int16_t>(state_msg->rear.temperature), r_temp);

  ASSERT_FLOAT_EQ(static_cast<std::uint16_t>(state_msg->front.voltage * 10.0), f_volt);
  ASSERT_FLOAT_EQ(static_cast<std::uint16_t>(state_msg->rear.voltage * 10.0), r_volt);

  ASSERT_FLOAT_EQ(
    static_cast<std::int16_t>(state_msg->front.current * 10.0), (f_bat_amps_1 + f_bat_amps_2));
  ASSERT_FLOAT_EQ(
    static_cast<std::int16_t>(state_msg->rear.current * 10.0), (r_bat_amps_1 + r_bat_amps_2));

  panther_system_ros_interface.Deactivate();
  panther_system_ros_interface.Deinitialize();

  rclcpp::shutdown();
}

TEST(TestPantherSystemRosInterface, test_errors)
{
  rclcpp::init(0, nullptr);
  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_panther_system_node");

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = test_node->create_subscription<panther_msgs::msg::DriverState>(
    panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  panther_hardware_interfaces::PantherSystemRosInterface panther_system_ros_interface;

  panther_system_ros_interface.Initialize();
  panther_system_ros_interface.Activate();

  panther_hardware_interfaces::CanErrors can_errors;
  can_errors.error = true;
  can_errors.write_sdo_error = true;
  can_errors.read_sdo_error = false;
  can_errors.read_pdo_error = false;
  can_errors.front_data_timed_out = true;
  can_errors.rear_data_timed_out = false;
  can_errors.front_can_net_err = false;
  can_errors.rear_can_net_err = true;

  panther_system_ros_interface.UpdateMsgErrors(can_errors);

  panther_system_ros_interface.PublishDriverState();

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(test_node, state_msg, std::chrono::seconds(5)));

  ASSERT_TRUE(state_msg->error);
  ASSERT_TRUE(state_msg->write_sdo_error);
  ASSERT_FALSE(state_msg->read_sdo_error);
  ASSERT_FALSE(state_msg->read_pdo_error);

  ASSERT_TRUE(state_msg->front.data_timed_out);
  ASSERT_FALSE(state_msg->rear.data_timed_out);

  ASSERT_FALSE(state_msg->front.can_net_err);
  ASSERT_TRUE(state_msg->rear.can_net_err);

  panther_system_ros_interface.Deactivate();
  panther_system_ros_interface.Deinitialize();

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
