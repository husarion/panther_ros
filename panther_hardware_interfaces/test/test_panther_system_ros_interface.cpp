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
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <panther_utils/test/ros_test_utils.hpp>

#include <panther_hardware_interfaces/panther_system_ros_interface.hpp>

#include <test_constants.hpp>

TEST(TestPantherSystemRosInterface, TestNode)
{
  using panther_hardware_interfaces::PantherSystemRosInterface;

  std::vector<std::string> node_names;
  const std::string panther_system_node_name = "panther_system_node";
  const std::string panther_system_node_name_with_ns = "/" + panther_system_node_name;

  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_panther_system_node");

  std::unique_ptr<PantherSystemRosInterface> panther_system_ros_interface;

  panther_system_ros_interface =
    std::make_unique<PantherSystemRosInterface>(panther_system_node_name);
  node_names = test_node->get_node_names();
  ASSERT_TRUE(
    std::find(node_names.begin(), node_names.end(), panther_system_node_name_with_ns) !=
    node_names.end());

  panther_system_ros_interface.reset();
  node_names = test_node->get_node_names();
  ASSERT_FALSE(
    std::find(node_names.begin(), node_names.end(), panther_system_node_name_with_ns) !=
    node_names.end());

  // Check if it is possible to create a node once again (if everything was cleaned up properly)
  panther_system_ros_interface =
    std::make_unique<PantherSystemRosInterface>(panther_system_node_name);
  node_names = test_node->get_node_names();
  ASSERT_TRUE(
    std::find(node_names.begin(), node_names.end(), panther_system_node_name_with_ns) !=
    node_names.end());

  panther_system_ros_interface.reset();
}

TEST(TestPantherSystemRosInterface, Activation)
{
  using panther_hardware_interfaces::PantherSystemRosInterface;
  using panther_hardware_interfaces_test::kMotorControllersStateTopic;

  std::map<std::string, std::vector<std::string>> service_names_and_types;
  std::map<std::string, std::vector<std::string>> topic_names_and_types;

  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_panther_system_node");

  std::unique_ptr<PantherSystemRosInterface> panther_system_ros_interface =
    std::make_unique<PantherSystemRosInterface>("panther_system_node");

  // Necessary to add some waiting, so that topic lists are updated
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  topic_names_and_types = test_node->get_topic_names_and_types();
  ASSERT_TRUE(
    topic_names_and_types.find(kMotorControllersStateTopic) != topic_names_and_types.end());

  panther_system_ros_interface.reset();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  topic_names_and_types = test_node->get_topic_names_and_types();
  ASSERT_FALSE(
    topic_names_and_types.find(kMotorControllersStateTopic) != topic_names_and_types.end());

  panther_system_ros_interface = std::make_unique<PantherSystemRosInterface>("panther_system_node");

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  topic_names_and_types = test_node->get_topic_names_and_types();
  ASSERT_TRUE(
    topic_names_and_types.find(kMotorControllersStateTopic) != topic_names_and_types.end());

  panther_system_ros_interface.reset();
}

TEST(TestPantherSystemRosInterface, ErrorFlags)
{
  using panther_hardware_interfaces::PantherSystemRosInterface;

  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_panther_system_node");

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = test_node->create_subscription<panther_msgs::msg::DriverState>(
    panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::unique_ptr<PantherSystemRosInterface> panther_system_ros_interface =
    std::make_unique<PantherSystemRosInterface>("panther_system_node");

  panther_hardware_interfaces::RoboteqData front(
    panther_hardware_interfaces_test::kDrivetrainSettings);
  panther_hardware_interfaces::RoboteqData rear(
    panther_hardware_interfaces_test::kDrivetrainSettings);

  panther_hardware_interfaces::RoboteqDriverState front_driver_state;
  front_driver_state.fault_flags = 0b00000001;
  front_driver_state.script_flags = 0b00000010;
  front_driver_state.runtime_stat_flag_motor_1 = 0b00001000;
  front_driver_state.runtime_stat_flag_motor_2 = 0b00000100;

  panther_hardware_interfaces::RoboteqDriverState rear_driver_state;
  rear_driver_state.fault_flags = 0b00000010;
  rear_driver_state.script_flags = 0b00000001;
  rear_driver_state.runtime_stat_flag_motor_1 = 0b00100000;
  rear_driver_state.runtime_stat_flag_motor_2 = 0b00010000;

  front.SetDriverState(front_driver_state, false);
  rear.SetDriverState(rear_driver_state, false);

  panther_system_ros_interface->UpdateMsgErrorFlags(front, rear);
  panther_system_ros_interface->PublishDriverState();

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(test_node, state_msg, std::chrono::seconds(5)));

  ASSERT_TRUE(state_msg->front.fault_flag.overheat);
  ASSERT_TRUE(state_msg->front.script_flag.encoder_disconnected);
  ASSERT_TRUE(state_msg->front.left_motor_runtime_error.loop_error);
  ASSERT_TRUE(state_msg->front.right_motor_runtime_error.safety_stop_active);

  ASSERT_TRUE(state_msg->rear.fault_flag.overvoltage);
  ASSERT_TRUE(state_msg->rear.script_flag.loop_error);
  ASSERT_TRUE(state_msg->rear.left_motor_runtime_error.forward_limit_triggered);
  ASSERT_TRUE(state_msg->rear.right_motor_runtime_error.reverse_limit_triggered);

  panther_system_ros_interface.reset();
}

TEST(TestPantherSystemRosInterface, DriversStates)
{
  using panther_hardware_interfaces::PantherSystemRosInterface;

  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_panther_system_node");

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = test_node->create_subscription<panther_msgs::msg::DriverState>(
    panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::unique_ptr<PantherSystemRosInterface> panther_system_ros_interface =
    std::make_unique<PantherSystemRosInterface>("panther_system_node");

  panther_hardware_interfaces::DriverState front;
  panther_hardware_interfaces::DriverState rear;

  const std::int16_t f_temp = 36;
  const std::int16_t f_heatsink_temp = 37;
  const std::uint16_t f_volt = 405;
  const std::int16_t f_battery_current_1 = 15;
  const std::int16_t f_battery_current_2 = 12;
  const std::int16_t r_temp = 35;
  const std::int16_t r_heatsink_temp = 36;
  const std::uint16_t r_volt = 404;
  const std::int16_t r_battery_current_1 = 14;
  const std::int16_t r_battery_current_2 = 11;

  front.SetTemperature(f_temp);
  front.SetHeatsinkTemperature(f_heatsink_temp);
  front.SetVoltage(f_volt);
  front.SetBatteryCurrent1(f_battery_current_1);
  front.SetBatteryCurrent2(f_battery_current_2);

  rear.SetTemperature(r_temp);
  rear.SetHeatsinkTemperature(r_heatsink_temp);
  rear.SetVoltage(r_volt);
  rear.SetBatteryCurrent1(r_battery_current_1);
  rear.SetBatteryCurrent2(r_battery_current_2);

  panther_system_ros_interface->UpdateMsgDriversStates(front, rear);
  panther_system_ros_interface->PublishDriverState();

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(test_node, state_msg, std::chrono::seconds(5)));

  ASSERT_FLOAT_EQ(static_cast<std::int16_t>(state_msg->front.temperature), f_temp);
  ASSERT_FLOAT_EQ(static_cast<std::int16_t>(state_msg->rear.temperature), r_temp);

  ASSERT_FLOAT_EQ(
    static_cast<std::int16_t>(state_msg->front.heatsink_temperature), f_heatsink_temp);
  ASSERT_FLOAT_EQ(static_cast<std::int16_t>(state_msg->rear.heatsink_temperature), r_heatsink_temp);

  ASSERT_FLOAT_EQ(static_cast<std::uint16_t>(state_msg->front.voltage * 10.0), f_volt);
  ASSERT_FLOAT_EQ(static_cast<std::uint16_t>(state_msg->rear.voltage * 10.0), r_volt);

  ASSERT_FLOAT_EQ(
    static_cast<std::int16_t>(state_msg->front.current * 10.0),
    (f_battery_current_1 + f_battery_current_2));
  ASSERT_FLOAT_EQ(
    static_cast<std::int16_t>(state_msg->rear.current * 10.0),
    (r_battery_current_1 + r_battery_current_2));

  panther_system_ros_interface.reset();
}

TEST(TestPantherSystemRosInterface, Errors)
{
  using panther_hardware_interfaces::PantherSystemRosInterface;

  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_panther_system_node");

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = test_node->create_subscription<panther_msgs::msg::DriverState>(
    panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::unique_ptr<PantherSystemRosInterface> panther_system_ros_interface =
    std::make_unique<PantherSystemRosInterface>("panther_system_node");

  panther_hardware_interfaces::CANErrors can_errors;
  can_errors.error = true;

  can_errors.write_pdo_cmds_error = true;
  can_errors.read_pdo_motor_states_error = false;
  can_errors.read_pdo_driver_state_error = false;

  can_errors.front_motor_states_data_timed_out = true;
  can_errors.rear_motor_states_data_timed_out = false;

  can_errors.front_driver_state_data_timed_out = false;
  can_errors.rear_driver_state_data_timed_out = true;

  can_errors.front_can_net_err = false;
  can_errors.rear_can_net_err = true;

  panther_system_ros_interface->UpdateMsgErrors(can_errors);

  panther_system_ros_interface->PublishDriverState();

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(test_node, state_msg, std::chrono::seconds(5)));

  ASSERT_TRUE(state_msg->error);

  ASSERT_TRUE(state_msg->write_pdo_cmds_error);
  ASSERT_FALSE(state_msg->read_pdo_motor_states_error);
  ASSERT_FALSE(state_msg->read_pdo_driver_state_error);

  ASSERT_TRUE(state_msg->front.motor_states_data_timed_out);
  ASSERT_FALSE(state_msg->rear.motor_states_data_timed_out);

  ASSERT_FALSE(state_msg->front.driver_state_data_timed_out);
  ASSERT_TRUE(state_msg->rear.driver_state_data_timed_out);

  ASSERT_FALSE(state_msg->front.can_net_err);
  ASSERT_TRUE(state_msg->rear.can_net_err);

  panther_system_ros_interface.reset();
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
