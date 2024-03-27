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

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <panther_utils/test/ros_test_utils.hpp>

#include <panther_system_test_utils.hpp>
#include <roboteqs_mock.hpp>

class TestPantherSystem : public ::testing::Test
{
public:
  TestPantherSystem() { pth_test_.Start(pth_test_.GetDefaultPantherSystemUrdf()); }
  ~TestPantherSystem() { pth_test_.Stop(); }

  void CheckInterfaces()
  {
    EXPECT_EQ(pth_test_.GetResourceManager()->system_components_size(), 1u);
    ASSERT_EQ(pth_test_.GetResourceManager()->state_interface_keys().size(), 12u);

    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("fl_wheel_joint/position"));
    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("fr_wheel_joint/position"));
    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("rl_wheel_joint/position"));
    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("rr_wheel_joint/position"));

    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("fl_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("fr_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("rl_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("rr_wheel_joint/velocity"));

    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("fl_wheel_joint/effort"));
    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("fr_wheel_joint/effort"));
    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("rl_wheel_joint/effort"));
    EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("rr_wheel_joint/effort"));

    ASSERT_EQ(pth_test_.GetResourceManager()->command_interface_keys().size(), 4u);

    EXPECT_TRUE(
      pth_test_.GetResourceManager()->command_interface_exists("fl_wheel_joint/velocity"));
    EXPECT_TRUE(
      pth_test_.GetResourceManager()->command_interface_exists("fr_wheel_joint/velocity"));
    EXPECT_TRUE(
      pth_test_.GetResourceManager()->command_interface_exists("rl_wheel_joint/velocity"));
    EXPECT_TRUE(
      pth_test_.GetResourceManager()->command_interface_exists("rr_wheel_joint/velocity"));
  }

  void CheckInitialValues()
  {
    using hardware_interface::LoanedCommandInterface;
    using hardware_interface::LoanedStateInterface;

    LoanedStateInterface fl_s_p =
      pth_test_.GetResourceManager()->claim_state_interface("fl_wheel_joint/position");
    LoanedStateInterface fr_s_p =
      pth_test_.GetResourceManager()->claim_state_interface("fr_wheel_joint/position");
    LoanedStateInterface rl_s_p =
      pth_test_.GetResourceManager()->claim_state_interface("rl_wheel_joint/position");
    LoanedStateInterface rr_s_p =
      pth_test_.GetResourceManager()->claim_state_interface("rr_wheel_joint/position");

    LoanedStateInterface fl_s_v =
      pth_test_.GetResourceManager()->claim_state_interface("fl_wheel_joint/velocity");
    LoanedStateInterface fr_s_v =
      pth_test_.GetResourceManager()->claim_state_interface("fr_wheel_joint/velocity");
    LoanedStateInterface rl_s_v =
      pth_test_.GetResourceManager()->claim_state_interface("rl_wheel_joint/velocity");
    LoanedStateInterface rr_s_v =
      pth_test_.GetResourceManager()->claim_state_interface("rr_wheel_joint/velocity");

    LoanedStateInterface fl_s_e =
      pth_test_.GetResourceManager()->claim_state_interface("fl_wheel_joint/effort");
    LoanedStateInterface fr_s_e =
      pth_test_.GetResourceManager()->claim_state_interface("fr_wheel_joint/effort");
    LoanedStateInterface rl_s_e =
      pth_test_.GetResourceManager()->claim_state_interface("rl_wheel_joint/effort");
    LoanedStateInterface rr_s_e =
      pth_test_.GetResourceManager()->claim_state_interface("rr_wheel_joint/effort");

    LoanedCommandInterface fl_c_v =
      pth_test_.GetResourceManager()->claim_command_interface("fl_wheel_joint/velocity");
    LoanedCommandInterface fr_c_v =
      pth_test_.GetResourceManager()->claim_command_interface("fr_wheel_joint/velocity");
    LoanedCommandInterface rl_c_v =
      pth_test_.GetResourceManager()->claim_command_interface("rl_wheel_joint/velocity");
    LoanedCommandInterface rr_c_v =
      pth_test_.GetResourceManager()->claim_command_interface("rr_wheel_joint/velocity");

    ASSERT_FLOAT_EQ(fl_s_p.get_value(), 0.0);
    ASSERT_FLOAT_EQ(fr_s_p.get_value(), 0.0);
    ASSERT_FLOAT_EQ(rl_s_p.get_value(), 0.0);
    ASSERT_FLOAT_EQ(rr_s_p.get_value(), 0.0);

    ASSERT_FLOAT_EQ(fl_s_v.get_value(), 0.0);
    ASSERT_FLOAT_EQ(fr_s_v.get_value(), 0.0);
    ASSERT_FLOAT_EQ(rl_s_v.get_value(), 0.0);
    ASSERT_FLOAT_EQ(rr_s_v.get_value(), 0.0);

    ASSERT_FLOAT_EQ(fl_s_e.get_value(), 0.0);
    ASSERT_FLOAT_EQ(fr_s_e.get_value(), 0.0);
    ASSERT_FLOAT_EQ(rl_s_e.get_value(), 0.0);
    ASSERT_FLOAT_EQ(rr_s_e.get_value(), 0.0);

    ASSERT_FLOAT_EQ(fl_c_v.get_value(), 0.0);
    ASSERT_FLOAT_EQ(fr_c_v.get_value(), 0.0);
    ASSERT_FLOAT_EQ(rl_c_v.get_value(), 0.0);
    ASSERT_FLOAT_EQ(rr_c_v.get_value(), 0.0);
  }

  // 100 Hz
  const float period_ = 0.01;

  panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;
};

// TRANSITIONS
TEST_F(TestPantherSystem, ConfigureActivateFinalizePantherSystem)
{
  using panther_hardware_interfaces_test::kPantherSystemName;

  // check if hardware is configured
  auto status_map = pth_test_.GetResourceManager()->get_components_status();
  ASSERT_EQ(
    status_map[kPantherSystemName].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);

  try {
    pth_test_.ConfigurePantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to ConfigurePantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.GetResourceManager()->get_components_status();
  ASSERT_EQ(
    status_map[kPantherSystemName].state.label(),
    hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    pth_test_.ActivatePantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to ActivatePantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.GetResourceManager()->get_components_status();
  ASSERT_EQ(
    status_map[kPantherSystemName].state.label(),
    hardware_interface::lifecycle_state_names::ACTIVE);

  // Check interfaces
  CheckInterfaces();

  // Check initial values
  CheckInitialValues();

  try {
    pth_test_.ShutdownPantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to ShutdownPantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.GetResourceManager()->get_components_status();
  ASSERT_EQ(
    status_map[kPantherSystemName].state.label(),
    hardware_interface::lifecycle_state_names::FINALIZED);
}

TEST_F(TestPantherSystem, ConfigureActivateDeactivateDeconfigurePantherSystem)
{
  using panther_hardware_interfaces_test::kPantherSystemName;

  auto status_map = pth_test_.GetResourceManager()->get_components_status();
  ASSERT_EQ(
    status_map[kPantherSystemName].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);

  try {
    pth_test_.ConfigurePantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to ConfigurePantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.GetResourceManager()->get_components_status();
  ASSERT_EQ(
    status_map[kPantherSystemName].state.label(),
    hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    pth_test_.ActivatePantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to ActivatePantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.GetResourceManager()->get_components_status();
  ASSERT_EQ(
    status_map[kPantherSystemName].state.label(),
    hardware_interface::lifecycle_state_names::ACTIVE);

  // Check interfaces
  CheckInterfaces();

  // Check initial values
  CheckInitialValues();

  try {
    pth_test_.DeactivatePantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to DeactivatePantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.GetResourceManager()->get_components_status();
  ASSERT_EQ(
    status_map[kPantherSystemName].state.label(),
    hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    pth_test_.UnconfigurePantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to UnconfigurePantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.GetResourceManager()->get_components_status();
  ASSERT_EQ(
    status_map[kPantherSystemName].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

// WRITING
TEST_F(TestPantherSystem, WriteCommandsPantherSystem)
{
  using hardware_interface::LoanedCommandInterface;

  using panther_hardware_interfaces_test::DriverChannel;

  using panther_hardware_interfaces_test::kRadPerSecToRbtqCmd;

  const float fl_v = 0.1;
  const float fr_v = 0.2;
  const float rl_v = 0.3;
  const float rr_v = 0.4;

  pth_test_.ConfigureActivatePantherSystem();

  LoanedCommandInterface fl_c_v =
    pth_test_.GetResourceManager()->claim_command_interface("fl_wheel_joint/velocity");
  LoanedCommandInterface fr_c_v =
    pth_test_.GetResourceManager()->claim_command_interface("fr_wheel_joint/velocity");
  LoanedCommandInterface rl_c_v =
    pth_test_.GetResourceManager()->claim_command_interface("rl_wheel_joint/velocity");
  LoanedCommandInterface rr_c_v =
    pth_test_.GetResourceManager()->claim_command_interface("rr_wheel_joint/velocity");

  fl_c_v.set_value(fl_v);
  fr_c_v.set_value(fr_v);
  rl_c_v.set_value(rl_v);
  rr_c_v.set_value(rr_v);

  ASSERT_FLOAT_EQ(fl_c_v.get_value(), fl_v);
  ASSERT_FLOAT_EQ(fr_c_v.get_value(), fr_v);
  ASSERT_FLOAT_EQ(rl_c_v.get_value(), rl_v);
  ASSERT_FLOAT_EQ(rr_c_v.get_value(), rr_v);

  const auto TIME = rclcpp::Time(0);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  pth_test_.GetResourceManager()->write(TIME, PERIOD);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<std::int32_t>(fl_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<std::int32_t>(fr_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<std::int32_t>(rl_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<std::int32_t>(rr_v * kRadPerSecToRbtqCmd));

  pth_test_.ShutdownPantherSystem();
}

// READING
TEST_F(TestPantherSystem, ReadFeedbackPantherSystem)
{
  using hardware_interface::LoanedStateInterface;

  using panther_hardware_interfaces_test::DriverChannel;

  using panther_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  using panther_hardware_interfaces_test::kRbtqPosFbToRad;
  using panther_hardware_interfaces_test::kRbtqVelFbToRadPerSec;

  const std::int32_t fl_val = 100;
  const std::int32_t fr_val = 200;
  const std::int32_t rl_val = 300;
  const std::int32_t rr_val = 400;

  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetPosition(DriverChannel::CHANNEL2, fl_val);
  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetPosition(DriverChannel::CHANNEL1, fr_val);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetPosition(DriverChannel::CHANNEL2, rl_val);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetPosition(DriverChannel::CHANNEL1, rr_val);

  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetVelocity(DriverChannel::CHANNEL2, fl_val);
  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetVelocity(DriverChannel::CHANNEL1, fr_val);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetVelocity(DriverChannel::CHANNEL2, rl_val);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetVelocity(DriverChannel::CHANNEL1, rr_val);

  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetCurrent(DriverChannel::CHANNEL2, fl_val);
  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetCurrent(DriverChannel::CHANNEL1, fr_val);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetCurrent(DriverChannel::CHANNEL2, rl_val);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetCurrent(DriverChannel::CHANNEL1, rr_val);

  pth_test_.ConfigureActivatePantherSystem();

  LoanedStateInterface fl_s_p =
    pth_test_.GetResourceManager()->claim_state_interface("fl_wheel_joint/position");
  LoanedStateInterface fr_s_p =
    pth_test_.GetResourceManager()->claim_state_interface("fr_wheel_joint/position");
  LoanedStateInterface rl_s_p =
    pth_test_.GetResourceManager()->claim_state_interface("rl_wheel_joint/position");
  LoanedStateInterface rr_s_p =
    pth_test_.GetResourceManager()->claim_state_interface("rr_wheel_joint/position");

  LoanedStateInterface fl_s_v =
    pth_test_.GetResourceManager()->claim_state_interface("fl_wheel_joint/velocity");
  LoanedStateInterface fr_s_v =
    pth_test_.GetResourceManager()->claim_state_interface("fr_wheel_joint/velocity");
  LoanedStateInterface rl_s_v =
    pth_test_.GetResourceManager()->claim_state_interface("rl_wheel_joint/velocity");
  LoanedStateInterface rr_s_v =
    pth_test_.GetResourceManager()->claim_state_interface("rr_wheel_joint/velocity");

  LoanedStateInterface fl_s_e =
    pth_test_.GetResourceManager()->claim_state_interface("fl_wheel_joint/effort");
  LoanedStateInterface fr_s_e =
    pth_test_.GetResourceManager()->claim_state_interface("fr_wheel_joint/effort");
  LoanedStateInterface rl_s_e =
    pth_test_.GetResourceManager()->claim_state_interface("rl_wheel_joint/effort");
  LoanedStateInterface rr_s_e =
    pth_test_.GetResourceManager()->claim_state_interface("rr_wheel_joint/effort");

  const auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);
  try {
    pth_test_.GetResourceManager()->read(TIME, PERIOD);
  } catch (const std::exception & e) {
    FAIL() << "Exception: " << e.what();
    return;
  }

  ASSERT_FLOAT_EQ(fl_s_p.get_value(), fl_val * kRbtqPosFbToRad);
  ASSERT_FLOAT_EQ(fr_s_p.get_value(), fr_val * kRbtqPosFbToRad);
  ASSERT_FLOAT_EQ(rl_s_p.get_value(), rl_val * kRbtqPosFbToRad);
  ASSERT_FLOAT_EQ(rr_s_p.get_value(), rr_val * kRbtqPosFbToRad);

  ASSERT_FLOAT_EQ(fl_s_v.get_value(), fl_val * kRbtqVelFbToRadPerSec);
  ASSERT_FLOAT_EQ(fr_s_v.get_value(), fr_val * kRbtqVelFbToRadPerSec);
  ASSERT_FLOAT_EQ(rl_s_v.get_value(), rl_val * kRbtqVelFbToRadPerSec);
  ASSERT_FLOAT_EQ(rr_s_v.get_value(), rr_val * kRbtqVelFbToRadPerSec);

  ASSERT_FLOAT_EQ(fl_s_e.get_value(), fl_val * kRbtqCurrentFbToNewtonMeters);
  ASSERT_FLOAT_EQ(fr_s_e.get_value(), fr_val * kRbtqCurrentFbToNewtonMeters);
  ASSERT_FLOAT_EQ(rl_s_e.get_value(), rl_val * kRbtqCurrentFbToNewtonMeters);
  ASSERT_FLOAT_EQ(rr_s_e.get_value(), rr_val * kRbtqCurrentFbToNewtonMeters);

  pth_test_.ShutdownPantherSystem();
}

TEST_F(TestPantherSystem, ReadOtherRoboteqParamsPantherSystem)
{
  using hardware_interface::LoanedStateInterface;

  const std::int16_t f_temp = 30;
  const std::int16_t r_temp = 32;
  const std::int16_t f_heatsink_temp = 31;
  const std::int16_t r_heatsink_temp = 33;
  const std::uint16_t f_volt = 400;
  const std::uint16_t r_volt = 430;
  const std::int16_t f_battery_current_1 = 10;
  const std::int16_t r_battery_current_1 = 30;
  const std::int16_t f_battery_current_2 = 30;
  const std::int16_t r_battery_current_2 = 40;

  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetTemperature(f_temp);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetTemperature(r_temp);
  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetHeatsinkTemperature(f_heatsink_temp);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetHeatsinkTemperature(r_heatsink_temp);
  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetVoltage(f_volt);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetVoltage(r_volt);
  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetBatteryCurrent1(f_battery_current_1);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetBatteryCurrent1(r_battery_current_1);
  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetBatteryCurrent2(f_battery_current_2);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetBatteryCurrent2(r_battery_current_2);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");

  pth_test_.ConfigureActivatePantherSystem();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  unsigned state_msg_count = 0;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) {
      state_msg = msg;
      ++state_msg_count;
    });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  auto simulated_time = node->get_clock()->now();
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  try {
    pth_test_.GetResourceManager()->read(simulated_time, PERIOD);
    ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
  } catch (const std::exception & e) {
    FAIL() << "Exception: " << e.what();
    return;
  }

  ASSERT_TRUE(state_msg);

  ASSERT_EQ(static_cast<std::int16_t>(state_msg->front.temperature), f_temp);
  ASSERT_EQ(static_cast<std::int16_t>(state_msg->rear.temperature), r_temp);

  ASSERT_EQ(static_cast<std::int16_t>(state_msg->front.heatsink_temperature), f_heatsink_temp);
  ASSERT_EQ(static_cast<std::int16_t>(state_msg->rear.heatsink_temperature), r_heatsink_temp);

  ASSERT_EQ(static_cast<std::uint16_t>(state_msg->front.voltage * 10.0), f_volt);
  ASSERT_EQ(static_cast<std::uint16_t>(state_msg->rear.voltage * 10.0), r_volt);

  ASSERT_EQ(
    static_cast<std::int16_t>(state_msg->front.current * 10.0),
    (f_battery_current_1 + f_battery_current_2));
  ASSERT_EQ(
    static_cast<std::int16_t>(state_msg->rear.current * 10.0),
    (r_battery_current_1 + r_battery_current_2));

  pth_test_.ShutdownPantherSystem();
}

// ENCODER DISCONNECTED
TEST_F(TestPantherSystem, EncoderDisconnectedPantherSystem)
{
  using hardware_interface::LoanedCommandInterface;
  using panther_hardware_interfaces_test::DriverChannel;
  using panther_hardware_interfaces_test::DriverScriptFlags;

  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetDriverScriptFlag(
    DriverScriptFlags::ENCODER_DISCONNECTED);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");

  pth_test_.ConfigureActivatePantherSystem();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  const auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  pth_test_.GetResourceManager()->read(TIME, PERIOD);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
  ASSERT_TRUE(state_msg->front.script_flag.encoder_disconnected);

  // writing should be blocked - error

  LoanedCommandInterface fl_c_v =
    pth_test_.GetResourceManager()->claim_command_interface("fl_wheel_joint/velocity");
  LoanedCommandInterface fr_c_v =
    pth_test_.GetResourceManager()->claim_command_interface("fr_wheel_joint/velocity");
  LoanedCommandInterface rl_c_v =
    pth_test_.GetResourceManager()->claim_command_interface("rl_wheel_joint/velocity");
  LoanedCommandInterface rr_c_v =
    pth_test_.GetResourceManager()->claim_command_interface("rr_wheel_joint/velocity");

  fl_c_v.set_value(0.1);
  fr_c_v.set_value(0.1);
  rl_c_v.set_value(0.1);
  rr_c_v.set_value(0.1);

  ASSERT_FLOAT_EQ(fl_c_v.get_value(), 0.1);
  ASSERT_FLOAT_EQ(fr_c_v.get_value(), 0.1);
  ASSERT_FLOAT_EQ(rl_c_v.get_value(), 0.1);
  ASSERT_FLOAT_EQ(rr_c_v.get_value(), 0.1);

  pth_test_.GetResourceManager()->write(TIME, PERIOD);

  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);
  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);

  pth_test_.ShutdownPantherSystem();
}

// INITIAL PROCEDURE
TEST_F(TestPantherSystem, InitialProcedureTestPantherSystem)
{
  using hardware_interface::LoanedStateInterface;
  using panther_hardware_interfaces_test::DriverChannel;

  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetRoboteqCmd(DriverChannel::CHANNEL1, 234);
  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetRoboteqCmd(DriverChannel::CHANNEL2, 32);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetRoboteqCmd(DriverChannel::CHANNEL1, 54);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetRoboteqCmd(DriverChannel::CHANNEL2, 12);

  pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetResetRoboteqScript(65);
  pth_test_.GetRoboteqsMock()->GetRearDriver()->SetResetRoboteqScript(23);

  pth_test_.ConfigureActivatePantherSystem();

  ASSERT_EQ(pth_test_.GetRoboteqsMock()->GetFrontDriver()->GetResetRoboteqScript(), 2);
  ASSERT_EQ(pth_test_.GetRoboteqsMock()->GetRearDriver()->GetResetRoboteqScript(), 2);

  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);
  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(
    pth_test_.GetRoboteqsMock()->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);

  pth_test_.ShutdownPantherSystem();
}

// ERROR HANDLING
// TODO: FIX - return code -10, but otherwise seems to work
// TEST(TestPantherSystemOthers, test_error_state)
// {
//   using panther_hardware_interfaces_test::kDefaultJoints;
//   using panther_hardware_interfaces_test::kDefaultParamMap;
//   using panther_hardware_interfaces_test::kPantherSystemName;

//   panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

//   auto param_map = kDefaultParamMap;

//   param_map["max_write_pdo_cmds_errors_count"] = "1";
//   param_map["max_read_pdo_motor_states_errors_count"] = "1";
//   param_map["max_read_pdo_driver_state_errors_count"] = "1";

//   const std::string panther_system_urdf_ = pth_test_.BuildUrdf(param_map, kDefaultJoints);
//   const float period_ = 0.01;

//   pth_test_.Start(panther_system_urdf_);

//   pth_test_.ConfigureActivatePantherSystem();

//   pth_test_.GetRoboteqsMock()->GetFrontDriver()->StopPublishing();
//   pth_test_.GetRoboteqsMock()->GetRearDriver()->StopPublishing();
//   pth_test_.GetRoboteqsMock()->GetFrontDriver()->SetOnWriteWait<std::uint8_t>(0x202C, 0, 200000);
//   pth_test_.GetRoboteqsMock()->GetRearDriver()->SetOnWriteWait<std::uint8_t>(0x202C, 0, 200000);

//   auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
//   const auto PERIOD = rclcpp::Duration::from_seconds(period_);

//   pth_test_.GetResourceManager()->read(TIME, PERIOD);
//   pth_test_.GetResourceManager()->write(TIME, PERIOD);

//   auto status_map = pth_test_.GetResourceManager()->get_components_status();
//   ASSERT_EQ(
//     status_map[kPantherSystemName].state.label(),
//     hardware_interface::lifecycle_state_names::FINALIZED);

//   pth_test_.Stop();
// }

// WRONG ORDER URDF
TEST(TestPantherSystemOthers, WrongOrderURDF)
{
  using hardware_interface::LoanedCommandInterface;

  using panther_hardware_interfaces_test::DriverChannel;

  using panther_hardware_interfaces_test::kDefaultParamMap;
  using panther_hardware_interfaces_test::kRadPerSecToRbtqCmd;

  const float period_ = 0.01;

  panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

  std::vector<std::string> joints = {
    "rr_wheel_joint", "fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint"};

  const std::string panther_system_urdf_ = pth_test_.BuildUrdf(kDefaultParamMap, joints);

  pth_test_.Start(panther_system_urdf_);

  const float fl_v = 0.1;
  const float fr_v = 0.2;
  const float rl_v = 0.3;
  const float rr_v = 0.4;

  pth_test_.ConfigureActivatePantherSystem();

  // loaned command interfaces have to be destroyed before running Stop
  {
    LoanedCommandInterface fl_c_v =
      pth_test_.GetResourceManager()->claim_command_interface("fl_wheel_joint/velocity");
    LoanedCommandInterface fr_c_v =
      pth_test_.GetResourceManager()->claim_command_interface("fr_wheel_joint/velocity");
    LoanedCommandInterface rl_c_v =
      pth_test_.GetResourceManager()->claim_command_interface("rl_wheel_joint/velocity");
    LoanedCommandInterface rr_c_v =
      pth_test_.GetResourceManager()->claim_command_interface("rr_wheel_joint/velocity");

    fl_c_v.set_value(fl_v);
    fr_c_v.set_value(fr_v);
    rl_c_v.set_value(rl_v);
    rr_c_v.set_value(rr_v);

    ASSERT_FLOAT_EQ(fl_c_v.get_value(), fl_v);
    ASSERT_FLOAT_EQ(fr_c_v.get_value(), fr_v);
    ASSERT_FLOAT_EQ(rl_c_v.get_value(), rl_v);
    ASSERT_FLOAT_EQ(rr_c_v.get_value(), rr_v);

    const auto TIME = rclcpp::Time(0);
    const auto PERIOD = rclcpp::Duration::from_seconds(period_);

    pth_test_.GetResourceManager()->write(TIME, PERIOD);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ASSERT_EQ(
      pth_test_.GetRoboteqsMock()->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2),
      static_cast<std::int32_t>(fl_v * kRadPerSecToRbtqCmd));
    ASSERT_EQ(
      pth_test_.GetRoboteqsMock()->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1),
      static_cast<std::int32_t>(fr_v * kRadPerSecToRbtqCmd));
    ASSERT_EQ(
      pth_test_.GetRoboteqsMock()->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2),
      static_cast<std::int32_t>(rl_v * kRadPerSecToRbtqCmd));
    ASSERT_EQ(
      pth_test_.GetRoboteqsMock()->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1),
      static_cast<std::int32_t>(rr_v * kRadPerSecToRbtqCmd));
  }

  pth_test_.ShutdownPantherSystem();

  pth_test_.Stop();
}

// TIMEOUT TESTS

// TODO: fix
// TEST(TestPantherSystemOthers, pdo_read_motors_states_timeout_test)
// {
//   using panther_hardware_interfaces_test::kDefaultJoints;
//   using panther_hardware_interfaces_test::kDefaultParamMap;

//   panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

//   auto param_map = kDefaultParamMap;

//   // It is necessary to set max_read_pdo_errors_count to some higher value, because
//   // adding wait time to Roboteq mock block all communication (also PDO), and PDO timeouts
//   // happen
//   param_map["pdo_motor_states_timeout_ms"] = "15";
//   param_map["max_write_pdo_cmds_errors_count"] = "100";
//   param_map["max_read_pdo_motor_states_errors_count"] = "2";
//   param_map["max_read_pdo_driver_state_errors_count"] = "2";

//   const std::string panther_system_urdf_ = pth_test_.BuildUrdf(param_map, kDefaultJoints);
//   const float period_ = 0.01;

//   pth_test_.Start(panther_system_urdf_);

//   rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");
//   pth_test_.ConfigureActivatePantherSystem();

//   panther_msgs::msg::DriverState::SharedPtr state_msg;
//   auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
//     panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
//     [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

//   std::this_thread::sleep_for(std::chrono::seconds(2));

//   auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
//   const auto PERIOD = rclcpp::Duration::from_seconds(period_);

//   pth_test_.GetResourceManager()->read(TIME, PERIOD);

//   ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
//   ASSERT_FALSE(state_msg->read_pdo_motor_states_error);
//   state_msg.reset();

//   pth_test_.GetRoboteqsMock()->GetFrontDriver()->StopPublishing();
//   pth_test_.GetRoboteqsMock()->GetRearDriver()->StopPublishing();

//   pth_test_.GetResourceManager()->write(TIME, PERIOD);

//   std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

//   TIME += PERIOD;
//   pth_test_.GetResourceManager()->read(TIME, PERIOD);

//   ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
//   ASSERT_FALSE(state_msg->read_pdo_motor_states_error);
//   state_msg.reset();

//   pth_test_.GetResourceManager()->write(TIME, PERIOD);

//   std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

//   TIME += PERIOD;
//   pth_test_.GetResourceManager()->read(TIME, PERIOD);

//   ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
//   ASSERT_TRUE(state_msg->read_pdo_motor_states_error);

//   pth_test_.ShutdownPantherSystem();

//   pth_test_.Stop();
// }

// todo E-stop tests - it will the best to add them along with GPIO, as it will change the E-stop
// procedure

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // For testing individual tests:
  // testing::GTEST_FLAG(filter) = "TestPantherSystemOthers.pdo_read_motors_states_timeout_test";

  return RUN_ALL_TESTS();
}
