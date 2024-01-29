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

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <panther_utils/test/test_utils.hpp>

#include <panther_system_test_utils.hpp>
#include <roboteq_mock.hpp>

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
TEST_F(TestPantherSystem, configure_activate_finalize_panther_system)
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

TEST_F(TestPantherSystem, configure_activate_deactivate_deconfigure_panther_system)
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
TEST_F(TestPantherSystem, write_commands_panther_system)
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

  ASSERT_EQ(
    pth_test_.GetRoboteqMock()->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<std::int32_t>(fl_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    pth_test_.GetRoboteqMock()->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<std::int32_t>(fr_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    pth_test_.GetRoboteqMock()->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<std::int32_t>(rl_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    pth_test_.GetRoboteqMock()->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<std::int32_t>(rr_v * kRadPerSecToRbtqCmd));

  pth_test_.ShutdownPantherSystem();
}

// READING
TEST_F(TestPantherSystem, read_feedback_panther_system)
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

  pth_test_.GetRoboteqMock()->front_driver_->SetPosition(DriverChannel::CHANNEL2, fl_val);
  pth_test_.GetRoboteqMock()->front_driver_->SetPosition(DriverChannel::CHANNEL1, fr_val);
  pth_test_.GetRoboteqMock()->rear_driver_->SetPosition(DriverChannel::CHANNEL2, rl_val);
  pth_test_.GetRoboteqMock()->rear_driver_->SetPosition(DriverChannel::CHANNEL1, rr_val);

  pth_test_.GetRoboteqMock()->front_driver_->SetVelocity(DriverChannel::CHANNEL2, fl_val);
  pth_test_.GetRoboteqMock()->front_driver_->SetVelocity(DriverChannel::CHANNEL1, fr_val);
  pth_test_.GetRoboteqMock()->rear_driver_->SetVelocity(DriverChannel::CHANNEL2, rl_val);
  pth_test_.GetRoboteqMock()->rear_driver_->SetVelocity(DriverChannel::CHANNEL1, rr_val);

  pth_test_.GetRoboteqMock()->front_driver_->SetCurrent(DriverChannel::CHANNEL2, fl_val);
  pth_test_.GetRoboteqMock()->front_driver_->SetCurrent(DriverChannel::CHANNEL1, fr_val);
  pth_test_.GetRoboteqMock()->rear_driver_->SetCurrent(DriverChannel::CHANNEL2, rl_val);
  pth_test_.GetRoboteqMock()->rear_driver_->SetCurrent(DriverChannel::CHANNEL1, rr_val);

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

TEST_F(TestPantherSystem, read_other_roboteq_params_panther_system)
{
  using hardware_interface::LoanedStateInterface;

  const std::int16_t f_temp = 30;
  const std::int16_t r_temp = 32;
  const std::uint16_t f_volt = 400;
  const std::uint16_t r_volt = 430;
  const std::int16_t f_battery_current_1 = 10;
  const std::int16_t r_battery_current_1 = 30;
  const std::int16_t f_battery_current_2 = 30;
  const std::int16_t r_battery_current_2 = 40;

  pth_test_.GetRoboteqMock()->front_driver_->SetTemperature(f_temp);
  pth_test_.GetRoboteqMock()->rear_driver_->SetTemperature(r_temp);
  pth_test_.GetRoboteqMock()->front_driver_->SetVoltage(f_volt);
  pth_test_.GetRoboteqMock()->rear_driver_->SetVoltage(r_volt);
  pth_test_.GetRoboteqMock()->front_driver_->SetBatteryCurrent1(f_battery_current_1);
  pth_test_.GetRoboteqMock()->rear_driver_->SetBatteryCurrent1(r_battery_current_1);
  pth_test_.GetRoboteqMock()->front_driver_->SetBatteryCurrent2(f_battery_current_2);
  pth_test_.GetRoboteqMock()->rear_driver_->SetBatteryCurrent2(r_battery_current_2);

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

  // Every read call one value is updated - has to be called 8 times to update all of them and send
  // new values
  for (int i = 0; i < 8; ++i) {
    try {
      pth_test_.GetResourceManager()->read(simulated_time, PERIOD);
      ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
    } catch (const std::exception & e) {
      FAIL() << "Exception: " << e.what();
      return;
    }

    simulated_time += PERIOD;
  }

  ASSERT_TRUE(state_msg);

  ASSERT_EQ(static_cast<std::int16_t>(state_msg->front.temperature), f_temp);
  ASSERT_EQ(static_cast<std::int16_t>(state_msg->rear.temperature), r_temp);

  ASSERT_EQ(static_cast<std::uint16_t>(state_msg->front.voltage * 10.0), f_volt);
  ASSERT_EQ(static_cast<std::uint16_t>(state_msg->rear.voltage * 10.0), r_volt);

  ASSERT_EQ(
    static_cast<std::int16_t>(state_msg->front.current * 10.0),
    (f_battery_current_1 + f_battery_current_2));
  ASSERT_EQ(
    battery_current static_cast<std::int16_t>(state_msg->rear.current * 10.0),
    (r_battery_current_1 + r_battery_current_2));

  pth_test_.ShutdownPantherSystem();
}

// ENCODER DISCONNECTED
TEST_F(TestPantherSystem, encoder_disconnected_panther_system)
{
  using hardware_interface::LoanedCommandInterface;
  using panther_hardware_interfaces_test::DriverChannel;
  using panther_hardware_interfaces_test::DriverScriptFlags;

  pth_test_.GetRoboteqMock()->front_driver_->SetDriverScriptFlag(
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
  ASSERT_TRUE(state_msg->front.script_flag.encoder_disconected);

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

  ASSERT_EQ(pth_test_.GetRoboteqMock()->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(pth_test_.GetRoboteqMock()->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);
  ASSERT_EQ(pth_test_.GetRoboteqMock()->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(pth_test_.GetRoboteqMock()->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);

  pth_test_.ShutdownPantherSystem();
}

// INITIAL PROCEDURE
TEST_F(TestPantherSystem, initial_procedure_test_panther_system)
{
  using hardware_interface::LoanedStateInterface;
  using panther_hardware_interfaces_test::DriverChannel;

  pth_test_.GetRoboteqMock()->front_driver_->SetRoboteqCmd(DriverChannel::CHANNEL1, 234);
  pth_test_.GetRoboteqMock()->front_driver_->SetRoboteqCmd(DriverChannel::CHANNEL2, 32);
  pth_test_.GetRoboteqMock()->rear_driver_->SetRoboteqCmd(DriverChannel::CHANNEL1, 54);
  pth_test_.GetRoboteqMock()->rear_driver_->SetRoboteqCmd(DriverChannel::CHANNEL2, 12);

  pth_test_.GetRoboteqMock()->front_driver_->SetResetRoboteqScript(65);
  pth_test_.GetRoboteqMock()->rear_driver_->SetResetRoboteqScript(23);

  pth_test_.ConfigureActivatePantherSystem();

  ASSERT_EQ(pth_test_.GetRoboteqMock()->front_driver_->GetResetRoboteqScript(), 2);
  ASSERT_EQ(pth_test_.GetRoboteqMock()->rear_driver_->GetResetRoboteqScript(), 2);

  ASSERT_EQ(pth_test_.GetRoboteqMock()->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(pth_test_.GetRoboteqMock()->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);
  ASSERT_EQ(pth_test_.GetRoboteqMock()->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(pth_test_.GetRoboteqMock()->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);

  pth_test_.ShutdownPantherSystem();
}

// ERROR HANDLING
TEST(TestPantherSystemOthers, test_error_state)
{
  using panther_hardware_interfaces_test::kDefaultJoints;
  using panther_hardware_interfaces_test::kDefaultParamMap;
  using panther_hardware_interfaces_test::kPantherSystemName;

  panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

  auto param_map = kDefaultParamMap;

  param_map["max_read_pdo_errors_count"] = "1";
  param_map["max_read_sdo_errors_count"] = "1";
  param_map["max_write_sdo_errors_count"] = "1";

  const std::string panther_system_urdf_ = pth_test_.BuildUrdf(param_map, kDefaultJoints);
  const float period_ = 0.01;

  pth_test_.Start(panther_system_urdf_);

  pth_test_.ConfigureActivatePantherSystem();

  pth_test_.GetRoboteqMock()->front_driver_->SetOnWriteWait<std::int32_t>(0x2000, 1, 50000);
  pth_test_.GetRoboteqMock()->rear_driver_->SetOnWriteWait<std::int32_t>(0x2000, 1, 50000);

  auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  pth_test_.GetResourceManager()->read(TIME, PERIOD);
  pth_test_.GetResourceManager()->write(TIME, PERIOD);

  auto status_map = pth_test_.GetResourceManager()->get_components_status();
  ASSERT_EQ(
    status_map[kPantherSystemName].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);

  pth_test_.Stop();
}

// WRONG ORDER URDF
TEST(TestPantherSystemOthers, wrong_order_urdf)
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

    ASSERT_EQ(
      pth_test_.GetRoboteqMock()->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
      static_cast<std::int32_t>(fl_v * kRadPerSecToRbtqCmd));
    ASSERT_EQ(
      pth_test_.GetRoboteqMock()->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
      static_cast<std::int32_t>(fr_v * kRadPerSecToRbtqCmd));
    ASSERT_EQ(
      pth_test_.GetRoboteqMock()->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
      static_cast<std::int32_t>(rl_v * kRadPerSecToRbtqCmd));
    ASSERT_EQ(
      pth_test_.GetRoboteqMock()->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
      static_cast<std::int32_t>(rr_v * kRadPerSecToRbtqCmd));
  }

  pth_test_.ShutdownPantherSystem();

  pth_test_.Stop();
}

// TIMEOUT TESTS

TEST(TestPantherSystemOthers, sdo_write_timeout_test)
{
  using panther_hardware_interfaces_test::kDefaultJoints;
  using panther_hardware_interfaces_test::kDefaultParamMap;

  panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

  auto param_map = kDefaultParamMap;

  // It is necessary to set max_read_pdo_errors_count to some higher value, because
  // adding wait time to Roboteq mock block all communication (also PDO), and PDO timeouts
  // happen
  param_map["max_read_pdo_errors_count"] = "100";
  param_map["max_read_sdo_errors_count"] = "100";
  param_map["max_write_sdo_errors_count"] = "2";
  param_map["sdo_operation_timeout_ms"] = "4";

  const std::string panther_system_urdf_ = pth_test_.BuildUrdf(param_map, kDefaultJoints);
  const float period_ = 0.01;

  pth_test_.Start(panther_system_urdf_);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");
  pth_test_.ConfigureActivatePantherSystem();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  pth_test_.GetResourceManager()->read(TIME, PERIOD);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
  ASSERT_FALSE(state_msg->write_sdo_error);

  state_msg.reset();

  pth_test_.GetRoboteqMock()->rear_driver_->SetOnWriteWait<std::int32_t>(0x2000, 1, 4500);
  pth_test_.GetResourceManager()->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.GetResourceManager()->read(TIME, PERIOD);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
  ASSERT_FALSE(state_msg->write_sdo_error);
  state_msg.reset();

  pth_test_.GetResourceManager()->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.GetResourceManager()->read(TIME, PERIOD);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
  ASSERT_TRUE(state_msg->write_sdo_error);

  pth_test_.ShutdownPantherSystem();

  pth_test_.Stop();
}

TEST(TestPantherSystemOthers, sdo_read_timeout_test)
{
  using panther_hardware_interfaces_test::kDefaultJoints;
  using panther_hardware_interfaces_test::kDefaultParamMap;

  panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

  auto param_map = kDefaultParamMap;

  // It is necessary to set max_read_pdo_errors_count to some higher value, because
  // adding wait time to Roboteq mock block all communication (also PDO), and PDO timeouts
  // happen
  param_map["max_read_pdo_errors_count"] = "100";
  param_map["max_read_sdo_errors_count"] = "2";
  param_map["max_write_sdo_errors_count"] = "100";

  const std::string panther_system_urdf_ = pth_test_.BuildUrdf(param_map, kDefaultJoints);
  const float period_ = 0.01;

  pth_test_.Start(panther_system_urdf_);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");
  pth_test_.ConfigureActivatePantherSystem();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  pth_test_.GetResourceManager()->read(TIME, PERIOD);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));

  ASSERT_FALSE(state_msg->read_sdo_error);

  state_msg.reset();

  pth_test_.GetRoboteqMock()->front_driver_->SetOnReadWait<std::int8_t>(0x210F, 1, 5001);
  pth_test_.GetRoboteqMock()->front_driver_->SetOnReadWait<std::uint16_t>(0x210D, 2, 5001);
  pth_test_.GetRoboteqMock()->front_driver_->SetOnReadWait<std::int16_t>(0x210C, 1, 5001);
  pth_test_.GetRoboteqMock()->front_driver_->SetOnReadWait<std::int16_t>(0x210C, 2, 5001);
  pth_test_.GetRoboteqMock()->rear_driver_->SetOnReadWait<std::int8_t>(0x210F, 1, 5001);
  pth_test_.GetRoboteqMock()->rear_driver_->SetOnReadWait<std::uint16_t>(0x210D, 2, 5001);
  pth_test_.GetRoboteqMock()->rear_driver_->SetOnReadWait<std::int16_t>(0x210C, 1, 5001);
  pth_test_.GetRoboteqMock()->rear_driver_->SetOnReadWait<std::int16_t>(0x210C, 2, 5001);

  pth_test_.GetResourceManager()->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.GetResourceManager()->read(TIME, PERIOD);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
  ASSERT_FALSE(state_msg->read_sdo_error);
  state_msg.reset();

  pth_test_.GetResourceManager()->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.GetResourceManager()->read(TIME, PERIOD);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
  ASSERT_TRUE(state_msg->read_sdo_error);

  pth_test_.ShutdownPantherSystem();

  pth_test_.Stop();
}

TEST(TestPantherSystemOthers, pdo_read_timeout_test)
{
  using panther_hardware_interfaces_test::kDefaultJoints;
  using panther_hardware_interfaces_test::kDefaultParamMap;

  panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

  auto param_map = kDefaultParamMap;

  // It is necessary to set max_read_pdo_errors_count to some higher value, because
  // adding wait time to Roboteq mock block all communication (also PDO), and PDO timeouts
  // happen
  param_map["pdo_feedback_timeout_ms"] = "15";
  param_map["max_read_pdo_errors_count"] = "2";
  param_map["max_read_sdo_errors_count"] = "100";
  param_map["max_write_sdo_errors_count"] = "100";

  const std::string panther_system_urdf_ = pth_test_.BuildUrdf(param_map, kDefaultJoints);
  const float period_ = 0.01;

  pth_test_.Start(panther_system_urdf_);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");
  pth_test_.ConfigureActivatePantherSystem();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    panther_hardware_interfaces_test::kMotorControllersStateTopic, rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  pth_test_.GetResourceManager()->read(TIME, PERIOD);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));

  ASSERT_FALSE(state_msg->read_pdo_error);

  state_msg.reset();

  pth_test_.GetRoboteqMock()->front_driver_->StopPublishing();
  pth_test_.GetRoboteqMock()->rear_driver_->StopPublishing();

  pth_test_.GetResourceManager()->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.GetResourceManager()->read(TIME, PERIOD);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
  ASSERT_FALSE(state_msg->read_pdo_error);
  state_msg.reset();

  pth_test_.GetResourceManager()->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.GetResourceManager()->read(TIME, PERIOD);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node, state_msg, std::chrono::seconds(5)));
  ASSERT_TRUE(state_msg->read_pdo_error);

  pth_test_.ShutdownPantherSystem();

  pth_test_.Stop();
}

// todo estop tests - it will the best to add them along with GPIO, as it will change the estop
// procedure

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // For testing individual tests:
  // testing::GTEST_FLAG(filter) = "TestPantherSystem.read_other_roboteq_params_panther_system";

  return RUN_ALL_TESTS();
}
