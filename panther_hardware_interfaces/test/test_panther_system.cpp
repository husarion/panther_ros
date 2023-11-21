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

#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <panther_system_test_utils.hpp>
#include <roboteq_mock.hpp>

class TestPantherSystem : public ::testing::Test
{
public:
  TestPantherSystem() { pth_test_.Start(pth_test_.default_panther_system_urdf_); }
  ~TestPantherSystem() { pth_test_.Stop(); }

  void CheckInterfaces()
  {
    EXPECT_EQ(pth_test_.rm_->system_components_size(), 1u);
    ASSERT_EQ(pth_test_.rm_->state_interface_keys().size(), 12u);

    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("fl_wheel_joint/position"));
    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("fr_wheel_joint/position"));
    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("rl_wheel_joint/position"));
    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("rr_wheel_joint/position"));

    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("fl_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("fr_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("rl_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("rr_wheel_joint/velocity"));

    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("fl_wheel_joint/effort"));
    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("fr_wheel_joint/effort"));
    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("rl_wheel_joint/effort"));
    EXPECT_TRUE(pth_test_.rm_->state_interface_exists("rr_wheel_joint/effort"));

    ASSERT_EQ(pth_test_.rm_->command_interface_keys().size(), 4u);

    EXPECT_TRUE(pth_test_.rm_->command_interface_exists("fl_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.rm_->command_interface_exists("fr_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.rm_->command_interface_exists("rl_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.rm_->command_interface_exists("rr_wheel_joint/velocity"));
  }

  void CheckInitialValues()
  {
    using hardware_interface::LoanedCommandInterface;
    using hardware_interface::LoanedStateInterface;

    LoanedStateInterface fl_s_p = pth_test_.rm_->claim_state_interface("fl_wheel_joint/position");
    LoanedStateInterface fr_s_p = pth_test_.rm_->claim_state_interface("fr_wheel_joint/position");
    LoanedStateInterface rl_s_p = pth_test_.rm_->claim_state_interface("rl_wheel_joint/position");
    LoanedStateInterface rr_s_p = pth_test_.rm_->claim_state_interface("rr_wheel_joint/position");

    LoanedStateInterface fl_s_v = pth_test_.rm_->claim_state_interface("fl_wheel_joint/velocity");
    LoanedStateInterface fr_s_v = pth_test_.rm_->claim_state_interface("fr_wheel_joint/velocity");
    LoanedStateInterface rl_s_v = pth_test_.rm_->claim_state_interface("rl_wheel_joint/velocity");
    LoanedStateInterface rr_s_v = pth_test_.rm_->claim_state_interface("rr_wheel_joint/velocity");

    LoanedStateInterface fl_s_e = pth_test_.rm_->claim_state_interface("fl_wheel_joint/effort");
    LoanedStateInterface fr_s_e = pth_test_.rm_->claim_state_interface("fr_wheel_joint/effort");
    LoanedStateInterface rl_s_e = pth_test_.rm_->claim_state_interface("rl_wheel_joint/effort");
    LoanedStateInterface rr_s_e = pth_test_.rm_->claim_state_interface("rr_wheel_joint/effort");

    LoanedCommandInterface fl_c_v =
      pth_test_.rm_->claim_command_interface("fl_wheel_joint/velocity");
    LoanedCommandInterface fr_c_v =
      pth_test_.rm_->claim_command_interface("fr_wheel_joint/velocity");
    LoanedCommandInterface rl_c_v =
      pth_test_.rm_->claim_command_interface("rl_wheel_joint/velocity");
    LoanedCommandInterface rr_c_v =
      pth_test_.rm_->claim_command_interface("rr_wheel_joint/velocity");

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

void WaitForDriverStateMsg(
  rclcpp::Node::SharedPtr node, panther_msgs::msg::DriverState::SharedPtr state_msg)
{
  rclcpp::Time start = node->now();
  while (node->now() - start < rclcpp::Duration(std::chrono::seconds(5))) {
    rclcpp::spin_some(node);
    if (state_msg) {
      break;
    }
  }
}

// TRANSITIONS
TEST_F(TestPantherSystem, configure_activate_finalize_panther_system)
{
  // check if hardware is configured
  auto status_map = pth_test_.rm_->get_components_status();
  ASSERT_EQ(
    status_map[pth_test_.panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);

  try {
    pth_test_.ConfigurePantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to ConfigurePantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.rm_->get_components_status();
  ASSERT_EQ(
    status_map[pth_test_.panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    pth_test_.ActivatePantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to ActivatePantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.rm_->get_components_status();
  ASSERT_EQ(
    status_map[pth_test_.panther_system_name_].state.label(),
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
  status_map = pth_test_.rm_->get_components_status();
  ASSERT_EQ(
    status_map[pth_test_.panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::FINALIZED);
}

TEST_F(TestPantherSystem, configure_activate_deactivate_deConfigurePantherSystem)
{
  auto status_map = pth_test_.rm_->get_components_status();
  ASSERT_EQ(
    status_map[pth_test_.panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);

  try {
    pth_test_.ConfigurePantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to ConfigurePantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.rm_->get_components_status();
  ASSERT_EQ(
    status_map[pth_test_.panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    pth_test_.ActivatePantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to ActivatePantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.rm_->get_components_status();
  ASSERT_EQ(
    status_map[pth_test_.panther_system_name_].state.label(),
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
  status_map = pth_test_.rm_->get_components_status();
  ASSERT_EQ(
    status_map[pth_test_.panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    pth_test_.UnconfigurePantherSystem();
  } catch (const std::exception & e) {
    FAIL() << "Exception caught when trying to UnconfigurePantherSystem: " << e.what();
    return;
  }
  status_map = pth_test_.rm_->get_components_status();
  ASSERT_EQ(
    status_map[pth_test_.panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

// WRITING
TEST_F(TestPantherSystem, write_commands_panther_system)
{
  using panther_hardware_interfaces_test::DriverChannel;

  using hardware_interface::LoanedCommandInterface;

  const float fl_v = 0.1;
  const float fr_v = 0.2;
  const float rl_v = 0.3;
  const float rr_v = 0.4;

  pth_test_.ConfigureActivatePantherSystem();

  LoanedCommandInterface fl_c_v = pth_test_.rm_->claim_command_interface("fl_wheel_joint/velocity");
  LoanedCommandInterface fr_c_v = pth_test_.rm_->claim_command_interface("fr_wheel_joint/velocity");
  LoanedCommandInterface rl_c_v = pth_test_.rm_->claim_command_interface("rl_wheel_joint/velocity");
  LoanedCommandInterface rr_c_v = pth_test_.rm_->claim_command_interface("rr_wheel_joint/velocity");

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

  pth_test_.rm_->write(TIME, PERIOD);

  ASSERT_EQ(
    pth_test_.roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<int32_t>(fl_v * pth_test_.rad_per_sec_to_rbtq_cmd_));
  ASSERT_EQ(
    pth_test_.roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<int32_t>(fr_v * pth_test_.rad_per_sec_to_rbtq_cmd_));
  ASSERT_EQ(
    pth_test_.roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<int32_t>(rl_v * pth_test_.rad_per_sec_to_rbtq_cmd_));
  ASSERT_EQ(
    pth_test_.roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<int32_t>(rr_v * pth_test_.rad_per_sec_to_rbtq_cmd_));

  pth_test_.ShutdownPantherSystem();
}

// READING
TEST_F(TestPantherSystem, read_feedback_panther_system)
{
  using hardware_interface::LoanedStateInterface;
  using panther_hardware_interfaces_test::DriverChannel;

  const int32_t fl_val = 100;
  const int32_t fr_val = 200;
  const int32_t rl_val = 300;
  const int32_t rr_val = 400;

  pth_test_.roboteq_mock_->front_driver_->SetPosition(DriverChannel::CHANNEL2, fl_val);
  pth_test_.roboteq_mock_->front_driver_->SetPosition(DriverChannel::CHANNEL1, fr_val);
  pth_test_.roboteq_mock_->rear_driver_->SetPosition(DriverChannel::CHANNEL2, rl_val);
  pth_test_.roboteq_mock_->rear_driver_->SetPosition(DriverChannel::CHANNEL1, rr_val);

  pth_test_.roboteq_mock_->front_driver_->SetVelocity(DriverChannel::CHANNEL2, fl_val);
  pth_test_.roboteq_mock_->front_driver_->SetVelocity(DriverChannel::CHANNEL1, fr_val);
  pth_test_.roboteq_mock_->rear_driver_->SetVelocity(DriverChannel::CHANNEL2, rl_val);
  pth_test_.roboteq_mock_->rear_driver_->SetVelocity(DriverChannel::CHANNEL1, rr_val);

  pth_test_.roboteq_mock_->front_driver_->SetCurrent(DriverChannel::CHANNEL2, fl_val);
  pth_test_.roboteq_mock_->front_driver_->SetCurrent(DriverChannel::CHANNEL1, fr_val);
  pth_test_.roboteq_mock_->rear_driver_->SetCurrent(DriverChannel::CHANNEL2, rl_val);
  pth_test_.roboteq_mock_->rear_driver_->SetCurrent(DriverChannel::CHANNEL1, rr_val);

  pth_test_.ConfigureActivatePantherSystem();

  LoanedStateInterface fl_s_p = pth_test_.rm_->claim_state_interface("fl_wheel_joint/position");
  LoanedStateInterface fr_s_p = pth_test_.rm_->claim_state_interface("fr_wheel_joint/position");
  LoanedStateInterface rl_s_p = pth_test_.rm_->claim_state_interface("rl_wheel_joint/position");
  LoanedStateInterface rr_s_p = pth_test_.rm_->claim_state_interface("rr_wheel_joint/position");

  LoanedStateInterface fl_s_v = pth_test_.rm_->claim_state_interface("fl_wheel_joint/velocity");
  LoanedStateInterface fr_s_v = pth_test_.rm_->claim_state_interface("fr_wheel_joint/velocity");
  LoanedStateInterface rl_s_v = pth_test_.rm_->claim_state_interface("rl_wheel_joint/velocity");
  LoanedStateInterface rr_s_v = pth_test_.rm_->claim_state_interface("rr_wheel_joint/velocity");

  LoanedStateInterface fl_s_e = pth_test_.rm_->claim_state_interface("fl_wheel_joint/effort");
  LoanedStateInterface fr_s_e = pth_test_.rm_->claim_state_interface("fr_wheel_joint/effort");
  LoanedStateInterface rl_s_e = pth_test_.rm_->claim_state_interface("rl_wheel_joint/effort");
  LoanedStateInterface rr_s_e = pth_test_.rm_->claim_state_interface("rr_wheel_joint/effort");

  const auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);
  try {
    pth_test_.rm_->read(TIME, PERIOD);
  } catch (const std::exception & e) {
    FAIL() << "Exception: " << e.what();
    return;
  }

  ASSERT_FLOAT_EQ(fl_s_p.get_value(), fl_val * pth_test_.rbtq_pos_fb_to_rad_);
  ASSERT_FLOAT_EQ(fr_s_p.get_value(), fr_val * pth_test_.rbtq_pos_fb_to_rad_);
  ASSERT_FLOAT_EQ(rl_s_p.get_value(), rl_val * pth_test_.rbtq_pos_fb_to_rad_);
  ASSERT_FLOAT_EQ(rr_s_p.get_value(), rr_val * pth_test_.rbtq_pos_fb_to_rad_);

  ASSERT_FLOAT_EQ(fl_s_v.get_value(), fl_val * pth_test_.rbtq_vel_fb_to_rad_per_sec_);
  ASSERT_FLOAT_EQ(fr_s_v.get_value(), fr_val * pth_test_.rbtq_vel_fb_to_rad_per_sec_);
  ASSERT_FLOAT_EQ(rl_s_v.get_value(), rl_val * pth_test_.rbtq_vel_fb_to_rad_per_sec_);
  ASSERT_FLOAT_EQ(rr_s_v.get_value(), rr_val * pth_test_.rbtq_vel_fb_to_rad_per_sec_);

  ASSERT_FLOAT_EQ(fl_s_e.get_value(), fl_val * pth_test_.rbtq_current_fb_to_newton_meters_);
  ASSERT_FLOAT_EQ(fr_s_e.get_value(), fr_val * pth_test_.rbtq_current_fb_to_newton_meters_);
  ASSERT_FLOAT_EQ(rl_s_e.get_value(), rl_val * pth_test_.rbtq_current_fb_to_newton_meters_);
  ASSERT_FLOAT_EQ(rr_s_e.get_value(), rr_val * pth_test_.rbtq_current_fb_to_newton_meters_);

  pth_test_.ShutdownPantherSystem();
}

TEST_F(TestPantherSystem, read_other_roboteq_params_panther_system)
{
  using hardware_interface::LoanedStateInterface;

  const int16_t f_temp = 30;
  const int16_t r_temp = 32;
  const uint16_t f_volt = 400;
  const uint16_t r_volt = 430;
  const int16_t f_bat_amps_1 = 10;
  const int16_t r_bat_amps_1 = 30;
  const int16_t f_bat_amps_2 = 30;
  const int16_t r_bat_amps_2 = 40;

  pth_test_.roboteq_mock_->front_driver_->SetTemperature(f_temp);
  pth_test_.roboteq_mock_->rear_driver_->SetTemperature(r_temp);
  pth_test_.roboteq_mock_->front_driver_->SetVoltage(f_volt);
  pth_test_.roboteq_mock_->rear_driver_->SetVoltage(r_volt);
  pth_test_.roboteq_mock_->front_driver_->SetBatAmps1(f_bat_amps_1);
  pth_test_.roboteq_mock_->rear_driver_->SetBatAmps1(r_bat_amps_1);
  pth_test_.roboteq_mock_->front_driver_->SetBatAmps2(f_bat_amps_2);
  pth_test_.roboteq_mock_->rear_driver_->SetBatAmps2(r_bat_amps_2);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");

  pth_test_.ConfigureActivatePantherSystem();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  unsigned state_msg_count = 0;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    "/panther_system_node/driver/motor_controllers_state", rclcpp::SensorDataQoS(),
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
      pth_test_.rm_->read(simulated_time, PERIOD);
    } catch (const std::exception & e) {
      FAIL() << "Exception: " << e.what();
      return;
    }

    simulated_time += PERIOD;
  }

  // TODO
  rclcpp::Time start = node->now();
  while (node->now() - start < rclcpp::Duration(std::chrono::seconds(5))) {
    rclcpp::spin_some(node);
    if (state_msg && state_msg_count == 8) {
      break;
    }
  }

  sub.reset();

  ASSERT_TRUE(state_msg);

  ASSERT_EQ(static_cast<int16_t>(state_msg->front.temperature), f_temp);
  ASSERT_EQ(static_cast<int16_t>(state_msg->rear.temperature), r_temp);

  ASSERT_EQ(static_cast<uint16_t>(state_msg->front.voltage * 10.0), f_volt);
  ASSERT_EQ(static_cast<uint16_t>(state_msg->rear.voltage * 10.0), r_volt);

  ASSERT_EQ(static_cast<int16_t>(state_msg->front.current * 10.0), (f_bat_amps_1 + f_bat_amps_2));
  ASSERT_EQ(static_cast<int16_t>(state_msg->rear.current * 10.0), (r_bat_amps_1 + r_bat_amps_2));

  pth_test_.ShutdownPantherSystem();
}

// ENCODER DISCONNECTED
TEST_F(TestPantherSystem, encoder_disconnected_panther_system)
{
  using hardware_interface::LoanedCommandInterface;
  using panther_hardware_interfaces_test::DriverChannel;
  using panther_hardware_interfaces_test::DriverScriptFlags;

  pth_test_.roboteq_mock_->front_driver_->SetDriverScriptFlag(
    DriverScriptFlags::ENCODER_DISCONNECTED);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");

  pth_test_.ConfigureActivatePantherSystem();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    "/panther_system_node/driver/motor_controllers_state", rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  const auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  pth_test_.rm_->read(TIME, PERIOD);

  WaitForDriverStateMsg(node, state_msg);
  ASSERT_TRUE(state_msg->front.script_flag.encoder_disconected);

  // writing should be blocked - error

  LoanedCommandInterface fl_c_v = pth_test_.rm_->claim_command_interface("fl_wheel_joint/velocity");
  LoanedCommandInterface fr_c_v = pth_test_.rm_->claim_command_interface("fr_wheel_joint/velocity");
  LoanedCommandInterface rl_c_v = pth_test_.rm_->claim_command_interface("rl_wheel_joint/velocity");
  LoanedCommandInterface rr_c_v = pth_test_.rm_->claim_command_interface("rr_wheel_joint/velocity");

  fl_c_v.set_value(0.1);
  fr_c_v.set_value(0.1);
  rl_c_v.set_value(0.1);
  rr_c_v.set_value(0.1);

  ASSERT_FLOAT_EQ(fl_c_v.get_value(), 0.1);
  ASSERT_FLOAT_EQ(fr_c_v.get_value(), 0.1);
  ASSERT_FLOAT_EQ(rl_c_v.get_value(), 0.1);
  ASSERT_FLOAT_EQ(rr_c_v.get_value(), 0.1);

  pth_test_.rm_->write(TIME, PERIOD);

  ASSERT_EQ(pth_test_.roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(pth_test_.roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);
  ASSERT_EQ(pth_test_.roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(pth_test_.roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);

  pth_test_.ShutdownPantherSystem();
}

// INITIAL PROCEDURE
TEST_F(TestPantherSystem, initial_procedure_test_panther_system)
{
  using hardware_interface::LoanedStateInterface;
  using panther_hardware_interfaces_test::DriverChannel;

  pth_test_.roboteq_mock_->front_driver_->SetRoboteqCmd(DriverChannel::CHANNEL1, 234);
  pth_test_.roboteq_mock_->front_driver_->SetRoboteqCmd(DriverChannel::CHANNEL2, 32);
  pth_test_.roboteq_mock_->rear_driver_->SetRoboteqCmd(DriverChannel::CHANNEL1, 54);
  pth_test_.roboteq_mock_->rear_driver_->SetRoboteqCmd(DriverChannel::CHANNEL2, 12);

  pth_test_.roboteq_mock_->front_driver_->SetResetRoboteqScript(65);
  pth_test_.roboteq_mock_->rear_driver_->SetResetRoboteqScript(23);

  pth_test_.ConfigureActivatePantherSystem();

  // TODO check timing

  ASSERT_EQ(pth_test_.roboteq_mock_->front_driver_->GetResetRoboteqScript(), 2);
  ASSERT_EQ(pth_test_.roboteq_mock_->rear_driver_->GetResetRoboteqScript(), 2);

  ASSERT_EQ(pth_test_.roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(pth_test_.roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);
  ASSERT_EQ(pth_test_.roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(pth_test_.roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);

  pth_test_.ShutdownPantherSystem();
}

// ERROR HANDLING
TEST(TestPantherSystemOthers, test_error_state)
{
  panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

  pth_test_.param_map_["max_read_pdo_errors_count"] = "1";
  pth_test_.param_map_["max_read_sdo_errors_count"] = "1";
  pth_test_.param_map_["max_write_sdo_errors_count"] = "1";

  const std::string panther_system_urdf_ = pth_test_.BuildUrdf(
    pth_test_.param_map_, pth_test_.joints_);
  const float period_ = 0.01;

  pth_test_.Start(panther_system_urdf_);

  pth_test_.ConfigureActivatePantherSystem();

  pth_test_.roboteq_mock_->front_driver_->SetOnWriteWait<int32_t>(0x2000, 1, 50000);
  pth_test_.roboteq_mock_->rear_driver_->SetOnWriteWait<int32_t>(0x2000, 1, 50000);

  auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  pth_test_.rm_->read(TIME, PERIOD);
  pth_test_.rm_->write(TIME, PERIOD);

  TIME += PERIOD;

  pth_test_.rm_->read(TIME, PERIOD);
  pth_test_.rm_->write(TIME, PERIOD);

  auto status_map = pth_test_.rm_->get_components_status();
  ASSERT_EQ(
    status_map[pth_test_.panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);

  pth_test_.Stop();
}

// WRONG ORDER URDF
TEST(TestPantherSystemOthers, wrong_order_urdf)
{
  using hardware_interface::LoanedCommandInterface;
  using panther_hardware_interfaces_test::DriverChannel;

  const float period_ = 0.01;

  panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

  std::vector<std::string> joints = {
    "rr_wheel_joint", "fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint"};

  const std::string panther_system_urdf_ = pth_test_.BuildUrdf(pth_test_.param_map_, joints);

  pth_test_.Start(panther_system_urdf_);

  const float fl_v = 0.1;
  const float fr_v = 0.2;
  const float rl_v = 0.3;
  const float rr_v = 0.4;

  pth_test_.ConfigureActivatePantherSystem();

  // loaned command interfaces have to be destroyed before running Stop
  {
    LoanedCommandInterface fl_c_v =
      pth_test_.rm_->claim_command_interface("fl_wheel_joint/velocity");
    LoanedCommandInterface fr_c_v =
      pth_test_.rm_->claim_command_interface("fr_wheel_joint/velocity");
    LoanedCommandInterface rl_c_v =
      pth_test_.rm_->claim_command_interface("rl_wheel_joint/velocity");
    LoanedCommandInterface rr_c_v =
      pth_test_.rm_->claim_command_interface("rr_wheel_joint/velocity");

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

    pth_test_.rm_->write(TIME, PERIOD);

    ASSERT_EQ(
      pth_test_.roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
      static_cast<int32_t>(fl_v * pth_test_.rad_per_sec_to_rbtq_cmd_));
    ASSERT_EQ(
      pth_test_.roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
      static_cast<int32_t>(fr_v * pth_test_.rad_per_sec_to_rbtq_cmd_));
    ASSERT_EQ(
      pth_test_.roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
      static_cast<int32_t>(rl_v * pth_test_.rad_per_sec_to_rbtq_cmd_));
    ASSERT_EQ(
      pth_test_.roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
      static_cast<int32_t>(rr_v * pth_test_.rad_per_sec_to_rbtq_cmd_));
  }

  pth_test_.ShutdownPantherSystem();

  pth_test_.Stop();
}

// TIMEOUT TESTS

TEST(TestPantherSystemOthers, sdo_write_timeout_test)
{
  panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

  // It is necessary to set max_read_pdo_errors_count to some higher value, because
  // adding wait time to Roboteq mock block all communication (also PDO), and PDO timeouts
  // happen
  pth_test_.param_map_["max_read_pdo_errors_count"] = "100";
  pth_test_.param_map_["max_read_sdo_errors_count"] = "100";
  pth_test_.param_map_["max_write_sdo_errors_count"] = "2";

  const std::string panther_system_urdf_ = pth_test_.BuildUrdf(
    pth_test_.param_map_, pth_test_.joints_);
  const float period_ = 0.01;

  pth_test_.Start(panther_system_urdf_);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");
  pth_test_.ConfigureActivatePantherSystem();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    "/panther_system_node/driver/motor_controllers_state", rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  pth_test_.rm_->read(TIME, PERIOD);

  WaitForDriverStateMsg(node, state_msg);

  ASSERT_FALSE(state_msg->write_sdo_error);

  state_msg.reset();

  // More than sdo_operation_wait_timeout_
  pth_test_.roboteq_mock_->front_driver_->SetOnWriteWait<int32_t>(0x2000, 1, 5001);
  pth_test_.rm_->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.rm_->read(TIME, PERIOD);

  WaitForDriverStateMsg(node, state_msg);
  ASSERT_FALSE(state_msg->write_sdo_error);
  state_msg.reset();

  pth_test_.rm_->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.rm_->read(TIME, PERIOD);

  WaitForDriverStateMsg(node, state_msg);
  ASSERT_TRUE(state_msg->write_sdo_error);

  pth_test_.ShutdownPantherSystem();

  pth_test_.Stop();
}

TEST(TestPantherSystemOthers, sdo_read_timeout_test)
{
  panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

  // It is necessary to set max_read_pdo_errors_count to some higher value, because
  // adding wait time to Roboteq mock block all communication (also PDO), and PDO timeouts
  // happen
  pth_test_.param_map_["max_read_pdo_errors_count"] = "100";
  pth_test_.param_map_["max_read_sdo_errors_count"] = "2";
  pth_test_.param_map_["max_write_sdo_errors_count"] = "100";

  const std::string panther_system_urdf_ = pth_test_.BuildUrdf(
    pth_test_.param_map_, pth_test_.joints_);
  const float period_ = 0.01;

  pth_test_.Start(panther_system_urdf_);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");
  pth_test_.ConfigureActivatePantherSystem();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    "/panther_system_node/driver/motor_controllers_state", rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  pth_test_.rm_->read(TIME, PERIOD);

  WaitForDriverStateMsg(node, state_msg);

  ASSERT_FALSE(state_msg->read_sdo_error);

  state_msg.reset();

  // More than sdo_operation_wait_timeout_
  pth_test_.roboteq_mock_->front_driver_->SetOnReadWait<int8_t>(0x210F, 1, 5001);
  pth_test_.roboteq_mock_->front_driver_->SetOnReadWait<uint16_t>(0x210D, 2, 5001);
  pth_test_.roboteq_mock_->front_driver_->SetOnReadWait<int16_t>(0x210C, 1, 5001);
  pth_test_.roboteq_mock_->front_driver_->SetOnReadWait<int16_t>(0x210C, 2, 5001);
  pth_test_.roboteq_mock_->rear_driver_->SetOnReadWait<int8_t>(0x210F, 1, 5001);
  pth_test_.roboteq_mock_->rear_driver_->SetOnReadWait<uint16_t>(0x210D, 2, 5001);
  pth_test_.roboteq_mock_->rear_driver_->SetOnReadWait<int16_t>(0x210C, 1, 5001);
  pth_test_.roboteq_mock_->rear_driver_->SetOnReadWait<int16_t>(0x210C, 2, 5001);

  pth_test_.rm_->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.rm_->read(TIME, PERIOD);

  WaitForDriverStateMsg(node, state_msg);
  ASSERT_FALSE(state_msg->read_sdo_error);
  state_msg.reset();

  pth_test_.rm_->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.rm_->read(TIME, PERIOD);

  WaitForDriverStateMsg(node, state_msg);
  ASSERT_TRUE(state_msg->read_sdo_error);

  pth_test_.ShutdownPantherSystem();

  pth_test_.Stop();
}

TEST(TestPantherSystemOthers, pdo_read_timeout_test)
{
  panther_hardware_interfaces_test::PantherSystemTestUtils pth_test_;

  // It is necessary to set max_read_pdo_errors_count to some higher value, because
  // adding wait time to Roboteq mock block all communication (also PDO), and PDO timeouts
  // happen
  pth_test_.param_map_["pdo_feedback_timeout"] = "15";
  pth_test_.param_map_["max_read_pdo_errors_count"] = "2";
  pth_test_.param_map_["max_read_sdo_errors_count"] = "100";
  pth_test_.param_map_["max_write_sdo_errors_count"] = "100";

  const std::string panther_system_urdf_ = pth_test_.BuildUrdf(
    pth_test_.param_map_, pth_test_.joints_);
  const float period_ = 0.01;

  pth_test_.Start(panther_system_urdf_);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");
  pth_test_.ConfigureActivatePantherSystem();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    "/panther_system_node/driver/motor_controllers_state", rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  pth_test_.rm_->read(TIME, PERIOD);

  WaitForDriverStateMsg(node, state_msg);

  ASSERT_FALSE(state_msg->read_pdo_error);

  state_msg.reset();

  pth_test_.roboteq_mock_->front_driver_->StopPublishing();
  pth_test_.roboteq_mock_->rear_driver_->StopPublishing();

  pth_test_.rm_->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.rm_->read(TIME, PERIOD);

  WaitForDriverStateMsg(node, state_msg);
  ASSERT_FALSE(state_msg->read_pdo_error);
  state_msg.reset();

  pth_test_.rm_->write(TIME, PERIOD);

  std::this_thread::sleep_for(PERIOD.to_chrono<std::chrono::milliseconds>());

  TIME += PERIOD;
  pth_test_.rm_->read(TIME, PERIOD);

  WaitForDriverStateMsg(node, state_msg);
  ASSERT_TRUE(state_msg->read_pdo_error);

  pth_test_.ShutdownPantherSystem();

  pth_test_.Stop();
}

// TODO estop tests - it will the best to add them along with GPIO, as it will change the estop
// procedure

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // For testing individual tests:
  // testing::GTEST_FLAG(filter) =
  // "TestPantherSystemOthers.test_error_state";

  return RUN_ALL_TESTS();
}
