#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <mock_roboteq.hpp>
#include <test_utils.hpp>

// TRANSITIONS
TEST_F(TestPantherSystem, configure_activate_finalize_panther_system)
{
  // check if hardware is configured
  auto status_map = rm_->get_components_status();
  ASSERT_EQ(
    status_map[panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);

  try {
    configure_panther_system();
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to configure_panther_system: " << err.what();
    return;
  }
  status_map = rm_->get_components_status();
  ASSERT_EQ(
    status_map[panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    activate_panther_system();
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to activate_panther_system: " << err.what();
    return;
  }
  status_map = rm_->get_components_status();
  ASSERT_EQ(
    status_map[panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::ACTIVE);

  // Check interfaces
  check_interfaces();

  // Check initial values
  check_initial_values();

  try {
    shutdown_panther_system();
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to shutdown_panther_system: " << err.what();
    return;
  }
  status_map = rm_->get_components_status();
  ASSERT_EQ(
    status_map[panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::FINALIZED);
}

TEST_F(TestPantherSystem, configure_activate_deactivate_deconfigure_panther_system)
{
  auto status_map = rm_->get_components_status();
  ASSERT_EQ(
    status_map[panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);

  try {
    configure_panther_system();
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to configure_panther_system: " << err.what();
    return;
  }
  status_map = rm_->get_components_status();
  ASSERT_EQ(
    status_map[panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    activate_panther_system();
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to activate_panther_system: " << err.what();
    return;
  }
  status_map = rm_->get_components_status();
  ASSERT_EQ(
    status_map[panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::ACTIVE);

  // Check interfaces
  check_interfaces();

  // Check initial values
  check_initial_values();

  try {
    deactivate_panther_system();
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to deactivate_panther_system: " << err.what();
    return;
  }
  status_map = rm_->get_components_status();
  ASSERT_EQ(
    status_map[panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    unconfigure_panther_system();
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to unconfigure_panther_system: " << err.what();
    return;
  }
  status_map = rm_->get_components_status();
  ASSERT_EQ(
    status_map[panther_system_name_].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

// WRITING
TEST_F(TestPantherSystem, write_commands_panther_system)
{
  using hardware_interface::LoanedCommandInterface;

  const double fl_v = 0.1;
  const double fr_v = 0.2;
  const double rl_v = 0.3;
  const double rr_v = 0.4;

  configure_panther_system();
  activate_panther_system();

  LoanedCommandInterface fl_c_v = rm_->claim_command_interface("fl_wheel_joint/velocity");
  LoanedCommandInterface fr_c_v = rm_->claim_command_interface("fr_wheel_joint/velocity");
  LoanedCommandInterface rl_c_v = rm_->claim_command_interface("rl_wheel_joint/velocity");
  LoanedCommandInterface rr_c_v = rm_->claim_command_interface("rr_wheel_joint/velocity");

  fl_c_v.set_value(fl_v);
  fr_c_v.set_value(fr_v);
  rl_c_v.set_value(rl_v);
  rr_c_v.set_value(rr_v);

  ASSERT_EQ(fl_v, fl_c_v.get_value());
  ASSERT_EQ(fr_v, fr_c_v.get_value());
  ASSERT_EQ(rl_v, rl_c_v.get_value());
  ASSERT_EQ(rr_v, rr_c_v.get_value());

  const auto TIME = rclcpp::Time(0);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  rm_->write(TIME, PERIOD);

  ASSERT_EQ(
    roboteq_mock_->front_driver_->GetRoboteqCmd(1), int32_t(fl_v * rad_per_sec_to_rbtq_cmd_));
  ASSERT_EQ(
    roboteq_mock_->front_driver_->GetRoboteqCmd(2), int32_t(fr_v * rad_per_sec_to_rbtq_cmd_));
  ASSERT_EQ(
    roboteq_mock_->rear_driver_->GetRoboteqCmd(1), int32_t(rl_v * rad_per_sec_to_rbtq_cmd_));
  ASSERT_EQ(
    roboteq_mock_->rear_driver_->GetRoboteqCmd(2), int32_t(rr_v * rad_per_sec_to_rbtq_cmd_));

  shutdown_panther_system();
}

// READING
TEST_F(TestPantherSystem, read_feedback_panther_system)
{
  using hardware_interface::LoanedStateInterface;

  const int32_t fl_val = 100;
  const int32_t fr_val = 200;
  const int32_t rl_val = 300;
  const int32_t rr_val = 400;

  roboteq_mock_->front_driver_->SetPosition(1, fl_val);
  roboteq_mock_->front_driver_->SetPosition(2, fr_val);
  roboteq_mock_->rear_driver_->SetPosition(1, rl_val);
  roboteq_mock_->rear_driver_->SetPosition(2, rr_val);

  roboteq_mock_->front_driver_->SetVelocity(1, fl_val);
  roboteq_mock_->front_driver_->SetVelocity(2, fr_val);
  roboteq_mock_->rear_driver_->SetVelocity(1, rl_val);
  roboteq_mock_->rear_driver_->SetVelocity(2, rr_val);

  roboteq_mock_->front_driver_->SetCurrent(1, fl_val);
  roboteq_mock_->front_driver_->SetCurrent(2, fr_val);
  roboteq_mock_->rear_driver_->SetCurrent(1, rl_val);
  roboteq_mock_->rear_driver_->SetCurrent(2, rr_val);

  configure_panther_system();
  activate_panther_system();

  LoanedStateInterface fl_s_p = rm_->claim_state_interface("fl_wheel_joint/position");
  LoanedStateInterface fr_s_p = rm_->claim_state_interface("fr_wheel_joint/position");
  LoanedStateInterface rl_s_p = rm_->claim_state_interface("rl_wheel_joint/position");
  LoanedStateInterface rr_s_p = rm_->claim_state_interface("rr_wheel_joint/position");

  LoanedStateInterface fl_s_v = rm_->claim_state_interface("fl_wheel_joint/velocity");
  LoanedStateInterface fr_s_v = rm_->claim_state_interface("fr_wheel_joint/velocity");
  LoanedStateInterface rl_s_v = rm_->claim_state_interface("rl_wheel_joint/velocity");
  LoanedStateInterface rr_s_v = rm_->claim_state_interface("rr_wheel_joint/velocity");

  LoanedStateInterface fl_s_e = rm_->claim_state_interface("fl_wheel_joint/effort");
  LoanedStateInterface fr_s_e = rm_->claim_state_interface("fr_wheel_joint/effort");
  LoanedStateInterface rl_s_e = rm_->claim_state_interface("rl_wheel_joint/effort");
  LoanedStateInterface rr_s_e = rm_->claim_state_interface("rr_wheel_joint/effort");

  const auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);
  try {
    rm_->read(TIME, PERIOD);
  } catch (std::exception & err) {
    FAIL() << "Exception: " << err.what();
    return;
  }

  // TODO channel order
  ASSERT_NEAR(fr_s_p.get_value(), fl_val * rbtq_pos_fb_to_rad_, assert_near_abs_error_);
  ASSERT_NEAR(fl_s_p.get_value(), fr_val * rbtq_pos_fb_to_rad_, assert_near_abs_error_);
  ASSERT_NEAR(rr_s_p.get_value(), rl_val * rbtq_pos_fb_to_rad_, assert_near_abs_error_);
  ASSERT_NEAR(rl_s_p.get_value(), rr_val * rbtq_pos_fb_to_rad_, assert_near_abs_error_);

  ASSERT_NEAR(fr_s_v.get_value(), fl_val * rbtq_vel_fb_to_rad_per_sec_, assert_near_abs_error_);
  ASSERT_NEAR(fl_s_v.get_value(), fr_val * rbtq_vel_fb_to_rad_per_sec_, assert_near_abs_error_);
  ASSERT_NEAR(rr_s_v.get_value(), rl_val * rbtq_vel_fb_to_rad_per_sec_, assert_near_abs_error_);
  ASSERT_NEAR(rl_s_v.get_value(), rr_val * rbtq_vel_fb_to_rad_per_sec_, assert_near_abs_error_);

  ASSERT_NEAR(
    fr_s_e.get_value(), fl_val * rbtq_current_fb_to_newton_meters_, assert_near_abs_error_);
  ASSERT_NEAR(
    fl_s_e.get_value(), fr_val * rbtq_current_fb_to_newton_meters_, assert_near_abs_error_);
  ASSERT_NEAR(
    rr_s_e.get_value(), rl_val * rbtq_current_fb_to_newton_meters_, assert_near_abs_error_);
  ASSERT_NEAR(
    rl_s_e.get_value(), rr_val * rbtq_current_fb_to_newton_meters_, assert_near_abs_error_);

  shutdown_panther_system();
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

  roboteq_mock_->front_driver_->SetTemperature(f_temp);
  roboteq_mock_->rear_driver_->SetTemperature(r_temp);
  roboteq_mock_->front_driver_->SetVoltage(f_volt);
  roboteq_mock_->rear_driver_->SetVoltage(r_volt);
  roboteq_mock_->front_driver_->SetBatAmps1(f_bat_amps_1);
  roboteq_mock_->rear_driver_->SetBatAmps1(r_bat_amps_1);
  roboteq_mock_->front_driver_->SetBatAmps2(f_bat_amps_2);
  roboteq_mock_->rear_driver_->SetBatAmps2(r_bat_amps_2);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");

  configure_panther_system();
  activate_panther_system();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    "/panther_system_node/driver/motor_controllers_state", rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  const auto TIME = node->get_clock()->now();
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);
  try {
    rm_->read(TIME, PERIOD);
  } catch (std::exception & err) {
    FAIL() << "Exception: " << err.what();
    return;
  }

  rclcpp::Time start = node->now();
  while (node->now() - start < rclcpp::Duration(std::chrono::seconds(5))) {
    rclcpp::spin_some(node);
    if (state_msg) {
      break;
    }
  }

  sub.reset();

  ASSERT_TRUE(state_msg);

  ASSERT_EQ(state_msg->front.temperature, f_temp);
  ASSERT_EQ(state_msg->rear.temperature, r_temp);

  ASSERT_EQ(state_msg->front.voltage, f_volt / 10);
  ASSERT_EQ(state_msg->rear.voltage, r_volt / 10);

  ASSERT_EQ(state_msg->front.current, (f_bat_amps_1 + f_bat_amps_2) / 10);
  ASSERT_EQ(state_msg->rear.current, (r_bat_amps_1 + r_bat_amps_2) / 10);

  shutdown_panther_system();
}

// ENCODER DISCONNECTED
TEST_F(TestPantherSystem, encoder_disconnected_panther_system)
{
  using hardware_interface::LoanedCommandInterface;

  roboteq_mock_->front_driver_->SetDriverScriptFlag(DriverScriptFlags::ENCODER_DISCONNECTED);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");

  configure_panther_system();
  activate_panther_system();

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    "/panther_system_node/driver/motor_controllers_state", rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  const auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(period_);

  rm_->read(TIME, PERIOD);

  rclcpp::Time start = node->now();
  while (node->now() - start < rclcpp::Duration(std::chrono::seconds(5))) {
    rclcpp::spin_some(node);
    if (state_msg) {
      break;
    }
  }

  ASSERT_TRUE(state_msg->front.script_flag.encoder_disconected);

  // writing should be blocked - error

  LoanedCommandInterface fl_c_v = rm_->claim_command_interface("fl_wheel_joint/velocity");
  LoanedCommandInterface fr_c_v = rm_->claim_command_interface("fr_wheel_joint/velocity");
  LoanedCommandInterface rl_c_v = rm_->claim_command_interface("rl_wheel_joint/velocity");
  LoanedCommandInterface rr_c_v = rm_->claim_command_interface("rr_wheel_joint/velocity");

  fl_c_v.set_value(0.1);
  fr_c_v.set_value(0.1);
  rl_c_v.set_value(0.1);
  rr_c_v.set_value(0.1);

  ASSERT_EQ(0.1, fl_c_v.get_value());
  ASSERT_EQ(0.1, fr_c_v.get_value());
  ASSERT_EQ(0.1, rl_c_v.get_value());
  ASSERT_EQ(0.1, rr_c_v.get_value());

  rm_->write(TIME, PERIOD);

  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(1), 0);
  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(2), 0);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(1), 0);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(2), 0);

  shutdown_panther_system();
}

// INITIAL PROCEDURE
TEST_F(TestPantherSystem, initial_procedure_test_panther_system)
{
  using hardware_interface::LoanedStateInterface;

  roboteq_mock_->front_driver_->SetRoboteqCmd(1, 234);
  roboteq_mock_->front_driver_->SetRoboteqCmd(2, 32);
  roboteq_mock_->rear_driver_->SetRoboteqCmd(1, 54);
  roboteq_mock_->rear_driver_->SetRoboteqCmd(2, 12);

  roboteq_mock_->front_driver_->SetResetRoboteqScript(65);
  roboteq_mock_->rear_driver_->SetResetRoboteqScript(23);

  configure_panther_system();
  activate_panther_system();

  // TODO check timing

  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(1), 0);
  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(2), 0);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(1), 0);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(2), 0);

  ASSERT_EQ(roboteq_mock_->front_driver_->GetResetRoboteqScript(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetResetRoboteqScript(), 2);

  shutdown_panther_system();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // For testing individual tests:
  // testing::GTEST_FLAG(filter) = "TestPantherSystem.encoder_disconnected_panther_system";

  return RUN_ALL_TESTS();
}