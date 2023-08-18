#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <mock_roboteq.hpp>

// UTILS
std::string panther_system_urdf =
  R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="Panther">
  <ros2_control name="wheels" type="system">
    <hardware>
      <plugin>panther_hardware_interfaces/PantherSystem</plugin>
      <param name="encoder_resolution">1600</param>
      <param name="gear_ratio">30.08</param>
      <param name="gearbox_efficiency">0.75</param>
      <param name="motor_torque_constant">0.11</param>
      <param name="max_rpm_motor_speed">3600.0</param>
      <param name="master_can_id">3</param>
      <param name="front_driver_can_id">1</param>
      <param name="rear_driver_can_id">2</param>
      <param name="roboteq_state_period">1.0</param>
    </hardware>

    <joint name="fl_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
    <joint name="fr_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
    <joint name="rl_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
    <joint name="rr_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
  </ros2_control>
</robot>
)";

void set_components_state(
  hardware_interface::ResourceManager & rm, const std::vector<std::string> & components,
  const uint8_t state_id, const std::string & state_name)
{
  for (const auto & component : components) {
    rclcpp_lifecycle::State state(state_id, state_name);
    rm.set_component_state(component, state);
  }
}

auto configure_components = [](
                              hardware_interface::ResourceManager & rm,
                              const std::vector<std::string> & components = {"wheels"}) {
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
};

auto unconfigure_components = [](
                                hardware_interface::ResourceManager & rm,
                                const std::vector<std::string> & components = {"wheels"}) {
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
};

auto activate_components = [](
                             hardware_interface::ResourceManager & rm,
                             const std::vector<std::string> & components = {"wheels"}) {
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);
};

auto deactivate_components = [](
                               hardware_interface::ResourceManager & rm,
                               const std::vector<std::string> & components = {"wheels"}) {
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
};

auto shutdown_components = [](
                             hardware_interface::ResourceManager & rm,
                             const std::vector<std::string> & components = {"wheels"}) {
  set_components_state(
    rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
    hardware_interface::lifecycle_state_names::FINALIZED);
};

// LOADING
TEST(TestPantherSystem, load_panther_system)
{
  // Use try-catch instead of ASSERT_NO_THROW to get and print exception message
  try {
    hardware_interface::ResourceManager rm(panther_system_urdf);

    EXPECT_EQ(
      rm.get_components_status()["wheels"].state.label(),
      hardware_interface::lifecycle_state_names::UNCONFIGURED);

    SUCCEED();
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to create resource manager: " << err.what();
  }
}

// TRANSITIONS

void check_interfaces(hardware_interface::ResourceManager & rm)
{
  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(12u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("fl_wheel_joint/position"));
  EXPECT_TRUE(rm.state_interface_exists("fr_wheel_joint/position"));
  EXPECT_TRUE(rm.state_interface_exists("rl_wheel_joint/position"));
  EXPECT_TRUE(rm.state_interface_exists("rr_wheel_joint/position"));

  EXPECT_TRUE(rm.state_interface_exists("fl_wheel_joint/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("fr_wheel_joint/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("rl_wheel_joint/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("rr_wheel_joint/velocity"));

  EXPECT_TRUE(rm.state_interface_exists("fl_wheel_joint/effort"));
  EXPECT_TRUE(rm.state_interface_exists("fr_wheel_joint/effort"));
  EXPECT_TRUE(rm.state_interface_exists("rl_wheel_joint/effort"));
  EXPECT_TRUE(rm.state_interface_exists("rr_wheel_joint/effort"));

  ASSERT_EQ(4u, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.command_interface_exists("fl_wheel_joint/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("fr_wheel_joint/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("rl_wheel_joint/velocity"));
  EXPECT_TRUE(rm.command_interface_exists("rr_wheel_joint/velocity"));
}

void check_initial_values(hardware_interface::ResourceManager & rm)
{
  using hardware_interface::LoanedCommandInterface;
  using hardware_interface::LoanedStateInterface;

  LoanedStateInterface fl_s_p = rm.claim_state_interface("fl_wheel_joint/position");
  LoanedStateInterface fr_s_p = rm.claim_state_interface("fr_wheel_joint/position");
  LoanedStateInterface rl_s_p = rm.claim_state_interface("rl_wheel_joint/position");
  LoanedStateInterface rr_s_p = rm.claim_state_interface("rr_wheel_joint/position");

  LoanedStateInterface fl_s_v = rm.claim_state_interface("fl_wheel_joint/velocity");
  LoanedStateInterface fr_s_v = rm.claim_state_interface("fr_wheel_joint/velocity");
  LoanedStateInterface rl_s_v = rm.claim_state_interface("rl_wheel_joint/velocity");
  LoanedStateInterface rr_s_v = rm.claim_state_interface("rr_wheel_joint/velocity");

  LoanedStateInterface fl_s_e = rm.claim_state_interface("fl_wheel_joint/effort");
  LoanedStateInterface fr_s_e = rm.claim_state_interface("fr_wheel_joint/effort");
  LoanedStateInterface rl_s_e = rm.claim_state_interface("rl_wheel_joint/effort");
  LoanedStateInterface rr_s_e = rm.claim_state_interface("rr_wheel_joint/effort");

  LoanedCommandInterface fl_c_v = rm.claim_command_interface("fl_wheel_joint/velocity");
  LoanedCommandInterface fr_c_v = rm.claim_command_interface("fr_wheel_joint/velocity");
  LoanedCommandInterface rl_c_v = rm.claim_command_interface("rl_wheel_joint/velocity");
  LoanedCommandInterface rr_c_v = rm.claim_command_interface("rr_wheel_joint/velocity");

  ASSERT_EQ(0.0, fl_s_p.get_value());
  ASSERT_EQ(0.0, fr_s_p.get_value());
  ASSERT_EQ(0.0, rl_s_p.get_value());
  ASSERT_EQ(0.0, rr_s_p.get_value());

  ASSERT_EQ(0.0, fl_s_v.get_value());
  ASSERT_EQ(0.0, fr_s_v.get_value());
  ASSERT_EQ(0.0, rl_s_v.get_value());
  ASSERT_EQ(0.0, rr_s_v.get_value());

  ASSERT_EQ(0.0, fl_s_e.get_value());
  ASSERT_EQ(0.0, fr_s_e.get_value());
  ASSERT_EQ(0.0, rl_s_e.get_value());
  ASSERT_EQ(0.0, rr_s_e.get_value());

  ASSERT_EQ(0.0, fl_c_v.get_value());
  ASSERT_EQ(0.0, fr_c_v.get_value());
  ASSERT_EQ(0.0, rl_c_v.get_value());
  ASSERT_EQ(0.0, rr_c_v.get_value());
}

TEST(TestPantherSystem, configure_activate_finalize_panther_system)
{
  RoboteqMock roboteq_mock;
  roboteq_mock.Start();

  rclcpp::init(0, nullptr);

  hardware_interface::ResourceManager rm(panther_system_urdf);

  // check is hardware is configured
  auto status_map = rm.get_components_status();
  ASSERT_EQ(
    status_map["wheels"].state.label(), hardware_interface::lifecycle_state_names::UNCONFIGURED);

  try {
    configure_components(rm);
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to configure_components: " << err.what();
    return;
  }
  status_map = rm.get_components_status();
  ASSERT_EQ(
    status_map["wheels"].state.label(), hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    activate_components(rm);
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to activate_components: " << err.what();
    return;
  }
  status_map = rm.get_components_status();
  ASSERT_EQ(status_map["wheels"].state.label(), hardware_interface::lifecycle_state_names::ACTIVE);

  // Check interfaces
  check_interfaces(rm);

  // Check initial values
  check_initial_values(rm);

  try {
    shutdown_components(rm);
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to shutdown_components: " << err.what();
    return;
  }
  status_map = rm.get_components_status();
  ASSERT_EQ(
    status_map["wheels"].state.label(), hardware_interface::lifecycle_state_names::FINALIZED);

  // TODO test teardown
  roboteq_mock.Stop();
  rclcpp::shutdown();
}

TEST(TestPantherSystem, configure_activate_deactivate_deconfigure_panther_system)
{
  RoboteqMock roboteq_mock;
  roboteq_mock.Start();

  rclcpp::init(0, nullptr);

  hardware_interface::ResourceManager rm(panther_system_urdf);

  auto status_map = rm.get_components_status();
  ASSERT_EQ(
    status_map["wheels"].state.label(), hardware_interface::lifecycle_state_names::UNCONFIGURED);

  try {
    configure_components(rm);
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to create resource manager: " << err.what();
    return;
  }
  status_map = rm.get_components_status();
  ASSERT_EQ(
    status_map["wheels"].state.label(), hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    activate_components(rm);
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to create resource manager: " << err.what();
    return;
  }
  status_map = rm.get_components_status();
  ASSERT_EQ(status_map["wheels"].state.label(), hardware_interface::lifecycle_state_names::ACTIVE);

  // Check interfaces
  check_interfaces(rm);

  // Check initial values
  check_initial_values(rm);

  try {
    deactivate_components(rm);
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to deactivate_components: " << err.what();
    return;
  }
  status_map = rm.get_components_status();
  ASSERT_EQ(
    status_map["wheels"].state.label(), hardware_interface::lifecycle_state_names::INACTIVE);

  try {
    unconfigure_components(rm);
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to unconfigure_components: " << err.what();
    return;
  }
  status_map = rm.get_components_status();
  ASSERT_EQ(
    status_map["wheels"].state.label(), hardware_interface::lifecycle_state_names::UNCONFIGURED);

  // TODO test teardown
  roboteq_mock.Stop();
  rclcpp::shutdown();
}

// WRITING
TEST(TestPantherSystem, write_commands_panther_system)
{
  using hardware_interface::LoanedCommandInterface;

  RoboteqMock roboteq_mock;
  roboteq_mock.Start();

  rclcpp::init(0, nullptr);

  hardware_interface::ResourceManager rm(panther_system_urdf);

  configure_components(rm);
  activate_components(rm);

  LoanedCommandInterface fl_c_v = rm.claim_command_interface("fl_wheel_joint/velocity");
  LoanedCommandInterface fr_c_v = rm.claim_command_interface("fr_wheel_joint/velocity");
  LoanedCommandInterface rl_c_v = rm.claim_command_interface("rl_wheel_joint/velocity");
  LoanedCommandInterface rr_c_v = rm.claim_command_interface("rr_wheel_joint/velocity");

  fl_c_v.set_value(0.1);
  fr_c_v.set_value(0.2);
  rl_c_v.set_value(0.3);
  rr_c_v.set_value(0.4);

  ASSERT_EQ(0.1, fl_c_v.get_value());
  ASSERT_EQ(0.2, fr_c_v.get_value());
  ASSERT_EQ(0.3, rl_c_v.get_value());
  ASSERT_EQ(0.4, rr_c_v.get_value());

  const auto TIME = rclcpp::Time(0);
  const auto PERIOD = rclcpp::Duration::from_seconds(0.01);

  rm.write(TIME, PERIOD);

  double radians_per_second_to_roboteq_cmd =
    30.08 * (1.0 / (2.0 * M_PI)) * 60.0 * (1000.0 / 3600.0);

  ASSERT_EQ(
    roboteq_mock.front_driver_->GetRoboteqCmd(1), int32_t(0.1 * radians_per_second_to_roboteq_cmd));
  ASSERT_EQ(
    roboteq_mock.front_driver_->GetRoboteqCmd(2), int32_t(0.2 * radians_per_second_to_roboteq_cmd));
  ASSERT_EQ(
    roboteq_mock.rear_driver_->GetRoboteqCmd(1), int32_t(0.3 * radians_per_second_to_roboteq_cmd));
  ASSERT_EQ(
    roboteq_mock.rear_driver_->GetRoboteqCmd(2), int32_t(0.4 * radians_per_second_to_roboteq_cmd));

  shutdown_components(rm);

  // TODO test teardown
  roboteq_mock.Stop();
  rclcpp::shutdown();
}

// READING
TEST(TestPantherSystem, read_feedback_panther_system)
{
  using hardware_interface::LoanedStateInterface;

  RoboteqMock roboteq_mock;
  roboteq_mock.Start();

  // TODO wait for initialization
  // workaround
  std::this_thread::sleep_for(std::chrono::seconds(2));

  roboteq_mock.front_driver_->SetPosition(1, 100);
  roboteq_mock.front_driver_->SetPosition(2, 200);
  roboteq_mock.rear_driver_->SetPosition(1, 300);
  roboteq_mock.rear_driver_->SetPosition(2, 400);

  roboteq_mock.front_driver_->SetVelocity(1, 100);
  roboteq_mock.front_driver_->SetVelocity(2, 200);
  roboteq_mock.rear_driver_->SetVelocity(1, 300);
  roboteq_mock.rear_driver_->SetVelocity(2, 400);

  roboteq_mock.front_driver_->SetCurrent(1, 100);
  roboteq_mock.front_driver_->SetCurrent(2, 200);
  roboteq_mock.rear_driver_->SetCurrent(1, 300);
  roboteq_mock.rear_driver_->SetCurrent(2, 400);

  double roboteq_pos_feedback_to_radians_ = (1. / 1600) * (1.0 / 30.08) * (2.0 * M_PI);
  double roboteq_vel_feedback_to_radians_per_second_ = (1. / 30.08) * (1. / 60.) * (2.0 * M_PI);
  double roboteq_current_feedback_to_newton_meters_ = (1. / 10.) * 0.11 * 30.08 * 0.75;

  rclcpp::init(0, nullptr);

  hardware_interface::ResourceManager rm(panther_system_urdf);

  configure_components(rm);
  activate_components(rm);

  LoanedStateInterface fl_s_p = rm.claim_state_interface("fl_wheel_joint/position");
  LoanedStateInterface fr_s_p = rm.claim_state_interface("fr_wheel_joint/position");
  LoanedStateInterface rl_s_p = rm.claim_state_interface("rl_wheel_joint/position");
  LoanedStateInterface rr_s_p = rm.claim_state_interface("rr_wheel_joint/position");

  LoanedStateInterface fl_s_v = rm.claim_state_interface("fl_wheel_joint/velocity");
  LoanedStateInterface fr_s_v = rm.claim_state_interface("fr_wheel_joint/velocity");
  LoanedStateInterface rl_s_v = rm.claim_state_interface("rl_wheel_joint/velocity");
  LoanedStateInterface rr_s_v = rm.claim_state_interface("rr_wheel_joint/velocity");

  LoanedStateInterface fl_s_e = rm.claim_state_interface("fl_wheel_joint/effort");
  LoanedStateInterface fr_s_e = rm.claim_state_interface("fr_wheel_joint/effort");
  LoanedStateInterface rl_s_e = rm.claim_state_interface("rl_wheel_joint/effort");
  LoanedStateInterface rr_s_e = rm.claim_state_interface("rr_wheel_joint/effort");

  const auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(0.01);
  try {
    rm.read(TIME, PERIOD);
  } catch (std::exception & err) {
    FAIL() << "Exception: " << err.what();
    return;
  }
  // TODO channel order
  ASSERT_NEAR(fr_s_p.get_value(), 100 * roboteq_pos_feedback_to_radians_, 0.0001);
  ASSERT_NEAR(fl_s_p.get_value(), 200 * roboteq_pos_feedback_to_radians_, 0.0001);
  ASSERT_NEAR(rr_s_p.get_value(), 300 * roboteq_pos_feedback_to_radians_, 0.0001);
  ASSERT_NEAR(rl_s_p.get_value(), 400 * roboteq_pos_feedback_to_radians_, 0.0001);

  ASSERT_NEAR(fr_s_v.get_value(), 100 * roboteq_vel_feedback_to_radians_per_second_, 0.0001);
  ASSERT_NEAR(fl_s_v.get_value(), 200 * roboteq_vel_feedback_to_radians_per_second_, 0.0001);
  ASSERT_NEAR(rr_s_v.get_value(), 300 * roboteq_vel_feedback_to_radians_per_second_, 0.0001);
  ASSERT_NEAR(rl_s_v.get_value(), 400 * roboteq_vel_feedback_to_radians_per_second_, 0.0001);

  ASSERT_NEAR(fr_s_e.get_value(), 100 * roboteq_current_feedback_to_newton_meters_, 0.0001);
  ASSERT_NEAR(fl_s_e.get_value(), 200 * roboteq_current_feedback_to_newton_meters_, 0.0001);
  ASSERT_NEAR(rr_s_e.get_value(), 300 * roboteq_current_feedback_to_newton_meters_, 0.0001);
  ASSERT_NEAR(rl_s_e.get_value(), 400 * roboteq_current_feedback_to_newton_meters_, 0.0001);

  shutdown_components(rm);

  // TODO test teardown
  roboteq_mock.Stop();
  rclcpp::shutdown();
}

TEST(TestPantherSystem, read_other_roboteq_params_panther_system)
{
  using hardware_interface::LoanedStateInterface;

  RoboteqMock roboteq_mock;
  roboteq_mock.Start();

  // TODO wait for initialization
  // workaround
  std::this_thread::sleep_for(std::chrono::seconds(2));

  roboteq_mock.front_driver_->SetTemperature(30);
  roboteq_mock.rear_driver_->SetTemperature(32);
  roboteq_mock.front_driver_->SetVoltage(400);
  roboteq_mock.rear_driver_->SetVoltage(430);
  roboteq_mock.front_driver_->SetBatAmps1(10);
  roboteq_mock.rear_driver_->SetBatAmps1(20);
  roboteq_mock.front_driver_->SetBatAmps2(30);
  roboteq_mock.rear_driver_->SetBatAmps2(40);

  rclcpp::init(0, nullptr);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hardware_interface_test_node");

  hardware_interface::ResourceManager rm(panther_system_urdf);

  configure_components(rm);
  activate_components(rm);

  panther_msgs::msg::DriverState::SharedPtr state_msg;
  auto sub = node->create_subscription<panther_msgs::msg::DriverState>(
    "/panther_system_node/driver/motor_controllers_state", rclcpp::SensorDataQoS(),
    [&](const panther_msgs::msg::DriverState::SharedPtr msg) { state_msg = msg; });

  std::this_thread::sleep_for(std::chrono::seconds(2));

  const auto TIME = node->get_clock()->now();
  const auto PERIOD = rclcpp::Duration::from_seconds(0.01);
  try {
    rm.read(TIME, PERIOD);
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

  ASSERT_EQ(state_msg->front.temperature, 30);
  ASSERT_EQ(state_msg->rear.temperature, 32);

  ASSERT_EQ(state_msg->front.voltage, 40);
  ASSERT_EQ(state_msg->rear.voltage, 43);

  ASSERT_EQ(state_msg->front.current, 4);
  ASSERT_EQ(state_msg->rear.current, 6);

  shutdown_components(rm);

  // TODO test teardown
  roboteq_mock.Stop();
  rclcpp::shutdown();
}

// ENCODER DISCONNECTED

TEST(TestPantherSystem, encoder_disconnected_panther_system)
{
  using hardware_interface::LoanedStateInterface;

  RoboteqMock roboteq_mock;
  roboteq_mock.Start();

  // TODO wait for initialization
  // workaround
  std::this_thread::sleep_for(std::chrono::seconds(2));

  roboteq_mock.front_driver_->SetDriverScriptFlag(DriverScriptFlags::ENCODER_DISCONNECTED);

  rclcpp::init(0, nullptr);

  hardware_interface::ResourceManager rm(panther_system_urdf);

  configure_components(rm);
  activate_components(rm);

  const auto TIME = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto PERIOD = rclcpp::Duration::from_seconds(0.01);

  rm.read(TIME, PERIOD);

  // error handled with success -> state changes to unconfigured
  auto status_map = rm.get_components_status();
  ASSERT_EQ(
    status_map["wheels"].state.label(), hardware_interface::lifecycle_state_names::UNCONFIGURED);

  // TODO test teardown
  roboteq_mock.Stop();
  rclcpp::shutdown();
}

// INITIAL PROCEDURE

TEST(TestPantherSystem, initial_procedure_test_panther_system)
{
  using hardware_interface::LoanedStateInterface;

  RoboteqMock roboteq_mock;
  roboteq_mock.Start();

  // TODO wait for initialization
  // workaround
  std::this_thread::sleep_for(std::chrono::seconds(2));

  roboteq_mock.front_driver_->SetRoboteqCmd(1, 234);
  roboteq_mock.front_driver_->SetRoboteqCmd(2, 32);
  roboteq_mock.rear_driver_->SetRoboteqCmd(1, 54);
  roboteq_mock.rear_driver_->SetRoboteqCmd(2, 12);

  roboteq_mock.front_driver_->SetResetRoboteqScript(65);
  roboteq_mock.rear_driver_->SetResetRoboteqScript(23);

  rclcpp::init(0, nullptr);

  hardware_interface::ResourceManager rm(panther_system_urdf);

  configure_components(rm);

  activate_components(rm);

  // TODO check timing

  ASSERT_EQ(roboteq_mock.front_driver_->GetRoboteqCmd(1), 0);
  ASSERT_EQ(roboteq_mock.front_driver_->GetRoboteqCmd(2), 0);
  ASSERT_EQ(roboteq_mock.rear_driver_->GetRoboteqCmd(1), 0);
  ASSERT_EQ(roboteq_mock.rear_driver_->GetRoboteqCmd(2), 0);

  ASSERT_EQ(roboteq_mock.front_driver_->GetResetRoboteqScript(), 2);
  ASSERT_EQ(roboteq_mock.rear_driver_->GetResetRoboteqScript(), 2);

  shutdown_components(rm);

  // TODO test teardown
  roboteq_mock.Stop();
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  // testing::GTEST_FLAG(filter) = "TestPantherSystem.initial_procedure_test_panther_system";
  return RUN_ALL_TESTS();
}