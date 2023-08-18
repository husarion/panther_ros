#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include "lifecycle_msgs/msg/state.hpp"

#include <mock_roboteq.hpp>

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

// // set some new values in commands
// fl_c_v.set_value(0.1);
// fr_c_v.set_value(0.2);
// rl_c_v.set_value(0.3);
// rr_c_v.set_value(0.4);

// // State values should not be changed
// ASSERT_EQ(0.0, fl_s_p.get_value());
// ASSERT_EQ(0.0, fr_s_p.get_value());
// ASSERT_EQ(0.0, rl_s_p.get_value());
// ASSERT_EQ(0.0, rr_s_p.get_value());

// ASSERT_EQ(0.0, fl_s_v.get_value());
// ASSERT_EQ(0.0, fr_s_v.get_value());
// ASSERT_EQ(0.0, rl_s_v.get_value());
// ASSERT_EQ(0.0, rr_s_v.get_value());

// ASSERT_EQ(0.0, fl_s_e.get_value());
// ASSERT_EQ(0.0, fr_s_e.get_value());
// ASSERT_EQ(0.0, rl_s_e.get_value());
// ASSERT_EQ(0.0, rr_s_e.get_value());

// ASSERT_EQ(0.1, fl_c_v.get_value());
// ASSERT_EQ(0.2, fr_c_v.get_value());
// ASSERT_EQ(0.3, rl_c_v.get_value());
// ASSERT_EQ(0.4, rr_c_v.get_value());

// const auto TIME = rclcpp::Time(0);
// const auto PERIOD = rclcpp::Duration::from_seconds(0.01);

// // write() does not change values
// rm.write(TIME, PERIOD);
// ASSERT_EQ(3.45, j1p_s.get_value());
// ASSERT_EQ(0.0, j1v_s.get_value());
// ASSERT_EQ(2.78, j2p_s.get_value());
// ASSERT_EQ(0.0, j2v_s.get_value());
// ASSERT_EQ(0.11, j1p_c.get_value());
// ASSERT_EQ(0.22, j1v_c.get_value());
// ASSERT_EQ(0.33, j2p_c.get_value());
// ASSERT_EQ(0.44, j2v_c.get_value());

// // read() mirrors commands + offset to states
// rm.read(TIME, PERIOD);
// ASSERT_EQ(0.11 + offset, j1p_s.get_value());
// ASSERT_EQ(0.22, j1v_s.get_value());
// ASSERT_EQ(0.33 + offset, j2p_s.get_value());
// ASSERT_EQ(0.44, j2v_s.get_value());
// ASSERT_EQ(0.11, j1p_c.get_value());
// ASSERT_EQ(0.22, j1v_c.get_value());
// ASSERT_EQ(0.33, j2p_c.get_value());
// ASSERT_EQ(0.44, j2v_c.get_value());

// // set some new values in commands
// j1p_c.set_value(0.55);
// j1v_c.set_value(0.66);
// j2p_c.set_value(0.77);
// j2v_c.set_value(0.88);

// // state values should not be changed
// ASSERT_EQ(0.11 + offset, j1p_s.get_value());
// ASSERT_EQ(0.22, j1v_s.get_value());
// ASSERT_EQ(0.33 + offset, j2p_s.get_value());
// ASSERT_EQ(0.44, j2v_s.get_value());
// ASSERT_EQ(0.55, j1p_c.get_value());
// ASSERT_EQ(0.66, j1v_c.get_value());
// ASSERT_EQ(0.77, j2p_c.get_value());
// ASSERT_EQ(0.88, j2v_c.get_value());

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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}