#ifndef PANTHER_HARDWARE_INTERFACES__TEST_UTILS_HPP_
#define PANTHER_HARDWARE_INTERFACES__TEST_UTILS_HPP_

#include <cmath>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <mock_roboteq.hpp>

class TestPantherSystem : public ::testing::Test
{
public:
  std::unique_ptr<RoboteqMock> roboteq_mock_;
  std::unique_ptr<hardware_interface::ResourceManager> rm_;

  double rad_per_sec_to_rbtq_cmd_ = 30.08 * (1.0 / (2.0 * M_PI)) * 60.0 * (1000.0 / 3600.0);
  double rbtq_pos_fb_to_rad_ = (1. / 1600) * (1.0 / 30.08) * (2.0 * M_PI);
  double rbtq_vel_fb_to_rad_per_sec_ = (1. / 30.08) * (1. / 60.) * (2.0 * M_PI);
  double rbtq_current_fb_to_newton_meters_ = (1. / 10.) * 0.11 * 30.08 * 0.75;

  // 100 Hz
  const double period_ = 0.01;

  const double assert_near_abs_error_ = 0.0001;

  //  TODO: move to constructor
  void SetUp() override
  {
    roboteq_mock_ = std::make_unique<RoboteqMock>();
    roboteq_mock_->Start();
    rclcpp::init(0, nullptr);

    rm_ = std::make_unique<hardware_interface::ResourceManager>(panther_system_urdf_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    roboteq_mock_->Stop();
    roboteq_mock_.reset();
    rm_.reset();
  }

  const std::string panther_system_name_ = "wheels";

  const std::string panther_system_urdf_ =
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
      <param name="sdo_operation_timeout">4</param>
      <param name="pdo_feedback_timeout">15</param>
      <param name="max_roboteq_initialization_attempts">3</param>
      <param name="max_roboteq_activation_attempts">3</param>
      <param name="max_safety_stop_attempts">20</param>
      <param name="max_write_sdo_errors_count">2</param>
      <param name="max_read_sdo_errors_count">2</param>
      <param name="max_read_pdo_errors_count">1</param>
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

  // TODO: add test
  const std::string panther_system_urdf_changed_order_ =
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
      <param name="sdo_operation_timeout">4</param>
      <param name="pdo_feedback_timeout">15</param>
      <param name="max_roboteq_initialization_attempts">3</param>
      <param name="max_roboteq_activation_attempts">3</param>
      <param name="max_safety_stop_attempts">20</param>
      <param name="max_write_sdo_errors_count">2</param>
      <param name="max_read_sdo_errors_count">2</param>
      <param name="max_read_pdo_errors_count">1</param>
    </hardware>
    
    <joint name="rl_wheel_joint">
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
    <joint name="rr_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
    <joint name="fl_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
  </ros2_control>
</robot>
)";

  void set_state(const uint8_t state_id, const std::string & state_name)
  {
    rclcpp_lifecycle::State state(state_id, state_name);
    rm_->set_component_state(panther_system_name_, state);
  }

  void configure_panther_system()
  {
    set_state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  }

  void unconfigure_panther_system()
  {
    set_state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
  }

  void activate_panther_system()
  {
    set_state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
  }

  void deactivate_panther_system()
  {
    set_state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  }

  void shutdown_panther_system()
  {
    set_state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
      hardware_interface::lifecycle_state_names::FINALIZED);
  }

  void check_interfaces()
  {
    EXPECT_EQ(1u, rm_->system_components_size());
    ASSERT_EQ(12u, rm_->state_interface_keys().size());
    EXPECT_TRUE(rm_->state_interface_exists("fl_wheel_joint/position"));
    EXPECT_TRUE(rm_->state_interface_exists("fr_wheel_joint/position"));
    EXPECT_TRUE(rm_->state_interface_exists("rl_wheel_joint/position"));
    EXPECT_TRUE(rm_->state_interface_exists("rr_wheel_joint/position"));

    EXPECT_TRUE(rm_->state_interface_exists("fl_wheel_joint/velocity"));
    EXPECT_TRUE(rm_->state_interface_exists("fr_wheel_joint/velocity"));
    EXPECT_TRUE(rm_->state_interface_exists("rl_wheel_joint/velocity"));
    EXPECT_TRUE(rm_->state_interface_exists("rr_wheel_joint/velocity"));

    EXPECT_TRUE(rm_->state_interface_exists("fl_wheel_joint/effort"));
    EXPECT_TRUE(rm_->state_interface_exists("fr_wheel_joint/effort"));
    EXPECT_TRUE(rm_->state_interface_exists("rl_wheel_joint/effort"));
    EXPECT_TRUE(rm_->state_interface_exists("rr_wheel_joint/effort"));

    ASSERT_EQ(4u, rm_->command_interface_keys().size());
    EXPECT_TRUE(rm_->command_interface_exists("fl_wheel_joint/velocity"));
    EXPECT_TRUE(rm_->command_interface_exists("fr_wheel_joint/velocity"));
    EXPECT_TRUE(rm_->command_interface_exists("rl_wheel_joint/velocity"));
    EXPECT_TRUE(rm_->command_interface_exists("rr_wheel_joint/velocity"));
  }

  void check_initial_values()
  {
    using hardware_interface::LoanedCommandInterface;
    using hardware_interface::LoanedStateInterface;

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

    LoanedCommandInterface fl_c_v = rm_->claim_command_interface("fl_wheel_joint/velocity");
    LoanedCommandInterface fr_c_v = rm_->claim_command_interface("fr_wheel_joint/velocity");
    LoanedCommandInterface rl_c_v = rm_->claim_command_interface("rl_wheel_joint/velocity");
    LoanedCommandInterface rr_c_v = rm_->claim_command_interface("rr_wheel_joint/velocity");

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
};

#endif