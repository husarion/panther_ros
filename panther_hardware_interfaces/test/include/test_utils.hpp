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

#ifndef PANTHER_HARDWARE_INTERFACES__TEST_UTILS_HPP_
#define PANTHER_HARDWARE_INTERFACES__TEST_UTILS_HPP_

#include <cmath>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <mock_roboteq.hpp>

class PantherSystemTestUtils
{
public:
  std::unique_ptr<RoboteqMock> roboteq_mock_;
  std::unique_ptr<hardware_interface::ResourceManager> rm_;

  double rad_per_sec_to_rbtq_cmd_ = 30.08 * (1.0 / (2.0 * M_PI)) * 60.0 * (1000.0 / 3600.0);
  double rbtq_pos_fb_to_rad_ = (1. / 1600) * (1.0 / 30.08) * (2.0 * M_PI);
  double rbtq_vel_fb_to_rad_per_sec_ = (1. / 30.08) * (1. / 60.) * (2.0 * M_PI);
  double rbtq_current_fb_to_newton_meters_ = (1. / 10.) * 0.11 * 30.08 * 0.75;

  const std::string panther_system_name_ = "wheels";

  const std::string urdf_header_ = R"(<?xml version="1.0" encoding="utf-8"?>
<robot name="Panther">
<ros2_control name="wheels" type="system">
)";

  const std::string urdf_footer_ = R"(</ros2_control>
</robot>
)";

  const std::string joint_interfaces_ =
    R"(<command_interface name="velocity" />
<state_interface name="position" />
<state_interface name="velocity" />
<state_interface name="effort" />
)";

  std::string BuildUrdf(
    std::map<std::string, std::string> param_map, std::vector<std::string> joints)
  {
    std::stringstream urdf;

    urdf << urdf_header_ << R"(<hardware>
<plugin>panther_hardware_interfaces/PantherSystem</plugin>
)";

    for (auto const & [key, val] : param_map) {
      urdf << "<param name=\"" << key << "\">" << val << "</param>" << std::endl;
    }

    urdf << R"(</hardware>
)";

    for (auto const & joint : joints) {
      urdf << "<joint name=\"" << joint << "\">" << std::endl
           << joint_interfaces_ << "</joint>" << std::endl;
    }

    urdf << urdf_footer_;

    return urdf.str();
  }

  std::string default_panther_system_urdf_;

  std::map<std::string, std::string> param_map_ = {
    {"encoder_resolution", "1600"},
    {"gear_ratio", "30.08"},
    {"gearbox_efficiency", "0.75"},
    {"motor_torque_constant", "0.11"},
    {"max_rpm_motor_speed", "3600.0"},
    {"master_can_id", "3"},
    {"front_driver_can_id", "1"},
    {"rear_driver_can_id", "2"},
    {"sdo_operation_timeout", "4"},
    {"pdo_feedback_timeout", "15"},
    {"max_roboteq_initialization_attempts", "3"},
    {"max_roboteq_activation_attempts", "3"},
    {"max_safety_stop_attempts", "20"},
    {"max_write_sdo_errors_count", "2"},
    {"max_read_sdo_errors_count", "2"},
    {"max_read_pdo_errors_count", "1"},
  };

  std::vector<std::string> joints_ = {
    "fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"};

  PantherSystemTestUtils() { default_panther_system_urdf_ = BuildUrdf(param_map_, joints_); }

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

  void configure_activate_panther_system()
  {
    configure_panther_system();
    activate_panther_system();
  }

  void Start(std::string urdf)
  {
    roboteq_mock_ = std::make_unique<RoboteqMock>();
    roboteq_mock_->Start();
    rclcpp::init(0, nullptr);

    rm_ = std::make_unique<hardware_interface::ResourceManager>(urdf);
  }

  void Stop()
  {
    rclcpp::shutdown();
    roboteq_mock_->Stop();
    roboteq_mock_.reset();
    rm_.reset();
  }
};

class TestPantherSystem : public ::testing::Test
{
public:
  PantherSystemTestUtils pth_test_;

  TestPantherSystem() { pth_test_.Start(pth_test_.default_panther_system_urdf_); }
  ~TestPantherSystem() { pth_test_.Stop(); }

  // 100 Hz
  const double period_ = 0.01;

  const double assert_near_abs_error_ = 0.0001;

  void check_interfaces()
  {
    EXPECT_EQ(1u, pth_test_.rm_->system_components_size());
    ASSERT_EQ(12u, pth_test_.rm_->state_interface_keys().size());
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

    ASSERT_EQ(4u, pth_test_.rm_->command_interface_keys().size());
    EXPECT_TRUE(pth_test_.rm_->command_interface_exists("fl_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.rm_->command_interface_exists("fr_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.rm_->command_interface_exists("rl_wheel_joint/velocity"));
    EXPECT_TRUE(pth_test_.rm_->command_interface_exists("rr_wheel_joint/velocity"));
  }

  void check_initial_values()
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
