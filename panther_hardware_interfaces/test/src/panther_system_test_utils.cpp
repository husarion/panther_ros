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

#include <panther_system_test_utils.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <hardware_interface/types/lifecycle_state_names.hpp>

void PantherSystemTestUtils::SetState(const uint8_t state_id, const std::string & state_name)
{
  rclcpp_lifecycle::State state(state_id, state_name);
  rm_->set_component_state(panther_system_name_, state);
}

void PantherSystemTestUtils::ConfigurePantherSystem()
{
  SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
}

void PantherSystemTestUtils::UnconfigurePantherSystem()
{
  SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

void PantherSystemTestUtils::ActivatePantherSystem()
{
  SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);
}

void PantherSystemTestUtils::DeactivatePantherSystem()
{
  SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
}

void PantherSystemTestUtils::ShutdownPantherSystem()
{
  SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
    hardware_interface::lifecycle_state_names::FINALIZED);
}

void PantherSystemTestUtils::ConfigureActivatePantherSystem()
{
  ConfigurePantherSystem();
  ActivatePantherSystem();
}

void PantherSystemTestUtils::Start(std::string urdf)
{
  roboteq_mock_ = std::make_unique<RoboteqMock>();
  // PDO running on 100Hz
  roboteq_mock_->Start(std::chrono::milliseconds(10));
  rclcpp::init(0, nullptr);

  rm_ = std::make_unique<hardware_interface::ResourceManager>(urdf);
}

void PantherSystemTestUtils::Stop()
{
  rclcpp::shutdown();
  roboteq_mock_->Stop();
  roboteq_mock_.reset();
  rm_.reset();
}

std::string PantherSystemTestUtils::BuildUrdf(
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

void TestPantherSystem::CheckInterfaces()
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

void TestPantherSystem::CheckInitialValues()
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

  LoanedCommandInterface fl_c_v = pth_test_.rm_->claim_command_interface("fl_wheel_joint/velocity");
  LoanedCommandInterface fr_c_v = pth_test_.rm_->claim_command_interface("fr_wheel_joint/velocity");
  LoanedCommandInterface rl_c_v = pth_test_.rm_->claim_command_interface("rl_wheel_joint/velocity");
  LoanedCommandInterface rr_c_v = pth_test_.rm_->claim_command_interface("rr_wheel_joint/velocity");

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
