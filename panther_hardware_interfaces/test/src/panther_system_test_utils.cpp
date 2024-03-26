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

#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <hardware_interface/types/lifecycle_state_names.hpp>

namespace panther_hardware_interfaces_test
{

void PantherSystemTestUtils::Start(const std::string & urdf)
{
  roboteqs_mock_ = std::make_unique<RoboteqsMock>();
  roboteqs_mock_->Start(std::chrono::milliseconds(10), std::chrono::milliseconds(50));
  rclcpp::init(0, nullptr);

  rm_ = std::make_shared<hardware_interface::ResourceManager>(urdf);
}

void PantherSystemTestUtils::Stop()
{
  rclcpp::shutdown();
  roboteqs_mock_->Stop();
  roboteqs_mock_.reset();
  rm_.reset();
}

std::string PantherSystemTestUtils::BuildUrdf(
  const std::map<std::string, std::string> & param_map, const std::vector<std::string> & joints)
{
  std::stringstream urdf;

  urdf << kUrdfHeader << "<hardware>" << std::endl << kPluginName;

  for (auto const & [key, val] : param_map) {
    urdf << "<param name=\"" << key << "\">" << val << "</param>" << std::endl;
  }

  urdf << "</hardware>" << std::endl;

  for (auto const & joint : joints) {
    urdf << "<joint name=\"" << joint << "\">" << std::endl
         << kJointInterfaces << "</joint>" << std::endl;
  }

  urdf << kUrdfFooter;

  return urdf.str();
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

void PantherSystemTestUtils::SetState(const std::uint8_t state_id, const std::string & state_name)
{
  rclcpp_lifecycle::State state(state_id, state_name);
  rm_->set_component_state(kPantherSystemName, state);
}

}  // namespace panther_hardware_interfaces_test
