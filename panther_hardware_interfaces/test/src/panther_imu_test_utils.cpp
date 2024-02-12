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

#include <panther_imu_test_utils.hpp>

#include <cstdint>

#include <lifecycle_msgs/msg/state.hpp>

#include <hardware_interface/types/lifecycle_state_names.hpp>

namespace panther_hardware_interfaces_test
{

void PantherImuTestUtils::Start(const std::string & urdf)
{
  rm_ = std::make_shared<hardware_interface::ResourceManager>(urdf);
}

void PantherImuTestUtils::Stop()
{
  rclcpp::shutdown();
  rm_.reset();
}

std::string PantherImuTestUtils::BuildUrdf(
  const std::map<std::string, std::string> & param_map)
{
  std::stringstream urdf;

  urdf << kUrdfHeader << "<hardware>" << std::endl << kPluginName;

  // for (auto const & [key, val] : param_map) {
  //   urdf << "<param name=\"" << key << "\">" << val << "</param>" << std::endl;
  // }

  urdf << "</hardware>" << std::endl;

  urdf << kImuInterfaces << std::endl;

  urdf << kUrdfFooter;

  return urdf.str();
}

void PantherImuTestUtils::ConfigurePantherImu()
{
  SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
}

void PantherImuTestUtils::UnconfigurePantherImu()
{
  SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

void PantherImuTestUtils::ActivatePantherImu()
{
  SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);
}

void PantherImuTestUtils::DeactivatePantherImu()
{
  SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
}

void PantherImuTestUtils::ShutdownPantherImu()
{
  SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
    hardware_interface::lifecycle_state_names::FINALIZED);
}

void PantherImuTestUtils::ConfigureActivatePantherImu()
{
  ConfigurePantherImu();
  ActivatePantherImu();
}

void PantherImuTestUtils::SetState(const std::uint8_t state_id, const std::string & state_name)
{
  rclcpp_lifecycle::State state(state_id, state_name);
  rm_->set_component_state(kPantherImuName, state);
}

}  // namespace panther_hardware_interfaces_test
