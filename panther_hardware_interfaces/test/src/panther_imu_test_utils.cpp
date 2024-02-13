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
  rclcpp::init(0, nullptr);
}

void PantherImuTestUtils::Stop()
{
  rclcpp::shutdown();
  rm_.reset();
}

std::string PantherImuTestUtils::BuildUrdf(
  const std::map<std::string, std::string> & param_map, const std::list<std::string> & interfaces_list)
{
  std::stringstream urdf;

  urdf << kUrdfHeader << "<hardware>" << std::endl << kPluginName;

  for (auto const & [key, val] : param_map) {
    urdf << "<param name=\"" << key << "\">" << val << "</param>" << std::endl;
  }

  urdf << "</hardware>" << std::endl;

  urdf << "<sensor name=\"imu\" >" << std::endl;

  for (auto const & val : interfaces_list) {
    urdf << "<state_interface name=\"" << val << "\"/>" << std::endl;
  }
  urdf << "</sensor>" << std::endl;


  urdf << kUrdfFooter;

  return urdf.str();
}

hardware_interface::return_type PantherImuTestUtils::ConfigurePantherImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
}

hardware_interface::return_type PantherImuTestUtils::UnconfigurePantherImu()
{
 return  SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

hardware_interface::return_type PantherImuTestUtils::ActivatePantherImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);
}

hardware_interface::return_type PantherImuTestUtils::DeactivatePantherImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
}

hardware_interface::return_type PantherImuTestUtils::ShutdownPantherImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
    hardware_interface::lifecycle_state_names::FINALIZED);
}

hardware_interface::return_type PantherImuTestUtils::SetState(const std::uint8_t state_id, const std::string & state_name)
{
  rclcpp_lifecycle::State state(state_id, state_name);
  return rm_->set_component_state(kPantherImuName, state);
}

}  // namespace panther_hardware_interfaces_test
