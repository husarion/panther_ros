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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_ROBOT_SYSTEM_TEST_UTILS_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_ROBOT_SYSTEM_TEST_UTILS_HPP_

#include <cmath>
#include <cstdint>
#include <map>
#include <string>

#include <gtest/gtest.h>

#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/rclcpp.hpp>

#include "roboteqs_mock.hpp"
#include "test_constants.hpp"

namespace husarion_ugv_hardware_interfaces_test
{

/**
 * @brief Utility class for testing Panther System
 */
class PantherSystemTestUtils
{
public:
  PantherSystemTestUtils()
  {
    default_panther_system_urdf_ = BuildUrdf(kDefaultParamMap, kDefaultJoints);
  }

  /**
   * @brief Starts Roboteq Mock, initializes rclcpp and creates resource manager
   * @param urdf urdf used to create resource manager
   */
  void Start(const std::string & urdf)
  {
    roboteqs_mock_ = std::make_unique<RoboteqsMock>();
    roboteqs_mock_->Start(std::chrono::milliseconds(10), std::chrono::milliseconds(50));
    rclcpp::init(0, nullptr);

    rm_ = std::make_shared<hardware_interface::ResourceManager>(urdf);
  }

  /**
   * @brief Shuts down rclcpp, stops Roboteq mock and destroys resource manager
   */
  void Stop()
  {
    rclcpp::shutdown();
    roboteqs_mock_->Stop();
    roboteqs_mock_.reset();
    rm_.reset();
  }

  /**
   * @brief Creates and returns URDF as a string
   * @param param_map map with hardware parameters
   * @param joints vector of joint names
   */
  std::string BuildUrdf(
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

  void ConfigurePantherSystem()
  {
    SetState(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  }

  void UnconfigurePantherSystem()
  {
    SetState(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
  }

  void ActivatePantherSystem()
  {
    SetState(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
  }

  void DeactivatePantherSystem()
  {
    SetState(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  }

  void ShutdownPantherSystem()
  {
    SetState(
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
      hardware_interface::lifecycle_state_names::FINALIZED);
  }

  void ConfigureActivatePantherSystem()
  {
    ConfigurePantherSystem();
    ActivatePantherSystem();
  }

  std::shared_ptr<hardware_interface::ResourceManager> GetResourceManager() { return rm_; }
  std::shared_ptr<RoboteqsMock> GetRoboteqsMock() { return roboteqs_mock_; }

  std::string GetDefaultPantherSystemUrdf() const { return default_robot_system_urdf_; }

private:
  /**
   * @brief Changes current state of the resource manager to the one set in parameters. It is
   * recommended to use wrapper functions
   * @param state_id
   * @param state_name
   */
  void SetState(const std::uint8_t state_id, const std::string & state_name)
  {
    rclcpp_lifecycle::State state(state_id, state_name);
    rm_->set_component_state(kPantherSystemName, state);
  }

  std::shared_ptr<RoboteqsMock> roboteqs_mock_;
  std::shared_ptr<hardware_interface::ResourceManager> rm_;

  std::string default_panther_system_urdf_;
};

}  // namespace husarion_ugv_hardware_interfaces_test

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_ROBOT_SYSTEM_TEST_UTILS_HPP_
