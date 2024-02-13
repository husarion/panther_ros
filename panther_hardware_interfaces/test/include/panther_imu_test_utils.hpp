// Copyright 2024 Husarion sp. z o.o.
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

#ifndef PANTHER_HARDWARE_INTERFACES_PANTHER_IMU_TEST_UTILS_HPP_
#define PANTHER_HARDWARE_INTERFACES_PANTHER_IMU_TEST_UTILS_HPP_

#include <cmath>
#include <cstdint>
#include <map>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/resource_manager.hpp>

#include <test_imu_constants.hpp>

namespace panther_hardware_interfaces_test
{

/**
 * @brief Utility class for testing Panther Imu
 */
class PantherImuTestUtils
{
public:
  PantherImuTestUtils()
  {
    default_panther_imu_urdf_ = BuildUrdf(kImuObligatoryParams, kImuInterfaces);
  }

  PantherImuTestUtils(const std::map<std::string, std::string>& param_map,
                      const std::list<std::string>& interfaces_list)
  {
    default_panther_imu_urdf_ = BuildUrdf(param_map, interfaces_list);
  }

  /**
   * @brief Starts Spacial Mock, initializes rclcpp and creates resource manager
   * @param urdf urdf used to create resource manager
   */
  void Start(const std::string& urdf);

  /**
   * @brief Shuts down rclcpp, stops Spacial mock and destroys resource manager
   */
  void Stop();

  /**
   * @brief Creates and returns URDF as a string
   * @param param_map map with hardware parameters
   */
  std::string BuildUrdf(const std::map<std::string, std::string>& param_map,
                        const std::list<std::string>& interfaces_list);

  hardware_interface::return_type ConfigurePantherImu();
  hardware_interface::return_type UnconfigurePantherImu();
  hardware_interface::return_type ActivatePantherImu();
  hardware_interface::return_type DeactivatePantherImu();
  hardware_interface::return_type ShutdownPantherImu();

  std::shared_ptr<hardware_interface::ResourceManager> GetResourceManager()
  {
    return rm_;
  }

  std::string GetDefaultPantherImuUrdf() const
  {
    return default_panther_imu_urdf_;
  }

private:
  /**
   * @brief Changes current state of the resource manager to the one set in parameters. It is
   * recommended to use wrapper functions
   * @param state_id
   * @param state_name
   */
  hardware_interface::return_type SetState(const std::uint8_t state_id, const std::string& state_name);

  std::shared_ptr<hardware_interface::ResourceManager> rm_;

  std::string default_panther_imu_urdf_;
};

}  // namespace panther_hardware_interfaces_test

#endif  // PANTHER_HARDWARE_INTERFACES_PANTHER_IMU_TEST_UTILS_HPP_
