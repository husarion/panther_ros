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

#ifndef PANTHER_HARDWARE_INTERFACES_TEST_UTILS_MOCK_DRIVER_HPP_
#define PANTHER_HARDWARE_INTERFACES_TEST_UTILS_MOCK_DRIVER_HPP_

#include <cstdint>
#include <future>
#include <memory>
#include <string>

#include <gmock/gmock.h>

#include "panther_hardware_interfaces/panther_system/robot_driver/driver.hpp"

namespace panther_hardware_interfaces_test
{

class MockDriver : public panther_hardware_interfaces::Driver
{
public:
  MOCK_METHOD(std::future<void>, Boot, (), (override));
  MOCK_METHOD(bool, IsCANError, (), (const, override));
  MOCK_METHOD(bool, IsHeartbeatTimeout, (), (const, override));

  MOCK_METHOD(panther_hardware_interfaces::DriverState, ReadDriverState, (), (override));
  MOCK_METHOD(void, ResetScript, (), (override));
  MOCK_METHOD(void, TurnOnEStop, (), (override));
  MOCK_METHOD(void, TurnOffEStop, (), (override));

  std::shared_ptr<panther_hardware_interfaces::MotorDriver> GetMotorDriver(
    const std::string & name) override
  {
    return motor_drivers_.at(name);
  }

  void AddMotorDriver(
    const std::string name,
    std::shared_ptr<panther_hardware_interfaces::MotorDriver> motor_driver) override
  {
    motor_drivers_.emplace(name, motor_driver);
  }

private:
  std::map<std::string, std::shared_ptr<panther_hardware_interfaces::MotorDriver>> motor_drivers_;
};

class MockMotorDriver : public panther_hardware_interfaces::MotorDriver
{
public:
  MOCK_METHOD(panther_hardware_interfaces::MotorDriverState, ReadMotorDriverState, (), (override));
  MOCK_METHOD(void, SendCmdVel, (const std::int32_t cmd), (override));
  MOCK_METHOD(void, TurnOnSafetyStop, (), (override));
};

}  // namespace panther_hardware_interfaces_test

#endif  // PANTHER_HARDWARE_INTERFACES_TEST_UTILS_MOCK_DRIVER_HPP_
