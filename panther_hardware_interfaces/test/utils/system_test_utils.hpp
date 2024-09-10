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

#ifndef PANTHER_HARDWARE_INTERFACES_TEST_SYSTEM_TEST_UTILS_HPP_
#define PANTHER_HARDWARE_INTERFACES_TEST_SYSTEM_TEST_UTILS_HPP_

#include <cstdint>
#include <future>
#include <memory>
#include <string>

#include <gmock/gmock.h>

#include "panther_hardware_interfaces/panther_system/gpio/gpio_controller.hpp"
#include "panther_hardware_interfaces/panther_system/robot_driver/robot_driver.hpp"
#include "panther_hardware_interfaces/panther_system/system_e_stop.hpp"

#include "utils/mock_driver.hpp"

namespace panther_hardware_interfaces_test
{

class MockRobotDriver : public panther_hardware_interfaces::RobotDriver
{
public:
  MOCK_METHOD(void, Initialize, (), (override));
  MOCK_METHOD(void, Deinitialize, (), (override));
  MOCK_METHOD(void, Activate, (), (override));
  MOCK_METHOD(void, UpdateCommunicationState, (), (override));
  MOCK_METHOD(void, UpdateMotorsState, (), (override));
  MOCK_METHOD(void, UpdateDriversState, (), (override));
  MOCK_METHOD(
    const panther_hardware_interfaces::RoboteqData &, GetData, (const std::string &), (override));
  MOCK_METHOD(void, SendSpeedCommands, (const std::vector<float> &), (override));
  MOCK_METHOD(void, TurnOnEStop, (), (override));
  MOCK_METHOD(void, TurnOffEStop, (), (override));
  MOCK_METHOD(void, TurnOnSafetyStop, (), (override));
  MOCK_METHOD(void, AttemptErrorFlagResetWithZeroSpeed, (), (override));
};

class MockGPIODriver : public panther_hardware_interfaces::GPIODriverInterface
{
public:
  MOCK_METHOD(void, GPIOMonitorEnable, (const bool, const unsigned), (override));
  MOCK_METHOD(
    void, ConfigureEdgeEventCallback,
    (const std::function<void(const panther_hardware_interfaces::GPIOInfo &)> &), (override));
  MOCK_METHOD(
    void, ChangePinDirection,
    (const panther_hardware_interfaces::GPIOPin, const gpiod::line::direction), (override));
  MOCK_METHOD(
    bool, IsPinAvailable, (const panther_hardware_interfaces::GPIOPin), (const, override));
  MOCK_METHOD(bool, IsPinActive, (const panther_hardware_interfaces::GPIOPin), (override));
  MOCK_METHOD(
    bool, SetPinValue, (const panther_hardware_interfaces::GPIOPin, const bool), (override));
};

class MockGPIOController : public panther_hardware_interfaces::GPIOControllerInterface
{
public:
  MockGPIOController() : GPIOControllerInterface()
  {
    gpio_driver_ = std::make_shared<::testing::NiceMock<MockGPIODriver>>();
  }

  MOCK_METHOD(void, Start, (), (override));
  MOCK_METHOD(void, EStopTrigger, (), (override));
  MOCK_METHOD(void, EStopReset, (), (override));
  MOCK_METHOD(bool, MotorPowerEnable, (const bool), (override));
  MOCK_METHOD(bool, FanEnable, (const bool), (override));
  MOCK_METHOD(bool, AUXPowerEnable, (const bool), (override));
  MOCK_METHOD(bool, DigitalPowerEnable, (const bool), (override));
  MOCK_METHOD(bool, ChargerEnable, (const bool), (override));
  MOCK_METHOD(bool, LEDControlEnable, (const bool), (override));
  MOCK_METHOD(
    (std::unordered_map<panther_hardware_interfaces::GPIOPin, bool>), QueryControlInterfaceIOStates,
    (), (const, override));
};

class MockEStop : public panther_hardware_interfaces::EStopInterface
{
public:
  MockEStop() : EStopInterface() {}

  MOCK_METHOD(bool, ReadEStopState, (), (override));
  MOCK_METHOD(void, TriggerEStop, (), (override));
  MOCK_METHOD(void, ResetEStop, (), (override));
};

}  // namespace panther_hardware_interfaces_test

#endif  // PANTHER_HARDWARE_INTERFACES_TEST_SYSTEM_TEST_UTILS_HPP_
