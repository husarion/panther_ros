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

class MockRobotDriver : public panther_hardware_interfaces::RobotDriverInterface
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
  MOCK_METHOD(void, AttemptErrorFlagReset, (), (override));
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

hardware_interface::HardwareInfo GenerateDefaultHardwareInfo()
{
  hardware_interface::HardwareInfo hardware_info;
  hardware_info.name = "test";
  hardware_info.hardware_class_type = "UGVSystem";

  hardware_interface::InterfaceInfo vel_command_interface;
  vel_command_interface.name = "velocity";
  hardware_interface::InterfaceInfo pos_state_interface;
  pos_state_interface.name = "position";
  hardware_interface::InterfaceInfo vel_state_interface;
  vel_state_interface.name = "velocity";
  hardware_interface::InterfaceInfo eff_state_interface;
  eff_state_interface.name = "effort";

  hardware_interface::ComponentInfo wheel_joint;
  wheel_joint.command_interfaces = {vel_command_interface};
  wheel_joint.state_interfaces = {pos_state_interface, vel_state_interface, eff_state_interface};

  auto fl_wheel_joint = wheel_joint;
  fl_wheel_joint.name = "fl_wheel_joint";
  auto fr_wheel_joint = wheel_joint;
  fr_wheel_joint.name = "fr_wheel_joint";
  auto rl_wheel_joint = wheel_joint;
  rl_wheel_joint.name = "rl_wheel_joint";
  auto rr_wheel_joint = wheel_joint;
  rr_wheel_joint.name = "rr_wheel_joint";

  hardware_info.joints = {fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint};

  std::unordered_map<std::string, std::string> hardware_paremeters = {
    // drivetrain settings
    {"motor_torque_constant", "0.11"},
    {"gear_ratio", "30.08"},
    {"gearbox_efficiency", "0.75"},
    {"encoder_resolution", "1600"},
    {"max_rpm_motor_speed", "3600.0"},

    // CANopen settings
    {"can_interface_name", "panther_can"},
    {"master_can_id", "3"},
    {"pdo_motor_states_timeout_ms", "15"},
    {"pdo_driver_state_timeout_ms", "75"},
    {"sdo_operation_timeout_ms", "100"},

    // Driver states update frequency
    {"driver_states_update_frequency", "20.0"},

    // Roboteq initialization and activation attempts
    {"max_roboteq_initialization_attempts", "5"},
    {"max_roboteq_activation_attempts", "5"},

    // Roboteq error filter params
    {"max_write_pdo_cmds_errors_count", "4"},
    {"max_read_pdo_motor_states_errors_count", "4"},
    {"max_read_pdo_driver_state_errors_count", "20"},
  };

  hardware_info.hardware_parameters = hardware_paremeters;

  return hardware_info;
}

}  // namespace panther_hardware_interfaces_test

#endif  // PANTHER_HARDWARE_INTERFACES_TEST_SYSTEM_TEST_UTILS_HPP_
