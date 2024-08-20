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

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sys/socket.h>

#include <panther_hardware_interfaces/panther_system/motors_controller/canopen_controller.hpp>

#include "utils/test_constants.hpp"

class MockRoboteqDriver : public panther_hardware_interfaces::RoboteqDriverInterface
{
public:
  MOCK_METHOD(std::future<void>, Boot, (), (override));

  MOCK_METHOD(bool, IsCANError, (), (const, override));
  MOCK_METHOD(bool, IsHeartbeatTimeout, (), (const, override));

  MOCK_METHOD(
    panther_hardware_interfaces::RoboteqMotorsStates, ReadRoboteqMotorsStates, (), (override));
  MOCK_METHOD(
    panther_hardware_interfaces::RoboteqDriverState, ReadRoboteqDriverState, (), (override));

  MOCK_METHOD(
    void, SendRoboteqCmd, (const std::int32_t cmd_channel_1, const std::int32_t cmd_channel_2),
    (override));
  MOCK_METHOD(void, ResetRoboteqScript, (), (override));
  MOCK_METHOD(void, TurnOnEStop, (), (override));
  MOCK_METHOD(void, TurnOffEStop, (), (override));
  MOCK_METHOD(void, TurnOnSafetyStopChannel1, (), (override));
  MOCK_METHOD(void, TurnOnSafetyStopChannel2, (), (override));
};

class MockCANopenController : public panther_hardware_interfaces::CANopenController
{
public:
  MockCANopenController(const panther_hardware_interfaces::CANopenSettings & canopen_settings)
  : CANopenController(canopen_settings)
  {
  }

  MOCK_METHOD(void, BootDrivers, (), (override));
  MOCK_METHOD(void, InitializeDrivers, (), (override));
  MOCK_METHOD(
    std::shared_ptr<panther_hardware_interfaces::RoboteqDriverInterface>, GetDriver,
    (const std::string & name), (override));
};

class TestCANopenController : public ::testing::Test
{
public:
  TestCANopenController()
  : can_interface_name_(panther_hardware_interfaces_test::kCANopenSettings.can_interface_name)
  {
    InitializeCANSocket();

    canopen_controller_ =
      std::make_unique<MockCANopenController>(panther_hardware_interfaces_test::kCANopenSettings);

    ON_CALL(*canopen_controller_, BootDrivers()).WillByDefault(testing::Return());
    ON_CALL(*canopen_controller_, InitializeDrivers()).WillByDefault(testing::Return());
    ON_CALL(*canopen_controller_, GetDriver(testing::_))
      .WillByDefault(testing::Return(std::make_shared<MockRoboteqDriver>()));
  }

  ~TestCANopenController() { DeinitializeCANSocket(); }

protected:
  std::unique_ptr<MockCANopenController> canopen_controller_;

  void InitializeCANSocket()
  {
    if (system("sudo modprobe vcan") != 0) {
      throw std::runtime_error("Failed to load vcan module");
    }

    // if link already exists, do not create it
    const auto check_command = "ip link show " + can_interface_name_ + " > /dev/null 2>&1";
    if (std::system(check_command.c_str()) != 0) {
      const auto command = "sudo ip link add dev " + can_interface_name_ + " type vcan";
      if (system(command.c_str()) != 0) {
        throw std::runtime_error("Failed to add vcan device");
      }
    }

    const auto command = "sudo ip link set up " + can_interface_name_;
    if (system(command.c_str()) != 0) {
      throw std::runtime_error("Failed to set up vcan device");
    }
  }

  void DeinitializeCANSocket()
  {
    std::string command = "sudo ip link set down " + can_interface_name_;
    if (system(command.c_str()) != 0) {
      throw std::runtime_error("Failed to delete vcan device");
    }
  }

  const std::string can_interface_name_;
};

TEST_F(TestCANopenController, InitializeAndDeinitialize)
{
  EXPECT_CALL(*canopen_controller_, InitializeDrivers()).Times(1);
  EXPECT_CALL(*canopen_controller_, BootDrivers()).Times(1);
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());

  // Check if deinitialization worked correctly - initialize once again
  EXPECT_CALL(*canopen_controller_, InitializeDrivers()).Times(1);
  EXPECT_CALL(*canopen_controller_, BootDrivers()).Times(1);
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());
}

TEST_F(TestCANopenController, InitializeWithError)
{
  ON_CALL(*canopen_controller_, BootDrivers())
    .WillByDefault(testing::Throw(std::runtime_error("")));
  EXPECT_CALL(*canopen_controller_, InitializeDrivers()).Times(1);
  EXPECT_CALL(*canopen_controller_, BootDrivers()).Times(0);
  ASSERT_THROW(canopen_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());

  ON_CALL(*canopen_controller_, BootDrivers()).WillByDefault(testing::Return());
  EXPECT_CALL(*canopen_controller_, InitializeDrivers()).Times(1);
  EXPECT_CALL(*canopen_controller_, BootDrivers()).Times(1);
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());
}

// // TODO Tests for concrete implementations of CANopenController
// TEST(TestCANopenControllerOthers, BootTimeout)
// {
//   std::unique_ptr<panther_hardware_interfaces::CANopenController> canopen_controller_;

//   canopen_controller_ = std::make_unique<panther_hardware_interfaces::CANopenController>(
//     panther_hardware_interfaces_test::kCANopenSettings);

//   // No roboteq mock, so it won't be possible to boot - here is checked if after some time it
//   will
//   // finish with exception
//   ASSERT_THROW(canopen_controller_->Initialize(), std::runtime_error);

//   canopen_controller_->Deinitialize();
// }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
