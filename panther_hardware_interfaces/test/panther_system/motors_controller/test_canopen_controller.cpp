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

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <sys/socket.h>

#include <panther_hardware_interfaces/panther_system/motors_controller/canopen_controller.hpp>

#include "utils/test_constants.hpp"

class FakeCANSocket
{
public:
  FakeCANSocket(const std::string & can_interface_name)
  : can_interface_name_(can_interface_name), can_device_created_(false)
  {
  }

  ~FakeCANSocket() { Deinitialize(); }

  void Initialize()
  {
    std::cout << "Initializing vcan device" << std::endl;
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

    can_device_created_ = true;

    const auto command = "sudo ip link set up " + can_interface_name_;
    if (system(command.c_str()) != 0) {
      throw std::runtime_error("Failed to set up vcan device");
    }
  }

  void Deinitialize()
  {
    if (!can_device_created_) {
      return;
    }

    std::string command = "sudo ip link delete " + can_interface_name_;
    if (system(command.c_str()) != 0) {
      throw std::runtime_error("Failed to delete vcan device");
    }

    can_device_created_ = false;
  }

private:
  const std::string can_interface_name_;
  bool can_device_created_;
};

class TestCANopenController : public ::testing::Test
{
public:
  TestCANopenController();

  ~TestCANopenController() {}

protected:
  std::unique_ptr<FakeCANSocket> can_socket_;
  std::unique_ptr<panther_hardware_interfaces::CANopenController> canopen_controller_;
};

TestCANopenController::TestCANopenController()
{
  can_socket_ = std::make_unique<FakeCANSocket>(
    panther_hardware_interfaces_test::kCANopenSettings.can_interface_name);

  canopen_controller_ = std::make_unique<panther_hardware_interfaces::CANopenController>(
    panther_hardware_interfaces_test::kCANopenSettings);
}

TEST_F(TestCANopenController, InitializeAndDeinitialize)
{
  can_socket_->Initialize();

  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());

  // Check if deinitialization worked correctly - initialize once again
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());
}

TEST_F(TestCANopenController, InitializeWithError)
{
  // CAN socket not initialized, should throw
  ASSERT_THROW(canopen_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());

  can_socket_->Initialize();
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
