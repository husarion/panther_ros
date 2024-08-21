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

#include "utils/fake_can_socket.hpp"
#include "utils/test_constants.hpp"

class TestCANopenController : public ::testing::Test
{
public:
  TestCANopenController();

  ~TestCANopenController() {}

protected:
  std::unique_ptr<panther_hardware_interfaces_test::FakeCANSocket> can_socket_;
  std::unique_ptr<panther_hardware_interfaces::CANopenController> canopen_controller_;
};

TestCANopenController::TestCANopenController()
{
  can_socket_ = std::make_unique<panther_hardware_interfaces_test::FakeCANSocket>(
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
