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
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/canopen_controller.hpp>

#include <roboteqs_mock.hpp>
#include <test_constants.hpp>

class TestCANopenController : public ::testing::Test
{
public:
  TestCANopenController()
  {
    canopen_controller_ = std::make_unique<panther_hardware_interfaces::CANopenController>(
      panther_hardware_interfaces_test::kCANopenSettings);

    roboteqs_mock_ = std::make_unique<panther_hardware_interfaces_test::RoboteqsMock>();
    roboteqs_mock_->Start(std::chrono::milliseconds(10), std::chrono::milliseconds(50));
  }

  ~TestCANopenController()
  {
    roboteqs_mock_->Stop();
    roboteqs_mock_.reset();
  }

protected:
  std::unique_ptr<panther_hardware_interfaces_test::RoboteqsMock> roboteqs_mock_;
  std::unique_ptr<panther_hardware_interfaces::CANopenController> canopen_controller_;
};

TEST_F(TestCANopenController, InitializeAndDeinitialize)
{
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());

  // Check if deinitialization worked correctly - initialize once again
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());
}

TEST_F(TestCANopenController, InitializeErrorDeviceType)
{
  roboteqs_mock_->GetFrontDriver()->SetOnReadWait<std::uint32_t>(0x1000, 0, 100000);
  ASSERT_THROW(canopen_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());

  roboteqs_mock_->GetFrontDriver()->SetOnReadWait<std::uint32_t>(0x1000, 0, 0);
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());
}

TEST_F(TestCANopenController, InitializeErrorVendorId)
{
  roboteqs_mock_->GetRearDriver()->SetOnReadWait<std::uint32_t>(0x1018, 1, 100000);
  ASSERT_THROW(canopen_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());

  roboteqs_mock_->GetRearDriver()->SetOnReadWait<std::uint32_t>(0x1018, 1, 0);
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());
}

TEST(TestCANopenControllerOthers, BootTimeout)
{
  std::unique_ptr<panther_hardware_interfaces::CANopenController> canopen_controller_;

  canopen_controller_ = std::make_unique<panther_hardware_interfaces::CANopenController>(
    panther_hardware_interfaces_test::kCANopenSettings);

  // No roboteq mock, so it won't be possible to boot - here is checked if after some time it will
  // finish with exception
  ASSERT_THROW(canopen_controller_->Initialize(), std::runtime_error);

  canopen_controller_->Deinitialize();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
