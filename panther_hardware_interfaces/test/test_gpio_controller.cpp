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

#include <cstdlib>

#include <gtest/gtest.h>
#include <gpiod.hpp>

#include <panther_hardware_interfaces/gpio_controller.hpp>

const std::vector<panther_gpiod::GPIOInfo> gpio_config_info_storage{
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::WATCHDOG, gpiod::line::direction::OUTPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::AUX_PW_EN, gpiod::line::direction::OUTPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::CHRG_DISABLE, gpiod::line::direction::OUTPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::DRIVER_EN, gpiod::line::direction::OUTPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::E_STOP_RESET, gpiod::line::direction::INPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::FAN_SW, gpiod::line::direction::OUTPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::GPOUT1, gpiod::line::direction::OUTPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::GPOUT2, gpiod::line::direction::OUTPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::GPIN1, gpiod::line::direction::INPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::GPIN2, gpiod::line::direction::INPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::SHDN_INIT, gpiod::line::direction::INPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::VDIG_OFF, gpiod::line::direction::OUTPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::VMOT_ON, gpiod::line::direction::OUTPUT},
  panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::CHRG_SENSE, gpiod::line::direction::INPUT},
};

class GPIOControllerWrapper : public panther_hardware_interfaces::GPIOControllerPTH12X
{
public:
  void WatchdogEnable() { this->watchdog_->TurnOn(); }
  void WatchdogDisable() { this->watchdog_->TurnOff(); }
  bool IsWatchdogEnabled() { return this->watchdog_->IsWatchdogEnabled(); }
};

class TestGPIOController : public ::testing::Test
{
public:
  TestGPIOController();
  ~TestGPIOController() { gpio_controller_wrapper_.reset(); }

protected:
  float GetRobotVersion();

  std::unique_ptr<GPIOControllerWrapper> gpio_controller_wrapper_;
  static constexpr int watchdog_edges_per_100ms_ = 10;
};

TestGPIOController::TestGPIOController()
{
  if (GetRobotVersion() < 1.2 - std::numeric_limits<float>::epsilon()) {
    throw std::runtime_error("Tests for this robot versions are not implemented");
  }

  gpio_controller_wrapper_ = std::make_unique<GPIOControllerWrapper>();
  gpio_controller_wrapper_->Start();
}

float TestGPIOController::GetRobotVersion()
{
  const char * robot_version_env = std::getenv("PANTHER_ROBOT_VERSION");

  if (!robot_version_env) {
    throw std::runtime_error("Can't read 'PANTHER_ROBOT_VERSION' environment variable");
  }

  return std::stof(robot_version_env);
}

TEST_F(TestGPIOController, TestMotorsInit)
{
  EXPECT_TRUE(gpio_controller_wrapper_->IsPinActive(panther_gpiod::GPIOPin::VMOT_ON));
  EXPECT_TRUE(gpio_controller_wrapper_->IsPinActive(panther_gpiod::GPIOPin::DRIVER_EN));
}

TEST_F(TestGPIOController, TestWatchdog)
{
  auto edge_cnt = std::make_shared<int>(0);

  gpio_controller_wrapper_->RegisterGPIOEventCallback([this, edge_cnt](const auto & state) mutable {
    if (state.pin == panther_gpiod::GPIOPin::WATCHDOG) {
      (*edge_cnt)++;
    }
  });

  gpio_controller_wrapper_->WatchdogEnable();
  ASSERT_TRUE(gpio_controller_wrapper_->IsWatchdogEnabled());

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  gpio_controller_wrapper_->WatchdogDisable();

  ASSERT_FALSE(gpio_controller_wrapper_->IsWatchdogEnabled());
  EXPECT_EQ(*edge_cnt, watchdog_edges_per_100ms_ + 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(*edge_cnt, watchdog_edges_per_100ms_ + 1);
}

TEST_F(TestGPIOController, TestPinsAvailability)
{
  for (const auto & info : gpio_config_info_storage) {
    EXPECT_TRUE(gpio_controller_wrapper_->IsPinAvailable(info.pin));
  }
}

TEST_F(TestGPIOController, TestFanEnbale)
{
  gpio_controller_wrapper_->FanEnable(true);
  EXPECT_TRUE(gpio_controller_wrapper_->IsPinActive(panther_gpiod::GPIOPin::FAN_SW));

  gpio_controller_wrapper_->FanEnable(false);
  EXPECT_FALSE(gpio_controller_wrapper_->IsPinActive(panther_gpiod::GPIOPin::FAN_SW));
}

TEST_F(TestGPIOController, TestAUXPowerEnbale)
{
  gpio_controller_wrapper_->AUXPowerEnable(true);
  EXPECT_TRUE(gpio_controller_wrapper_->IsPinActive(panther_gpiod::GPIOPin::AUX_PW_EN));

  gpio_controller_wrapper_->AUXPowerEnable(false);
  EXPECT_FALSE(gpio_controller_wrapper_->IsPinActive(panther_gpiod::GPIOPin::AUX_PW_EN));
}

TEST_F(TestGPIOController, TestChargerEnable)
{
  gpio_controller_wrapper_->ChargerEnable(true);
  EXPECT_FALSE(gpio_controller_wrapper_->IsPinActive(panther_gpiod::GPIOPin::CHRG_DISABLE));

  gpio_controller_wrapper_->ChargerEnable(false);
  EXPECT_TRUE(gpio_controller_wrapper_->IsPinActive(panther_gpiod::GPIOPin::CHRG_DISABLE));
}

TEST_F(TestGPIOController, TestQueryControlInterfaceIOStates)
{
  std::unordered_map<panther_gpiod::GPIOPin, bool> io_states =
    gpio_controller_wrapper_->QueryControlInterfaceIOStates();

  ASSERT_EQ(io_states.size(), 7);

  for (const auto & [pin, expected_state] : io_states) {
    bool actual_state = gpio_controller_wrapper_->IsPinActive(pin);
    EXPECT_EQ(expected_state, actual_state);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
