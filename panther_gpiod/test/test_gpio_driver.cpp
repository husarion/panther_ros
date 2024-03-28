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

#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

#include "gpiod.hpp"
#include "gtest/gtest.h"

#include "panther_gpiod/gpio_driver.hpp"
#include "panther_utils/test/test_utils.hpp"

using namespace panther_gpiod;

class TestGPIODriver : public ::testing::Test
{
public:
  TestGPIODriver() = default;
  virtual ~TestGPIODriver() = default;

  void GPIOEventCallback(const GPIOInfo & gpio_info);

protected:
  void SetUp() override;
  void TearDown() override;
  void SetAndVerifyPinState(const GPIOPin & pin);

  std::unique_ptr<GPIODriver> gpio_driver_;
  std::pair<GPIOPin, gpiod::line::value> last_gpio_event_summary_;
  const std::vector<GPIOInfo> gpio_config_info_{
    GPIOInfo{GPIOPin::LED_SBC_SEL, gpiod::line::direction::OUTPUT},
  };
};

void TestGPIODriver::SetUp()
{
  gpio_driver_ = std::make_unique<GPIODriver>(gpio_config_info_);

  last_gpio_event_summary_.first = static_cast<GPIOPin>(-1);
  last_gpio_event_summary_.second = static_cast<gpiod::line::value>(-1);
}

void TestGPIODriver::TearDown() { gpio_driver_.reset(); }

void TestGPIODriver::GPIOEventCallback(const GPIOInfo & gpio_info)
{
  last_gpio_event_summary_.first = gpio_info.pin;
  last_gpio_event_summary_.second = gpio_info.value;
}

void TestGPIODriver::SetAndVerifyPinState(const GPIOPin & pin)
{
  EXPECT_TRUE(gpio_driver_->SetPinValue(pin, true));
  EXPECT_TRUE(gpio_driver_->IsPinActive(pin));

  EXPECT_TRUE(gpio_driver_->SetPinValue(pin, false));
  EXPECT_FALSE(gpio_driver_->IsPinActive(pin));
}

TEST(TestGPIODriverInitialization, EmptyInfoStorage)
{
  EXPECT_THROW(
    { auto gpio_driver = std::make_unique<GPIODriver>(std::vector<GPIOInfo>{}); },
    std::runtime_error);
}

TEST(TestGPIODriverInitialization, WrongPinConfigFail)
{
  // There is no OS version that supports simultaneous operation of MOTOR_ON and VMOT_ON pins.
  EXPECT_THROW(
    {
      auto gpio_driver = std::make_unique<GPIODriver>(
        std::vector<GPIOInfo>{{GPIOPin::MOTOR_ON, gpiod::line::direction::OUTPUT}});
      gpio_driver.reset();

      gpio_driver = std::make_unique<GPIODriver>(
        std::vector<GPIOInfo>{{GPIOPin::VMOT_ON, gpiod::line::direction::OUTPUT}});
    },
    std::invalid_argument);
}

TEST_F(TestGPIODriver, SetWrongPinValue)
{
  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::invalid_argument>(
    [&]() { this->gpio_driver_->SetPinValue(static_cast<GPIOPin>(-1), true); },
    "Pin not found in GPIO info storage."));
}

TEST_F(TestGPIODriver, IsPinAvailable)
{
  EXPECT_TRUE(this->gpio_driver_->IsPinAvailable(GPIOPin::LED_SBC_SEL));
  EXPECT_FALSE(this->gpio_driver_->IsPinAvailable(static_cast<GPIOPin>(-1)));
}

TEST_F(TestGPIODriver, GPIOMonitorEnableNoRT)
{
  this->gpio_driver_->GPIOMonitorEnable();

  SetAndVerifyPinState(GPIOPin::LED_SBC_SEL);
}

TEST_F(TestGPIODriver, GPIOMonitorEnableUseRT)
{
  // Redirect std::cerr warning to a stringstream
  std::stringstream buffer;
  std::streambuf * prev_cerr_buf = std::cerr.rdbuf(buffer.rdbuf());

  this->gpio_driver_->GPIOMonitorEnable(true);

  std::string captured_scheduling_rt_policy_waring = buffer.str();

  // Restore std::cerr to its previous buffer
  std::cerr.rdbuf(prev_cerr_buf);

  EXPECT_EQ(captured_scheduling_rt_policy_waring, "");

  SetAndVerifyPinState(GPIOPin::LED_SBC_SEL);
}

TEST_F(TestGPIODriver, GPIOEventCallbackFailWhenNoMonitorThread)
{
  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() {
      this->gpio_driver_->ConfigureEdgeEventCallback(
        std::bind(&TestGPIODriver::GPIOEventCallback, this, std::placeholders::_1));
    },
    "GPIO monitor thread is not running!"));
}

TEST_F(TestGPIODriver, GPIOEventCallbackShareNewPinState)
{
  auto tested_pin = GPIOPin::LED_SBC_SEL;

  this->gpio_driver_->GPIOMonitorEnable();
  this->gpio_driver_->ConfigureEdgeEventCallback(
    std::bind(&TestGPIODriver::GPIOEventCallback, this, std::placeholders::_1));

  EXPECT_TRUE(this->gpio_driver_->SetPinValue(tested_pin, true));
  EXPECT_TRUE(this->gpio_driver_->IsPinActive(tested_pin));

  EXPECT_EQ(this->last_gpio_event_summary_.first, tested_pin);
  EXPECT_EQ(this->last_gpio_event_summary_.second, gpiod::line::value::ACTIVE);

  EXPECT_TRUE(this->gpio_driver_->SetPinValue(tested_pin, false));
  EXPECT_FALSE(this->gpio_driver_->IsPinActive(tested_pin));

  EXPECT_EQ(this->last_gpio_event_summary_.first, tested_pin);
  EXPECT_EQ(this->last_gpio_event_summary_.second, gpiod::line::value::INACTIVE);
}

TEST_F(TestGPIODriver, ChangePinDirection)
{
  this->gpio_driver_->GPIOMonitorEnable();
  this->gpio_driver_->ChangePinDirection(GPIOPin::LED_SBC_SEL, gpiod::line::direction::INPUT);

  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::invalid_argument>(
    [&]() { this->gpio_driver_->SetPinValue(GPIOPin::LED_SBC_SEL, true); },
    "Cannot set value for INPUT pin."));

  this->gpio_driver_->ChangePinDirection(GPIOPin::LED_SBC_SEL, gpiod::line::direction::OUTPUT);

  SetAndVerifyPinState(GPIOPin::LED_SBC_SEL);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
