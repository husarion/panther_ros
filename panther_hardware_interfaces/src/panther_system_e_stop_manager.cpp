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

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "panther_hardware_interfaces/panther_system_e_stop_manager.hpp"

namespace panther_hardware_interfaces
{

void EStopStrategy::SetManagerResources(std::shared_ptr<EStopManagerResources> resources)
{
  manager_resources_ = resources;
}

bool EStopStrategy::ConfirmEStopResetSuccessful()
{
  // TODO: @pkowalsk1 test and/or update this strategy
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return !ReadEStopState();
}

bool EStopStrategyPTH12X::ReadEStopState()
{
  e_stop_triggered_ =
    !manager_resources_->gpio_controller->IsPinActive(panther_gpiod::GPIOPin::E_STOP_RESET);

  // In the case where E-Stop is triggered by another device within the robot's system (e.g.,
  // Roboteq or Safety Board), disabling the software Watchdog is necessary to prevent an
  // uncontrolled reset.
  if (e_stop_triggered_) {
    manager_resources_->gpio_controller->EStopTrigger();
  }

  return e_stop_triggered_;
}

void EStopStrategyPTH12X::TriggerEStop()
{
  manager_resources_->gpio_controller->InterruptEStopReset();

  std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);
  try {
    manager_resources_->gpio_controller->EStopTrigger();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Setting E-stop failed: " + std::string(e.what()));
  }

  e_stop_triggered_ = true;
}

void EStopStrategyPTH12X::ResetEStop()
{
  if (e_stop_manipulation_mtx_.try_lock()) {
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_, std::adopt_lock);

    if (!manager_resources_->motors_controller->AreVelocityCommandsNearZero()) {
      throw std::runtime_error(
        "Cannot reset the E-stop: the last velocity commands are not zero. Ensure that the "
        "controller is sending zero velocity commands or not sending any velocity commands before "
        "attempting to reset the E-stop.");
    }

    try {
      manager_resources_->gpio_controller->EStopReset();
    } catch (const EStopResetInterrupted & e) {
      throw std::runtime_error(
        "The E-stop reset operation was halted because the E-stop was triggered again.");
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Error when trying to reset E-stop using GPIO: " + std::string(e.what()));
    }

    manager_resources_->roboteq_error_filter->SetClearErrorsFlag();
  } else {
    std::fprintf(stdout, "E-stop trigger operation is in progress, skipping reset.");
  }

  if (ConfirmEStopResetSuccessful()) {
    e_stop_triggered_ = false;
  }
}

bool EStopStrategyPTH10X::ReadEStopState()
{
  const bool motors_on =
    manager_resources_->gpio_controller->IsPinActive(panther_gpiod::GPIOPin::STAGE2_INPUT);
  const bool driver_error = manager_resources_->roboteq_error_filter->IsError();

  if ((driver_error || !motors_on) && !e_stop_triggered_) {
    TriggerEStop();
  }

  return e_stop_triggered_;
}

void EStopStrategyPTH10X::TriggerEStop()
{
  std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);
  std::lock_guard<std::mutex> lck_g(*(manager_resources_->motor_controller_write_mtx));

  try {
    manager_resources_->motors_controller->TurnOnSafetyStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to set safety stop using CAN command: " + std::string(e.what()));
  }

  e_stop_triggered_ = true;
}

void EStopStrategyPTH10X::ResetEStop()
{
  if (e_stop_manipulation_mtx_.try_lock()) {
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_, std::adopt_lock);

    if (!manager_resources_->motors_controller->AreVelocityCommandsNearZero()) {
      throw std::runtime_error(
        "Can't reset E-stop - last velocity commands are different than zero. Make sure that your "
        "controller sends zero commands before trying to reset E-stop.");
    }

    if (!manager_resources_->gpio_controller->IsPinActive(panther_gpiod::GPIOPin::STAGE2_INPUT)) {
      throw std::runtime_error("Can't reset E-stop - motors are not powered.");
    }

    if (!manager_resources_->roboteq_error_filter->IsError()) {
      throw std::runtime_error("Can't reset E-stop - motor controller is in an error state.");
    }

    manager_resources_->roboteq_error_filter->SetClearErrorsFlag();
  } else {
    std::fprintf(stdout, "E-stop trigger operation is in progress, skipping reset.");
  }

  if (ConfirmEStopResetSuccessful()) {
    e_stop_triggered_ = false;
  }
}

void EStopManager::SetStrategy(std::unique_ptr<EStopStrategy> && strategy)
{
  strategy_ = std::move(strategy);
  strategy_->SetManagerResources(resources_);
}

void EStopManager::TriggerEStop() { strategy_->TriggerEStop(); };

void EStopManager::ResetEStop() { strategy_->ResetEStop(); };

bool EStopManager::ReadEStopState() { return strategy_->ReadEStopState(); };

}  // namespace panther_hardware_interfaces
