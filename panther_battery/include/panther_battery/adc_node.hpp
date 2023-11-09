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

#ifndef PANTHER_BATTERY_ADC_NODE_HPP_
#define PANTHER_BATTERY_ADC_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_battery/adc_data_reader.hpp>
#include <panther_battery/battery.hpp>
#include <panther_battery/battery_publisher.hpp>

namespace panther_battery
{

class ADCNode : public rclcpp::Node
{
public:
  ADCNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void BatteryPubTimerCB();
  void Initialize();

  static constexpr int kADCCurrentOffest = 625;

  std::shared_ptr<ADCDataReader> adc0_reader_;
  std::shared_ptr<ADCDataReader> adc1_reader_;
  std::shared_ptr<Battery> battery_1_;
  std::shared_ptr<Battery> battery_2_;
  std::shared_ptr<BatteryPublisher> battery_publisher_;

  rclcpp::TimerBase::SharedPtr battery_pub_timer_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ADC_NODE_HPP_
