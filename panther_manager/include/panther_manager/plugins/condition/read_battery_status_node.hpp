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

#ifndef PANTHER_MANAGER__PLUGINS__CONDITION__READ_BATTERY_STATUS_NODE_HPP_
#define PANTHER_MANAGER__PLUGINS__CONDITION__READ_BATTERY_STATUS_NODE_HPP_

#include <behaviortree_ros2/bt_topic_sub_node.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include "panther_utils/moving_average.hpp"

namespace panther_manager
{
using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class ReadBatteryStatus : public BT::RosTopicSubNode<BatteryStateMsg>
{
public:
  ReadBatteryStatus(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onTick(const std::shared_ptr<BatteryStateMsg> & last_msg) override;

private:
  inline bool IsStatusKnown(const std::shared_ptr<BatteryStateMsg> & msg) const
  {
    if (
      msg->power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN ||
      msg->power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
      return false;
    } else
      return true;
  }

  std::unique_ptr<panther_utils::MovingAverage<double>> battery_percent_ma_;
  double battery_charging_anim_step_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER__PLUGINS__CONDITION__READ_BATTERY_STATUS_NODE_HPP_
