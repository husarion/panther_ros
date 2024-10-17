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

#ifndef PANTHER_MANAGER_ROBOT_STATES_MANAGER_NODE_HPP_
#define PANTHER_MANAGER_ROBOT_STATES_MANAGER_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "panther_msgs/msg/led_animation.hpp"

#include "panther_utils/moving_average.hpp"

#include <panther_manager/behavior_tree_manager.hpp>

namespace panther_manager
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using BoolMsg = std_msgs::msg::Bool;
using Int8Msg = std_msgs::msg::Int8;
using SetBoolSrv = std_srvs::srv::SetBool;
using LEDAnimationMsg = panther_msgs::msg::LEDAnimation;

/**
 * @brief This class is responsible for creating a BehaviorTree responsible for docking management,
 * spinning it, and updating blackboard entries based on subscribed topics.
 */
class RobotStateManagerNode : public rclcpp::Node
{
public:
  RobotStateManagerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RobotStateManagerNode() {}

  void Initialize();

protected:
  void DeclareParameters();
  void RegisterBehaviorTree();
  std::map<std::string, std::any> CreateBlackboard();

  /**
   * @brief Checks whether the required blackboard entries for the docking tree are present. These
   * entries are usually updated when the first ROS message containing required information is
   * received.
   *
   * @return True if all required blackboard entries are present.
   */
  bool SystemReady();

  std::unique_ptr<BehaviorTreeManager> docking_tree_manager_;

private:
  void BatteryCB(const BatteryStateMsg::SharedPtr battery);
  void EStopCB(const BoolMsg::SharedPtr e_stop);
  void DockingSrvCB(
    const SetBoolSrv::Request::SharedPtr & request, SetBoolSrv::Response::SharedPtr response);
  void DockingTreeTimerCB();

  void PublishRobotState();

  float update_charging_anim_step_;

  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<BoolMsg>::SharedPtr e_stop_sub_;
  rclcpp::Publisher<Int8Msg>::SharedPtr robot_state_pub_;
  rclcpp::Service<SetBoolSrv>::SharedPtr docking_srv_;
  rclcpp::TimerBase::SharedPtr docking_tree_timer_;

  std::unique_ptr<panther_utils::MovingAverage<double>> battery_percent_ma_;
  BT::BehaviorTreeFactory factory_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_ROBOT_STATES_MANAGER_NODE_HPP_
