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

#include "std_msgs/msg/bool.hpp"

#include "panther_msgs/msg/robot_state.hpp"
#include "panther_utils/moving_average.hpp"

#include <panther_manager/behavior_tree_manager.hpp>

namespace panther_manager
{

using BoolMsg = std_msgs::msg::Bool;
using RobotStateMsg = panther_msgs::msg::RobotState;

/**
 * @brief This class is responsible for creating a BehaviorTree responsible for docking management,
 * spinning it, and updating blackboard entries based on subscribed topics.
 */
class RobotStatesManagerNode : public rclcpp::Node
{
public:
  RobotStatesManagerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RobotStatesManagerNode() {}

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
  void EStopCB(const BoolMsg::SharedPtr e_stop);
  void RobotStatesTimerCB();

  void PublishRobotStateMsg();
  RobotStateMsg CreateRobotStateMsg(int8_t state_id);

  rclcpp::Subscription<BoolMsg>::SharedPtr e_stop_sub_;
  rclcpp::Publisher<RobotStateMsg>::SharedPtr robot_state_pub_;
  rclcpp::TimerBase::SharedPtr docking_tree_timer_;

  BT::BehaviorTreeFactory factory_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_ROBOT_STATES_MANAGER_NODE_HPP_
