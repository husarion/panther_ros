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

#ifndef PANTHER_MANAGER_PLUGINS_DOCK_ROBOT_ACTION_NODE_HPP_
#define PANTHER_MANAGER_PLUGINS_DOCK_ROBOT_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_ros2/bt_action_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opennav_docking_msgs/action/dock_robot.hpp>

namespace panther_manager
{

class DockRobot : public BT::RosActionNode<opennav_docking_msgs::action::DockRobot>
{
  using Action = opennav_docking_msgs::action::DockRobot;
  using ActionResult = Action::Result;

public:
  DockRobot(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : RosActionNode<Action>(name, conf, params)
  {
  }

  bool setGoal(Goal & goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<bool>("use_dock_id", true, "Whether to use the dock's ID or dock pose fields"),
      BT::InputPort<std::string>("dock_id", "Dock ID or name to use"),
      BT::InputPort<std::string>("dock_type", "The dock plugin type, if using dock pose"),
      BT::InputPort<float>(
        "max_staging_time", 1000.0, "Maximum time to navigate to the staging pose"),
      BT::InputPort<bool>(
        "navigate_to_staging_pose", true, "Whether to autonomously navigate to staging pose"),

      BT::OutputPort<ActionResult::_success_type>("success", "If the action was successful"),
      BT::OutputPort<ActionResult::_error_code_type>("error_code", "Contextual error code, if any"),
      BT::OutputPort<ActionResult::_num_retries_type>("num_retries", "Number of retries attempted"),
    });
  }
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_PLUGINS_DOCK_ROBOT_ACTION_NODE_HPP_
