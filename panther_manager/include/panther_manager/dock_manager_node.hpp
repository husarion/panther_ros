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

#ifndef PANTHER_MANAGER_DOCK_MANAGER_NODE_HPP_
#define PANTHER_MANAGER_DOCK_MANAGER_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include <panther_manager/behavior_tree_manager.hpp>

namespace panther_manager
{

class DockManagerNode : public rclcpp::Node
{
public:
  DockManagerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DockManagerNode() {}

  void Initialize();

protected:
  void DeclareParameters();
  void RegisterBehaviorTree();
  std::map<std::string, std::any> CreateDockInitialBlackboard();
  bool SystemReady();
  void DockTreeTimerCB();

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BehaviorTreeManager> dock_tree_manager_;

private:
  rclcpp::TimerBase::SharedPtr dock_tree_timer_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_DOCK_MANAGER_NODE_HPP_
