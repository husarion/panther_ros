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

#ifndef PANTHER_MANAGER_BEHAVIOR_TREE_MANAGER_HPP_
#define PANTHER_MANAGER_BEHAVIOR_TREE_MANAGER_HPP_

#include <any>
#include <map>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"

namespace panther_manager
{

/**
 * @brief Class responsible for managing behavior trees.
 */
class BehaviorTreeManager
{
public:
  /**
   * @brief Constructor for the BehaviorTreeManager class.
   *
   * @param tree_name The name of the tree in the BehaviorTree project.
   * @param initial_blackboard A list with initial blackboard values for the tree.
   * @param groot_port The port used for the Groot2 publisher.
   */
  BehaviorTreeManager(
    const std::string & tree_name, const std::map<std::string, std::any> & initial_blackboard,
    const unsigned groot_port = 1667)
  : tree_name_(tree_name),
    initial_blackboard_(initial_blackboard),
    groot_port_(groot_port),
    tree_status_(BT::NodeStatus::IDLE)
  {
  }

  ~BehaviorTreeManager() {}

  /**
   * @brief Creates a BehaviorTree configuration, initializes the tree, and starts the Groot2
   * publisher.
   *
   * @param factory The factory object used to create the tree.
   */
  void Initialize(BT::BehaviorTreeFactory & factory);

  void TickOnce() { tree_status_ = tree_.tickOnce(); }
  void TickExactlyOnce() { tree_status_ = tree_.tickExactlyOnce(); }
  void TickWhileRunning() { tree_status_ = tree_.tickWhileRunning(); }
  void HaltTree() { tree_.haltTree(); }

  BT::NodeStatus GetTreeStatus() const { return tree_status_; }
  BT::Tree & GetTree() { return tree_; }
  BT::Blackboard::Ptr GetBlackboard() const { return config_.blackboard; }

protected:
  /**
   * @brief Creates a BehaviorTree configuration using a set of predefined blackboard values.
   *
   * @param bb_values A map containing the names of the blackboard entries and their initial values.
   * This map can include different entry types. Supported types are: bool, int, unsigned, float,
   * double, const char*, and string.
   * @exception std::invalid_argument thrown when the bb_values map contains an invalid blackboard
   * entry type.
   * @return A BehaviorTree configuration object.
   */
  BT::NodeConfig CreateBTConfig(const std::map<std::string, std::any> & bb_values) const;

private:
  const std::string tree_name_;
  const std::map<std::string, std::any> initial_blackboard_;
  const unsigned groot_port_;

  BT::Tree tree_;
  BT::NodeStatus tree_status_;
  BT::NodeConfig config_;
  std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_BEHAVIOR_TREE_MANAGER_HPP_
