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

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"

namespace panther_manager
{

/**
 * @brief todo
 *
 * @param project_path The path to the BehaviorTree project file.
 * @param plugin_libs A vector containing the names of the nodes that will be registered from
 * @param ros_plugin_libs A vector containing the names of the ROS nodes that will be registered
 * from plugins.
 */
struct BehaviorTreeParams
{
  std::string project_path;
  std::vector<std::string> plugin_libs;
  std::vector<std::string> ros_plugin_libs;
  std::string tree_name;
  std::map<std::string, std::any> initial_blackboard;
  unsigned groot_port = 1667;
};

/**
 * @brief todo
 */
class BehaviorTreeManager
{
public:
  BehaviorTreeManager(const BehaviorTreeParams & params)
  : project_path_(params.project_path),
    plugin_libs_(params.plugin_libs),
    ros_plugin_libs_(params.ros_plugin_libs),
    tree_name_(params.tree_name),
    initial_blackboard_(params.initial_blackboard),
    groot_port_(params.groot_port)
  {
  }
  ~BehaviorTreeManager() {}

  // void Initialize();
  void Initialize(const rclcpp::Node::SharedPtr & node);

  void TickOnce() { tree_status_ = tree_.tickOnce(); }
  void TickExactlyOnce() { tree_status_ = tree_.tickExactlyOnce(); }
  void TickWhileRunning() { tree_status_ = tree_.tickWhileRunning(); }
  void HaltTree() { tree_.haltTree(); }

  BT::NodeStatus GetTreeStatus() { return tree_status_; }
  BT::Tree & GetTree() { return tree_; }
  BT::Blackboard::Ptr GetBlackboard() { return config_.blackboard; }

protected:
  /**
   * @brief Registers a BehaviorTree into the factory from a file, with custom BT nodes loaded
   * from plugins.
   */
  void RegisterTree();

  /**
   * @brief Registers a BehaviorTree into the factory from a file, with custom BT nodes and ROS
   * nodes loaded from plugins.
   *
   * @param node The ROS node used with ROS plugins.
   */
  void RegisterTree(const rclcpp::Node::SharedPtr & node);

  void CreateTree();

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
  BT::NodeConfig CreateBTConfig(const std::map<std::string, std::any> & bb_values);

private:
  const std::string project_path_;
  const std::vector<std::string> plugin_libs_;
  const std::vector<std::string> ros_plugin_libs_;
  const std::string tree_name_;
  const std::map<std::string, std::any> initial_blackboard_;
  const unsigned groot_port_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::NodeStatus tree_status_ = BT::NodeStatus::IDLE;
  BT::NodeConfig config_;
  std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_BEHAVIOR_TREE_MANAGER_HPP_
