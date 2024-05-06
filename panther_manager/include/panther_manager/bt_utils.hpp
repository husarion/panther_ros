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

#ifndef PANTHER_MANAGER_BT_UTILS_HPP_
#define PANTHER_MANAGER_BT_UTILS_HPP_

#include <any>
#include <map>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"

namespace panther_manager::bt_utils
{

/**
 * @brief Registers a BehaviorTree into the factory from a file, with custom BT nodes loaded
 * from plugins.
 *
 * @param factory The BT factory used to register nodes and the BehaviorTree.
 * @param bt_project_path The path to the BehaviorTree project file.
 * @param plugin_libs A vector containing the names of the nodes that will be registered from
 * plugins.
 */
void RegisterBehaviorTree(
  BT::BehaviorTreeFactory & factory, const std::string & bt_project_path,
  const std::vector<std::string> plugin_libs)
{
  for (const auto & plugin : plugin_libs) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
  }

  factory.registerBehaviorTreeFromFile(bt_project_path);
}

/**
 * @brief Registers a BehaviorTree into the factory from a file, with custom BT nodes and ROS nodes
 * loaded from plugins.
 *
 * @param factory The BT factory used to register nodes and the BehaviorTree.
 * @param bt_project_path The path to the BehaviorTree project file.
 * @param plugin_libs A vector containing the names of the nodes that will be registered from
 * plugins.
 * @param node The ROS node used with ROS plugins.
 * @param ros_plugin_libs A vector containing the names of the ROS nodes that will be registered
 * from plugins.
 */
void RegisterBehaviorTree(
  BT::BehaviorTreeFactory & factory, const std::string & bt_project_path,
  const std::vector<std::string> plugin_libs, const rclcpp::Node::SharedPtr & node,
  const std::vector<std::string> ros_plugin_libs)
{
  for (const auto & plugin : ros_plugin_libs) {
    BT::RosNodeParams params;
    params.nh = node;
    RegisterRosNode(factory, BT::SharedLibrary::getOSName(plugin), params);
  }

  RegisterBehaviorTree(factory, bt_project_path, plugin_libs);
}

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
BT::NodeConfig CreateBTConfig(const std::map<std::string, std::any> & bb_values)
{
  BT::NodeConfig config;
  config.blackboard = BT::Blackboard::create();

  for (auto & [name, value] : bb_values) {
    const std::type_info & type = value.type();
    if (type == typeid(bool)) {
      config.blackboard->set<bool>(name, std::any_cast<bool>(value));
    } else if (type == typeid(int)) {
      config.blackboard->set<int>(name, std::any_cast<int>(value));
    } else if (type == typeid(unsigned)) {
      config.blackboard->set<unsigned>(name, std::any_cast<unsigned>(value));
    } else if (type == typeid(float)) {
      config.blackboard->set<float>(name, std::any_cast<float>(value));
    } else if (type == typeid(double)) {
      config.blackboard->set<double>(name, std::any_cast<double>(value));
    } else if (type == typeid(const char *)) {
      config.blackboard->set<std::string>(name, std::any_cast<const char *>(value));
    } else if (type == typeid(std::string)) {
      config.blackboard->set<std::string>(name, std::any_cast<std::string>(value));
    } else {
      throw std::invalid_argument(
        "Invalid type for blackboard entry. Valid types are: bool, int, unsigned, float, double, "
        "const char*, std::string");
    }
  }

  return config;
}

}  // namespace panther_manager::bt_utils

#endif  // PANTHER_MANAGER_BT_UTILS_HPP_
