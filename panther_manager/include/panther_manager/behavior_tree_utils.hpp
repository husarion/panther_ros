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

#ifndef PANTHER_MANAGER_BEHAVIOR_TREE_UTILS_HPP_
#define PANTHER_MANAGER_BEHAVIOR_TREE_UTILS_HPP_

#include <any>
#include <map>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"

namespace panther_manager::behavior_tree_utils
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
inline void RegisterBehaviorTree(
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
inline void RegisterBehaviorTree(
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

}  // namespace panther_manager::behavior_tree_utils

namespace panther_manager
{
// TODO: @pawelirh move to a separate file with an appropriate abstraction layer
/**
 * @brief Get the Logger Prefix of BT node
 *
 * @param bt_node_name Behavior tree node name
 * @return std::string Logger prefix
 */
inline std::string GetLoggerPrefix(const std::string & bt_node_name)
{
  return std::string("[" + bt_node_name + "] ");
}

/**
 * @brief Get the Input Param object.
 * Wrapper for BT::getInput(), verifies if input parameter has value.
 *
 * @tparam ParamType
 * @param node BT node to use for param retrieve
 * @param param_name Param to retrieve from blackboard
 * @throws `BT::RuntimeError` if param has no value or couldn't convert to target type
 * @return ParamType
 */
template <class ParamType>
ParamType GetInputParam(BT::TreeNode * node, const std::string & param_name)
{
  auto input_param = node->getInput<ParamType>(param_name);

  if (!input_param.has_value()) {
    throw BT::RuntimeError(
      std::string(GetLoggerPrefix(node->name()) + "Error while reading port [") + param_name +
        std::string("]: "),
      input_param.error());
  }

  return input_param.value();
}

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_BEHAVIOR_TREE_UTILS_HPP_
