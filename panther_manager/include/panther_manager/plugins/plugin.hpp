// Copyright (c) 2023 Davide Faconti
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

#pragma once

#include <filesystem>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "ros/ros.h"

// Use this macro to generate a plugin for any BT node requiring NodeHandle
//
// - BT::RosActionNode
// - BT::RosServiceNode
// - BT::RosServiceNode
// - BT::RosTopicPubNode
// - BT::RosTopicSubNode
//
// - First argument: type to register (name of the class)
// - Second argument: string with the registration name
//
// Usage example:
//   CreateRosNodePlugin(MyClassName, "MyClassName");

#define CreateRosNodePlugin(TYPE, REGISTRATION_NAME)                                \
  BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(                                   \
    BT::BehaviorTreeFactory & factory, const std::shared_ptr<ros::NodeHandle> & nh) \
  {                                                                                 \
    factory.registerNodeType<TYPE>(REGISTRATION_NAME, nh);                          \
  }

/**
 * @brief RegisterRosNode function used to load a plugin and register
 * the containing Node definition.
 *
 * @param factory   the factory where the node should be registered.
 * @param filepath  path to the plugin.
 * @param nh    parameters to pass to the instances of the Node.
 */
inline void RegisterRosNode(
  BT::BehaviorTreeFactory & factory, const std::filesystem::path & filepath,
  const std::shared_ptr<ros::NodeHandle> & nh)
{
  BT::SharedLibrary loader(filepath);
  typedef void (*Func)(BT::BehaviorTreeFactory &, const std::shared_ptr<ros::NodeHandle> &);
  auto func = (Func)loader.getSymbol("BT_RegisterRosNodeFromPlugin");
  func(factory, nh);
}
