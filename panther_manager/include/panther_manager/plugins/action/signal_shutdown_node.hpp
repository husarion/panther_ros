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

#ifndef PANTHER_MANAGER_SIGNAL_SHUTDOWN_NODE_HPP_
#define PANTHER_MANAGER_SIGNAL_SHUTDOWN_NODE_HPP_

#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

namespace panther_manager
{

class SignalShutdown : public BT::SyncActionNode
{
public:
  explicit SignalShutdown(const std::string & name, const BT::NodeConfig & conf)
  : BT::SyncActionNode(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("reason", "", "reason to shutdown robot"),
    };
  }

  virtual BT::NodeStatus tick() override;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SIGNAL_SHUTDOWN_NODE_HPP_
