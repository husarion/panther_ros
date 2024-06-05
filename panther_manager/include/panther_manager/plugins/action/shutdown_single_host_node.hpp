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

#ifndef PANTHER_MANAGER_SHUTDOWN_SINGLE_HOST_NODE_HPP_
#define PANTHER_MANAGER_SHUTDOWN_SINGLE_HOST_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/basic_types.h"

#include "panther_manager/plugins/shutdown_host.hpp"
#include "panther_manager/plugins/shutdown_hosts_node.hpp"

namespace panther_manager
{

class ShutdownSingleHost : public ShutdownHosts
{
public:
  ShutdownSingleHost(const std::string & name, const BT::NodeConfig & conf)
  : ShutdownHosts(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("ip", "ip of the host to shutdown"),
      BT::InputPort<std::string>("username", "user to log into while executing shutdown command"),
      BT::InputPort<unsigned>("port", "SSH communication port"),
      BT::InputPort<std::string>("command", "command to execute on shutdown"),
      BT::InputPort<float>("timeout", "time in seconds to wait for host to shutdown"),
      BT::InputPort<bool>(
        "ping_for_success", "ping host until it is not available or timeout is reached"),
    };
  }

private:
  bool UpdateHosts(std::vector<std::shared_ptr<ShutdownHost>> & hosts) override;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SHUTDOWN_SINGLE_HOST_NODE_HPP_
