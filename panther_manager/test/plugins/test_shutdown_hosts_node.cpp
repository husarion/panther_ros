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

#include <string>

#include <gtest/gtest.h>

#include <panther_manager/plugins/shutdown_hosts_node.hpp>
#include <plugin_test_utils.hpp>

class ShutdownHostsNodeDuplicated : public panther_manager::ShutdownHosts
{
public:
  ShutdownHostsNodeDuplicated(const std::string & name, const BT::NodeConfig & conf)
  : panther_manager::ShutdownHosts(name, conf)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

private:
  virtual void update_hosts(
    std::vector<std::shared_ptr<panther_manager::ShutdownHost>> & hosts) override final;
};

void ShutdownHostsNodeDuplicated::update_hosts(
  std::vector<std::shared_ptr<panther_manager::ShutdownHost>> & hosts)
{
  hosts.emplace_back(std::make_shared<panther_manager::ShutdownHost>("127.0.0.1", "husarion"));
  hosts.emplace_back(std::make_shared<panther_manager::ShutdownHost>("localhost", "husarion"));
  hosts.emplace_back(std::make_shared<panther_manager::ShutdownHost>("127.0.0.1", "husarion"));
}

//  TODO: Create TESTs completely

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
