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

typedef panther_manager::plugin_test_utils::PluginTestUtils ShutdownHostsNodeBehaviorTreeTest;

class ShutdownHostsNodeWrapper : public panther_manager::ShutdownHosts
{
public:
  ShutdownHostsNodeWrapper(const std::string & name, const BT::NodeConfig & conf)
  : panther_manager::ShutdownHosts(name, conf)
  {
  }
  void RemoveDuplicatedHosts(std::vector<std::shared_ptr<panther_manager::ShutdownHost>> & hosts);
  std::vector<std::shared_ptr<panther_manager::ShutdownHost>> & GetHosts();
};

void ShutdownHostsNodeWrapper::RemoveDuplicatedHosts(
  std::vector<std::shared_ptr<panther_manager::ShutdownHost>> & hosts)
{
  panther_manager::ShutdownHosts::RemoveDuplicatedHosts(hosts);
}

std::vector<std::shared_ptr<panther_manager::ShutdownHost>> & ShutdownHostsNodeWrapper::GetHosts()
{
  return hosts_;
}

class DuplicatedHostsShutdownHostsNodeWrapper : public ShutdownHostsNodeWrapper
{
public:
  DuplicatedHostsShutdownHostsNodeWrapper(const std::string & name, const BT::NodeConfig & conf)
  : ShutdownHostsNodeWrapper(name, conf)
  {
  }
  bool UpdateHosts(std::vector<std::shared_ptr<panther_manager::ShutdownHost>> & hosts);
  static BT::PortsList providedPorts() { return {}; }
};

bool DuplicatedHostsShutdownHostsNodeWrapper::UpdateHosts(
  std::vector<std::shared_ptr<panther_manager::ShutdownHost>> & hosts)
{
  hosts.emplace_back(std::make_shared<panther_manager::ShutdownHost>("127.0.0.1", "husarion"));
  hosts.emplace_back(std::make_shared<panther_manager::ShutdownHost>("localhost", "husarion"));
  hosts.emplace_back(std::make_shared<panther_manager::ShutdownHost>("127.0.0.1", "husarion"));
  return true;
}

class FailedUpdateHostsShutdownHostsNodeWrapper : public ShutdownHostsNodeWrapper
{
public:
  FailedUpdateHostsShutdownHostsNodeWrapper(const std::string & name, const BT::NodeConfig & conf)
  : ShutdownHostsNodeWrapper(name, conf)
  {
  }
  bool UpdateHosts(std::vector<std::shared_ptr<panther_manager::ShutdownHost>> & hosts);
  static BT::PortsList providedPorts() { return {}; }
};

bool FailedUpdateHostsShutdownHostsNodeWrapper::UpdateHosts(
  std::vector<std::shared_ptr<panther_manager::ShutdownHost>> & hosts)
{
  hosts.emplace_back(std::make_shared<panther_manager::ShutdownHost>("127.0.0.1", "husarion"));
  hosts.emplace_back(std::make_shared<panther_manager::ShutdownHost>("localhost", "husarion"));
  return false;
}

class ShutdownHostsNodeTest : public testing::Test
{
public:
  void CreateDuplicationWrapper();
  void CreateFailedUpdatedHostWrapper();

protected:
  std::unique_ptr<ShutdownHostsNodeWrapper> wrapper;
};

void ShutdownHostsNodeTest::CreateDuplicationWrapper()
{
  BT::NodeConfig conf;
  wrapper = std::make_unique<DuplicatedHostsShutdownHostsNodeWrapper>("Duplicated hosts", conf);
}

void ShutdownHostsNodeTest::CreateFailedUpdatedHostWrapper()
{
  BT::NodeConfig conf;
  wrapper = std::make_unique<FailedUpdateHostsShutdownHostsNodeWrapper>("Failed hosts", conf);
}

TEST_F(ShutdownHostsNodeTest, GoodRemovingDuplicatedHosts)
{
  CreateDuplicationWrapper();
  std::vector<std::shared_ptr<panther_manager::ShutdownHost>> hosts;
  ASSERT_TRUE(wrapper->UpdateHosts(hosts));
  ASSERT_EQ(hosts.size(), 3);
  wrapper->RemoveDuplicatedHosts(hosts);
  ASSERT_EQ(hosts.size(), 2);
}

TEST_F(ShutdownHostsNodeBehaviorTreeTest, FailedBehaviorTreeWhenUpdateHostReturnsFalse)
{
  RegisterNodeWithoutParams<FailedUpdateHostsShutdownHostsNodeWrapper>("ShutdownHosts");
  CreateTree("ShutdownHosts", {});
  auto & tree = GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
