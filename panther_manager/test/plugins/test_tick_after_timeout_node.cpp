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

#include <cstdint>
#include <map>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <panther_manager/plugins/action/call_trigger_service_node.hpp>
#include <panther_manager/plugins/decorator/tick_after_timeout_node.hpp>
#include <plugin_test_utils.hpp>

class TestTickAfterTimeout : public panther_manager::plugin_test_utils::PluginTestUtils
{
public:
  virtual std::string BuildBehaviorTree(
    const std::string & plugin_name, const std::map<std::string, std::string> & service) override;
};

std::string TestTickAfterTimeout::BuildBehaviorTree(
  const std::string & /* plugin_name */, const std::map<std::string, std::string> & /* service */
)
{
  std::stringstream bt;

  bt << tree_header_ << std::endl;
  bt << "\t\t\t<TickAfterTimeout timeout=\"0.1\" >" << std::endl;

  bt << "\t\t\t\t<AlwaysSuccess name=\"success_action\"/>" << std::endl;

  bt << "\t\t\t</TickAfterTimeout>" << std::endl;

  bt << tree_footer_;

  return bt.str();
}

TEST_F(TestTickAfterTimeout, GoodTickAfterTimeout)
{
  RegisterNodeWithoutParams<panther_manager::TickAfterTimeout>("TickAfterTimeout");

  CreateTree("", {});
  auto & tree = GetTree();

  for (std::size_t i = 0; i < 10; ++i) {
    auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
    if (status != BT::NodeStatus::SUCCESS && status != BT::NodeStatus::SKIPPED) {
      FAIL() << "Bad status of behavior tree.";
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
