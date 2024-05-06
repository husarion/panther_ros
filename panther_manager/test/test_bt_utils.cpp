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

#include <any>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/rclcpp.hpp"

#include "panther_manager/bt_utils.hpp"
#include "panther_utils/test/test_utils.hpp"

class TestRegisterBT : public testing::Test
{
public:
  TestRegisterBT();
  ~TestRegisterBT();

protected:
  void CreateBTProjectFile(const std::string & tree_xml);

  std::string bt_project_path_;

private:
  const std::string simple_tree_ = R"(
    <root BTCPP_format="4" project_name="Test">
      <BehaviorTree ID="TestTree">
        <Sequence>
          <AlwaysSuccess/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";
};

TestRegisterBT::TestRegisterBT()
{
  bt_project_path_ = testing::TempDir() + "test_bt.btproj";
  CreateBTProjectFile(simple_tree_);
}

TestRegisterBT::~TestRegisterBT() { std::filesystem::remove(bt_project_path_); }

void TestRegisterBT::CreateBTProjectFile(const std::string & tree_xml)
{
  std::ofstream out(bt_project_path_);
  if (out.is_open()) {
    out << tree_xml;
    out.close();
  }
}

TEST_F(TestRegisterBT, RegisterBehaviorTreeInvalidPlugin)
{
  BT::BehaviorTreeFactory factory;

  EXPECT_THROW(
    panther_manager::bt_utils::RegisterBehaviorTree(factory, bt_project_path_, {"invalid_bt_node"}),
    BT::RuntimeError);
}

TEST_F(TestRegisterBT, RegisterBehaviorTree)
{
  BT::BehaviorTreeFactory factory;

  EXPECT_NO_THROW(panther_manager::bt_utils::RegisterBehaviorTree(
    factory, bt_project_path_, {"tick_after_timeout_bt_node", "signal_shutdown_bt_node"}));

  // check if nodes were registered
  auto nodes = factory.manifests();
  EXPECT_TRUE(nodes.find("TickAfterTimeout") != nodes.end());
  EXPECT_TRUE(nodes.find("SignalShutdown") != nodes.end());

  // check if tree was registered
  auto trees = factory.registeredBehaviorTrees();
  EXPECT_TRUE(std::find(trees.begin(), trees.end(), "TestTree") != trees.end());
}

TEST_F(TestRegisterBT, RegisterBehaviorTreeROS)
{
  BT::BehaviorTreeFactory factory;

  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node");

  EXPECT_NO_THROW(panther_manager::bt_utils::RegisterBehaviorTree(
    factory, bt_project_path_, {}, node,
    {"call_trigger_service_bt_node", "call_set_bool_service_bt_node"}));

  // check if nodes were registered
  auto nodes = factory.manifests();
  EXPECT_TRUE(nodes.find("CallTriggerService") != nodes.end());
  EXPECT_TRUE(nodes.find("CallSetBoolService") != nodes.end());

  // check if tree was registered
  auto trees = factory.registeredBehaviorTrees();
  EXPECT_TRUE(std::find(trees.begin(), trees.end(), "TestTree") != trees.end());

  rclcpp::shutdown();
}

TEST(TestBTUtils, CreateBTConfigInvalidItem)
{
  const std::map<std::string, std::any> bb_values = {{"value", 1l}};

  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::invalid_argument>(
    [&bb_values]() { panther_manager::bt_utils::CreateBTConfig(bb_values); },
    "Invalid type for blackboard entry."));
}

TEST(TestBTUtils, CreateBTConfig)
{
  const std::map<std::string, std::any> bb_values = {
    {"bool_value", true},
    {"int_value", 1},
    {"unsigned_value", unsigned(1)},
    {"float_value", 1.0f},
    {"double_value", 1.0},
    {"char_value", "value"},
    {"string_value", std::string("value")},
  };

  BT::NodeConfig config;
  ASSERT_NO_THROW(config = panther_manager::bt_utils::CreateBTConfig(bb_values));

  EXPECT_TRUE(config.blackboard->get<bool>("bool_value"));
  EXPECT_EQ(1, config.blackboard->get<int>("int_value"));
  EXPECT_EQ(1, config.blackboard->get<unsigned>("unsigned_value"));
  EXPECT_FLOAT_EQ(1.0f, config.blackboard->get<float>("float_value"));
  EXPECT_EQ(1.0, config.blackboard->get<double>("double_value"));
  EXPECT_EQ("value", config.blackboard->get<std::string>("char_value"));
  EXPECT_EQ("value", config.blackboard->get<std::string>("string_value"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
