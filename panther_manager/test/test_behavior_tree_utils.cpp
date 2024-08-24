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
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

#include "panther_manager/behavior_tree_utils.hpp"
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
    panther_manager::behavior_tree_utils::RegisterBehaviorTree(
      factory, bt_project_path_, {"invalid_bt_node"}),
    BT::RuntimeError);
}

TEST_F(TestRegisterBT, RegisterBehaviorTree)
{
  BT::BehaviorTreeFactory factory;

  EXPECT_NO_THROW(panther_manager::behavior_tree_utils::RegisterBehaviorTree(
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

  BT::RosNodeParams params;
  params.nh = std::make_shared<rclcpp::Node>("test_node");

  EXPECT_NO_THROW(panther_manager::behavior_tree_utils::RegisterBehaviorTree(
    factory, bt_project_path_, {}, params,
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

TEST(TestConvertFromString, GoodVectorInt)
{
  std::string str = "1;2;3";
  auto result = BT::convertFromString<std::vector<int>>(str);

  EXPECT_EQ(result.size(), 3);
  EXPECT_EQ(result[0], 1);
  EXPECT_EQ(result[1], 2);
  EXPECT_EQ(result[2], 3);
}

TEST(TestConvertFromString, EmptyVectorInt)
{
  std::string str = "";
  EXPECT_THROW(BT::convertFromString<std::vector<int>>(str), BT::RuntimeError);
}

TEST(TestConvertFromString, InvalidVectorIntWithFloat)
{
  std::string str = "1.0;2;3;4";
  EXPECT_THROW(BT::convertFromString<std::vector<int>>(str), BT::RuntimeError);
}

TEST(TestConvertFromString, InvalidVectorIntWithLetter)
{
  std::string str = "a;2;3;4";
  EXPECT_THROW(BT::convertFromString<std::vector<int>>(str), BT::RuntimeError);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
