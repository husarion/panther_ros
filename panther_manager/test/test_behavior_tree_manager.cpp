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

#include "panther_manager/behavior_tree_manager.hpp"
#include "panther_utils/test/test_utils.hpp"

class BehaviorTreeManagerWrapper : public panther_manager::BehaviorTreeManager
{
public:
  BehaviorTreeManagerWrapper(const panther_manager::BehaviorTreeParams & params)
  : BehaviorTreeManager(params)
  {
  }
  ~BehaviorTreeManagerWrapper() {}

  BT::NodeConfig CreateBTConfig(const std::map<std::string, std::any> & bb_values)
  {
    return BehaviorTreeManager::CreateBTConfig(bb_values);
  }
};

class TestBehaviorTreeManager : public testing::Test
{
public:
  TestBehaviorTreeManager();
  ~TestBehaviorTreeManager() {}

protected:
  static constexpr char kTreeName[] = "TestTree";

  std::unique_ptr<BehaviorTreeManagerWrapper> behavior_tree_manager_;
  BT::BehaviorTreeFactory factory_;
};

TestBehaviorTreeManager::TestBehaviorTreeManager()
{
  panther_manager::BehaviorTreeParams params;
  params.tree_name = kTreeName;
  params.initial_blackboard = {};
  behavior_tree_manager_ = std::make_unique<BehaviorTreeManagerWrapper>(params);
}

TEST_F(TestBehaviorTreeManager, CreateBTConfigInvalidItem)
{
  const std::map<std::string, std::any> bb_values = {{"value", 1l}};

  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::invalid_argument>(
    [&]() { behavior_tree_manager_->CreateBTConfig(bb_values); },
    "Invalid type for blackboard entry."));
}

TEST_F(TestBehaviorTreeManager, CreateBTConfig)
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
  ASSERT_NO_THROW(config = behavior_tree_manager_->CreateBTConfig(bb_values));

  EXPECT_TRUE(config.blackboard->get<bool>("bool_value"));
  EXPECT_EQ(1, config.blackboard->get<int>("int_value"));
  EXPECT_EQ(1, config.blackboard->get<unsigned>("unsigned_value"));
  EXPECT_FLOAT_EQ(1.0f, config.blackboard->get<float>("float_value"));
  EXPECT_EQ(1.0, config.blackboard->get<double>("double_value"));
  EXPECT_EQ("value", config.blackboard->get<std::string>("char_value"));
  EXPECT_EQ("value", config.blackboard->get<std::string>("string_value"));
}

TEST_F(TestBehaviorTreeManager, InitializeInvalidTreeName)
{
  const auto tree_xml = R"(
    <root BTCPP_format="4" project_name="Test">
      <BehaviorTree ID="InvalidName">
        <AlwaysSuccess/>
      </BehaviorTree>
    </root>
  )";

  ASSERT_NO_THROW(factory_.registerBehaviorTreeFromText(tree_xml));
  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { behavior_tree_manager_->Initialize(factory_); },
    "Can't find a tree with name: " + std::string(kTreeName)));
}

TEST_F(TestBehaviorTreeManager, Initialize)
{
  const std::string tree_xml = R"(
    <root BTCPP_format="4" project_name="Test">
      <BehaviorTree ID=")" + std::string(kTreeName) +
                               R"(">
        <Sequence>
          <AlwaysSuccess/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  ASSERT_NO_THROW(factory_.registerBehaviorTreeFromText(tree_xml));
  EXPECT_NO_THROW(behavior_tree_manager_->Initialize(factory_));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
