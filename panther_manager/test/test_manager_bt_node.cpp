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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "panther_msgs/msg/io_state.hpp"

#include "panther_manager/manager_bt_node.hpp"
#include "panther_utils/test/test_utils.hpp"

class ManagerBTNodeWrapper : public panther_manager::ManagerBTNode
{
public:
  ManagerBTNodeWrapper(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ManagerBTNode(node_name, options)
  {
  }

  void DeclareParameters() { return ManagerBTNode::DeclareParameters(); }
  void RegisterBehaviorTree() { return ManagerBTNode::RegisterBehaviorTree(); }
  void CreateLightsTree() { return ManagerBTNode::CreateLightsTree(); }
  void CreateSafetyTree() { return ManagerBTNode::CreateSafetyTree(); }
  void CreateShutdownTree() { return ManagerBTNode::CreateShutdownTree(); }
  bool SystemReady() { return ManagerBTNode::SystemReady(); }
};

class TestManagerBTNode : public testing::Test
{
public:
  TestManagerBTNode();
  ~TestManagerBTNode();

protected:
  void CreateBTProjectFile(const std::string & tree_xml);

  std::shared_ptr<ManagerBTNodeWrapper> manager_bt_node_;

private:
  std::string bt_project_path_;

  const std::string simple_tree_ = R"(
    <root BTCPP_format="4" project_name="Test">
      <BehaviorTree>
        <Sequence>
          <ShutdownSingleHost/>
          <ShutdownHostsFromFile/>
          <CallTriggerService/>
          <CallSetBoolService/>
          <CallSetLedAnimationService/>
          <SignalShutdown/>
          <TickAfterTimeout>
            <AlwaysSuccess/>
          </TickAfterTimeout>
        </Sequence>
      </BehaviorTree>
    </root>
  )";
};

TestManagerBTNode::TestManagerBTNode()
{
  bt_project_path_ = testing::TempDir() + "test_bt.btproj";

  CreateBTProjectFile(simple_tree_);

  std::vector<std::string> plugin_libs;
  plugin_libs.push_back("tick_after_timeout_bt_node");
  plugin_libs.push_back("shutdown_single_host_bt_node");
  plugin_libs.push_back("shutdown_hosts_from_file_bt_node");
  plugin_libs.push_back("signal_shutdown_bt_node");

  std::vector<std::string> ros_plugin_libs;
  ros_plugin_libs.push_back("call_set_bool_service_bt_node");
  ros_plugin_libs.push_back("call_trigger_service_bt_node");
  ros_plugin_libs.push_back("call_set_led_animation_service_bt_node");

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("bt_project_path", bt_project_path_));
  params.push_back(rclcpp::Parameter("plugin_libs", plugin_libs));
  params.push_back(rclcpp::Parameter("ros_plugin_libs", ros_plugin_libs));
  params.push_back(rclcpp::Parameter("launch_lights_tree", false));
  params.push_back(rclcpp::Parameter("launch_safety_tree", false));
  params.push_back(rclcpp::Parameter("launch_shutdown_tree", false));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  manager_bt_node_ = std::make_shared<ManagerBTNodeWrapper>("test_manager_bt_node", options);
}

TestManagerBTNode::~TestManagerBTNode() { std::filesystem::remove(bt_project_path_); }

void TestManagerBTNode::CreateBTProjectFile(const std::string & tree_xml)
{
  std::ofstream out(bt_project_path_);
  if (out.is_open()) {
    out << tree_xml;
    out.close();
  }
}

TEST_F(TestManagerBTNode, RegisterBehaviorTree)
{
  EXPECT_NO_THROW(manager_bt_node_->RegisterBehaviorTree());
}

TEST_F(TestManagerBTNode, CreateLightsTreeMissingTree)
{
  ASSERT_NO_THROW(manager_bt_node_->RegisterBehaviorTree());
  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { manager_bt_node_->CreateLightsTree(); }, "Can't find a tree with name: Lights"));
}

TEST_F(TestManagerBTNode, CreateLightsTree)
{
  const auto tree_xml = R"(
    <root BTCPP_format="4" project_name="Test">
      <BehaviorTree ID="Lights">
        <AlwaysSuccess/>
      </BehaviorTree>
    </root>
  )";

  CreateBTProjectFile(tree_xml);
  ASSERT_NO_THROW(manager_bt_node_->RegisterBehaviorTree());
  EXPECT_NO_THROW(manager_bt_node_->CreateLightsTree());
}

TEST_F(TestManagerBTNode, CreateSafetyTreeMissingTree)
{
  ASSERT_NO_THROW(manager_bt_node_->RegisterBehaviorTree());
  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { manager_bt_node_->CreateSafetyTree(); }, "Can't find a tree with name: Safety"));
}

TEST_F(TestManagerBTNode, CreateSafetyTree)
{
  const auto tree_xml = R"(
    <root BTCPP_format="4" project_name="Test">
      <BehaviorTree ID="Safety">
        <AlwaysSuccess/>
      </BehaviorTree>
    </root>
  )";

  CreateBTProjectFile(tree_xml);
  ASSERT_NO_THROW(manager_bt_node_->RegisterBehaviorTree());
  EXPECT_NO_THROW(manager_bt_node_->CreateSafetyTree());
}

TEST_F(TestManagerBTNode, CreateShutdownTreeMissingTree)
{
  ASSERT_NO_THROW(manager_bt_node_->RegisterBehaviorTree());
  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { manager_bt_node_->CreateShutdownTree(); }, "Can't find a tree with name: Shutdown"));
}

TEST_F(TestManagerBTNode, CreateShutdownTree)
{
  const auto tree_xml = R"(
    <root BTCPP_format="4" project_name="Test">
      <BehaviorTree ID="Shutdown">
        <AlwaysSuccess/>
      </BehaviorTree>
    </root>
  )";

  CreateBTProjectFile(tree_xml);
  ASSERT_NO_THROW(manager_bt_node_->RegisterBehaviorTree());
  EXPECT_NO_THROW(manager_bt_node_->CreateShutdownTree());
}

TEST_F(TestManagerBTNode, SystemReady)
{
  ASSERT_NO_THROW(manager_bt_node_->Initialize());
  EXPECT_FALSE(manager_bt_node_->SystemReady());

  auto e_stop_pub = manager_bt_node_->create_publisher<std_msgs::msg::Bool>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  auto io_state_pub = manager_bt_node_->create_publisher<panther_msgs::msg::IOState>(
    "hardware/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  auto battery_pub = manager_bt_node_->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery", 1);

  e_stop_pub->publish(std_msgs::msg::Bool());
  io_state_pub->publish(panther_msgs::msg::IOState());
  battery_pub->publish(sensor_msgs::msg::BatteryState());

  rclcpp::spin_some(manager_bt_node_->get_node_base_interface());
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  EXPECT_TRUE(manager_bt_node_->SystemReady());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
