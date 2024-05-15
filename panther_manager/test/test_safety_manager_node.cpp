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

#include <panther_manager/safety_manager_node.hpp>
#include "panther_utils/test/test_utils.hpp"

using BoolMsg = std_msgs::msg::Bool;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class SafetyManagerNodeWrapper : public panther_manager::SafetyManagerNode
{
public:
  SafetyManagerNodeWrapper(
    const std::string & node_name, const rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : SafetyManagerNode(node_name, options)
  {
  }

  ~SafetyManagerNodeWrapper() {}

  bool SystemReady() { return SafetyManagerNode::SystemReady(); }

  BT::Blackboard::Ptr GetSafetyTreeBlackboard()
  {
    return this->safety_tree_manager_->GetBlackboard();
  }
};

class TestSafetyManagerNode : public testing::Test
{
public:
  TestSafetyManagerNode();
  ~TestSafetyManagerNode();

protected:
  void CreateBTProjectFile(const std::string & tree_xml);

  template <typename MsgT>
  void PublishAndSpin(
    const std::string & topic_name, MsgT msg,
    const rclcpp::QoS qos_profile = rclcpp::SystemDefaultsQoS());

  std::shared_ptr<SafetyManagerNodeWrapper> safety_manager_node_;

private:
  std::string bt_project_path_;

  const std::string simple_tree_ = R"(
    <root BTCPP_format="4" project_name="Test">
      <BehaviorTree ID="Safety">
        <Sequence>
          <CallTriggerService service_name="dummy_service"/>
          <CallSetBoolService service_name="dummy_service"/>
          <SignalShutdown/>
          <TickAfterTimeout>
            <AlwaysSuccess/>
          </TickAfterTimeout>
        </Sequence>
      </BehaviorTree>

      <BehaviorTree ID="Shutdown">
        <AlwaysSuccess/>
      </BehaviorTree>
    </root>
  )";
};

TestSafetyManagerNode::TestSafetyManagerNode()
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

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("bt_project_path", bt_project_path_));
  params.push_back(rclcpp::Parameter("plugin_libs", plugin_libs));
  params.push_back(rclcpp::Parameter("ros_plugin_libs", ros_plugin_libs));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  safety_manager_node_ = std::make_shared<SafetyManagerNodeWrapper>(
    "test_safety_manager_node", options);
}

TestSafetyManagerNode::~TestSafetyManagerNode() { std::filesystem::remove(bt_project_path_); }

void TestSafetyManagerNode::CreateBTProjectFile(const std::string & tree_xml)
{
  std::ofstream out(bt_project_path_);
  if (out.is_open()) {
    out << tree_xml;
    out.close();
  }
}

template <typename MsgT>
void TestSafetyManagerNode::PublishAndSpin(
  const std::string & topic_name, MsgT msg, const rclcpp::QoS qos_profile)
{
  ASSERT_NO_THROW(safety_manager_node_->Initialize());
  auto publisher = safety_manager_node_->create_publisher<MsgT>(topic_name, qos_profile);

  publisher->publish(msg);

  rclcpp::spin_some(safety_manager_node_->get_node_base_interface());
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// TEST_F(TestSafetyManagerNode, RegisterBehaviorTree)
// {
//   EXPECT_NO_THROW(safety_manager_node_->RegisterBehaviorTree());
// }

// TEST_F(TestSafetyManagerNode, CreateSafetyTreeInvalidTreeName)
// {
//   const auto tree_xml = R"(
//     <root BTCPP_format="4" project_name="Test">
//       <BehaviorTree ID="InvalidName">
//         <AlwaysSuccess/>
//       </BehaviorTree>
//     </root>
//   )";

//   // overwrite default tree
//   CreateBTProjectFile(tree_xml);
//   ASSERT_NO_THROW(safety_manager_node_->RegisterBehaviorTree());
//   EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
//     [&]() { safety_manager_node_->CreateSafetyTree(); }, "Can't find a tree with name: Safety"));
// }

// TEST_F(TestSafetyManagerNode, CreateSafetyTree)
// {
//   ASSERT_NO_THROW(safety_manager_node_->RegisterBehaviorTree());
//   EXPECT_NO_THROW(safety_manager_node_->CreateSafetyTree());
// }

// TEST_F(TestSafetyManagerNode, CreateShutdownTreeInvalidTreeName)
// {
//   const auto tree_xml = R"(
//     <root BTCPP_format="4" project_name="Test">
//       <BehaviorTree ID="InvalidName">
//         <AlwaysSuccess/>
//       </BehaviorTree>
//     </root>
//   )";

//   // overwrite default tree
//   CreateBTProjectFile(tree_xml);
//   ASSERT_NO_THROW(safety_manager_node_->RegisterBehaviorTree());
//   EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
//     [&]() { safety_manager_node_->CreateShutdownTree(); },
//     "Can't find a tree with name: Shutdown"));
// }

// TEST_F(TestSafetyManagerNode, CreateShutdownTree)
// {
//   ASSERT_NO_THROW(safety_manager_node_->RegisterBehaviorTree());
//   EXPECT_NO_THROW(safety_manager_node_->CreateShutdownTree());
// }

TEST_F(TestSafetyManagerNode, SystemReady)
{
  ASSERT_NO_THROW(safety_manager_node_->Initialize());
  EXPECT_FALSE(safety_manager_node_->SystemReady());

  safety_manager_node_->GetSafetyTreeBlackboard()->set<bool>("e_stop_state", true);
  EXPECT_FALSE(safety_manager_node_->SystemReady());

  safety_manager_node_->GetSafetyTreeBlackboard()->set<unsigned>("battery_status", 0);
  EXPECT_FALSE(safety_manager_node_->SystemReady());

  safety_manager_node_->GetSafetyTreeBlackboard()->set<bool>("aux_state", false);
  EXPECT_FALSE(safety_manager_node_->SystemReady());

  safety_manager_node_->GetSafetyTreeBlackboard()->set<float>("cpu_temp", 20.0);
  EXPECT_FALSE(safety_manager_node_->SystemReady());

  safety_manager_node_->GetSafetyTreeBlackboard()->set<float>("driver_temp", 20.0);
  EXPECT_TRUE(safety_manager_node_->SystemReady());
}

TEST_F(TestSafetyManagerNode, BatteryCBBlackboardUpdate)
{
  const auto expected_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  const auto expected_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  const float expected_temp = 27.0;

  auto battery_state = sensor_msgs::msg::BatteryState();
  battery_state.power_supply_status = expected_status;
  battery_state.power_supply_health = expected_health;
  battery_state.temperature = expected_temp;

  PublishAndSpin("battery", battery_state);

  auto blackboard = safety_manager_node_->GetSafetyTreeBlackboard();
  EXPECT_EQ(blackboard->get<unsigned>("battery_status"), expected_status);
  EXPECT_EQ(blackboard->get<unsigned>("battery_health"), expected_health);
  EXPECT_FLOAT_EQ(blackboard->get<float>("bat_temp"), expected_temp);
}

TEST_F(TestSafetyManagerNode, EStopCBBlackboardUpdate)
{
  const bool expected_state = true;

  auto bool_msg = std_msgs::msg::Bool();
  bool_msg.data = expected_state;

  PublishAndSpin(
    "hardware/e_stop", bool_msg, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  auto blackboard = safety_manager_node_->GetSafetyTreeBlackboard();
  EXPECT_EQ(blackboard->get<bool>("e_stop_state"), expected_state);
}

TEST_F(TestSafetyManagerNode, DriverStateCBBlackboardUpdate)
{
  const float expected_temp = 21.0;

  auto driver_state_msg = panther_msgs::msg::DriverState();
  driver_state_msg.front.temperature = expected_temp;
  driver_state_msg.rear.temperature = expected_temp;

  PublishAndSpin("driver/motor_controllers_state", driver_state_msg);

  auto blackboard = safety_manager_node_->GetSafetyTreeBlackboard();
  EXPECT_FLOAT_EQ(blackboard->get<float>("driver_temp"), expected_temp);
}

TEST_F(TestSafetyManagerNode, IOStateCBBlackboardUpdate)
{
  const bool expected_aux_state = true;
  const bool expected_fan_state = true;

  auto io_state_msg = panther_msgs::msg::IOState();
  io_state_msg.aux_power = expected_aux_state;
  io_state_msg.fan = expected_fan_state;

  PublishAndSpin(
    "hardware/io_state", io_state_msg,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  auto blackboard = safety_manager_node_->GetSafetyTreeBlackboard();
  EXPECT_EQ(blackboard->get<bool>("aux_state"), expected_aux_state);
  EXPECT_EQ(blackboard->get<bool>("fan_state"), expected_fan_state);
}

TEST_F(TestSafetyManagerNode, SystemStatusCBBlackboardUpdate)
{
  const float expected_temp = 21.0;

  auto system_status_msg = panther_msgs::msg::SystemStatus();
  system_status_msg.cpu_temp = expected_temp;

  PublishAndSpin("system_status", system_status_msg);

  auto blackboard = safety_manager_node_->GetSafetyTreeBlackboard();
  EXPECT_FLOAT_EQ(blackboard->get<float>("cpu_temp"), expected_temp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
