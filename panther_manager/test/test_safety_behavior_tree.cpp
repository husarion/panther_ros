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
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "panther_msgs/msg/driver_state.hpp"
#include "panther_msgs/msg/io_state.hpp"
#include "panther_msgs/msg/system_status.hpp"

#include <include/test_behavior_tree_utils.hpp>
#include <panther_manager/safety_manager_node.hpp>

using BoolMsg = std_msgs::msg::Bool;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using IOStateMsg = panther_msgs::msg::IOState;
using DriverStateMsg = panther_msgs::msg::DriverState;
using SystemStatusMsg = panther_msgs::msg::SystemStatus;
using SetBoolSrv = std_srvs::srv::SetBool;
using TriggerSrv = std_srvs::srv::Trigger;

class SafetyManagerNodeWrapper : public panther_manager::SafetyManagerNode
{
public:
  SafetyManagerNodeWrapper(
    const std::string & node_name, const rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : SafetyManagerNode(node_name, options)
  {
  }

  ~SafetyManagerNodeWrapper() {}

  BT::NodeStatus GetSafetyTreeStatus() { return this->safety_tree_manager_->GetTreeStatus(); }
  BT::NodeStatus GetShutdownTreeStatus() { return this->shutdown_tree_manager_->GetTreeStatus(); }
};

class TestSafetyBehaviorTree : public testing::Test
{
public:
  TestSafetyBehaviorTree();
  ~TestSafetyBehaviorTree();

protected:
  bool SpinWhileRunning();
  void PublishDefaultStates();
  void PublishEStop(const bool data);
  void PublishBatteryState(
    const std::uint8_t status, const std::uint8_t health, const float percentage);
  void PublishIOState(const bool fan, const bool aux_power);
  void PublishDriverState(const float temperature);
  void PublishSystemStatus(const float cpu_temp);

  static constexpr float kFanTurnOffTimeout = 1.0;

  bool fan_state_ = false;
  bool aux_power_state_ = true;
  bool e_stop_triggered_ = false;
  std::shared_ptr<SafetyManagerNodeWrapper> safety_manager_node_;

private:
  void FanServerCB(
    const SetBoolSrv::Request::SharedPtr & request, SetBoolSrv::Response::SharedPtr response);
  void AUXPowerServerCB(
    const SetBoolSrv::Request::SharedPtr & request, SetBoolSrv::Response::SharedPtr response);
  void EStopTriggerSrvCB(
    const TriggerSrv::Request::SharedPtr & /*request*/, TriggerSrv::Response::SharedPtr response);

  rclcpp::Publisher<BoolMsg>::SharedPtr e_stop_pub_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_state_pub_;
  rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_pub_;
  rclcpp::Publisher<DriverStateMsg>::SharedPtr driver_state_pub_;
  rclcpp::Publisher<SystemStatusMsg>::SharedPtr system_status_pub_;

  rclcpp::Service<SetBoolSrv>::SharedPtr fan_server_;
  rclcpp::Service<SetBoolSrv>::SharedPtr aux_power_server_;
  rclcpp::Service<TriggerSrv>::SharedPtr e_stop_trigger_server_;
};

TestSafetyBehaviorTree::TestSafetyBehaviorTree()
{
  using namespace std::placeholders;

  rclcpp::init(0, nullptr);

  std::vector<std::string> plugin_libs;
  plugin_libs.push_back("tick_after_timeout_bt_node");
  plugin_libs.push_back("shutdown_single_host_bt_node");
  plugin_libs.push_back("shutdown_hosts_from_file_bt_node");
  plugin_libs.push_back("signal_shutdown_bt_node");

  std::vector<std::string> ros_plugin_libs;
  ros_plugin_libs.push_back("call_set_bool_service_bt_node");
  ros_plugin_libs.push_back("call_trigger_service_bt_node");

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("plugin_libs", plugin_libs));
  params.push_back(rclcpp::Parameter("ros_plugin_libs", ros_plugin_libs));
  params.push_back(rclcpp::Parameter("fan_turn_off_timeout", kFanTurnOffTimeout));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  safety_manager_node_ = std::make_shared<SafetyManagerNodeWrapper>(
    "test_safety_manager_node", options);

  e_stop_pub_ = safety_manager_node_->create_publisher<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  battery_state_pub_ = safety_manager_node_->create_publisher<BatteryStateMsg>("battery", 3);
  io_state_pub_ = safety_manager_node_->create_publisher<IOStateMsg>(
    "hardware/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  driver_state_pub_ = safety_manager_node_->create_publisher<DriverStateMsg>(
    "driver/motor_controllers_state", 3);
  system_status_pub_ = safety_manager_node_->create_publisher<SystemStatusMsg>("system_status", 3);

  fan_server_ = safety_manager_node_->create_service<SetBoolSrv>(
    "hardware/fan_enable", std::bind(&TestSafetyBehaviorTree::FanServerCB, this, _1, _2));
  aux_power_server_ = safety_manager_node_->create_service<SetBoolSrv>(
    "hardware/aux_power_enable",
    std::bind(&TestSafetyBehaviorTree::AUXPowerServerCB, this, _1, _2));
  e_stop_trigger_server_ = safety_manager_node_->create_service<TriggerSrv>(
    "hardware/e_stop_trigger", std::bind(&TestSafetyBehaviorTree::EStopTriggerSrvCB, this, _1, _2));
}

TestSafetyBehaviorTree::~TestSafetyBehaviorTree() { rclcpp::shutdown(); }

bool TestSafetyBehaviorTree::SpinWhileRunning()
{
  return behavior_tree::test_utils::SpinWhileRunning(
    safety_manager_node_, [&]() { return safety_manager_node_->GetSafetyTreeStatus(); },
    std::chrono::milliseconds(1000));
}

void TestSafetyBehaviorTree::PublishDefaultStates()
{
  PublishEStop(false);
  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD,
    0.0);
  PublishIOState(false, true);
  PublishDriverState(0.0);
  PublishSystemStatus(0.0);
}

void TestSafetyBehaviorTree::PublishEStop(const bool data)
{
  BoolMsg msg;
  msg.data = data;
  e_stop_pub_->publish(msg);
}

void TestSafetyBehaviorTree::PublishBatteryState(
  const std::uint8_t status, const std::uint8_t health, const float temperature)
{
  BatteryStateMsg msg;
  msg.power_supply_status = status;
  msg.power_supply_health = health;
  msg.temperature = temperature;
  battery_state_pub_->publish(msg);
}

void TestSafetyBehaviorTree::PublishIOState(const bool fan, const bool aux_power)
{
  IOStateMsg msg;
  msg.fan = fan;
  msg.aux_power = aux_power;
  io_state_pub_->publish(msg);
}

void TestSafetyBehaviorTree::PublishDriverState(const float temperature)
{
  DriverStateMsg msg;
  msg.front.temperature = temperature;
  msg.rear.temperature = temperature;
  driver_state_pub_->publish(msg);
}

void TestSafetyBehaviorTree::PublishSystemStatus(const float cpu_temp)
{
  SystemStatusMsg msg;
  msg.cpu_temp = cpu_temp;
  system_status_pub_->publish(msg);
}

void TestSafetyBehaviorTree::FanServerCB(
  const SetBoolSrv::Request::SharedPtr & request, SetBoolSrv::Response::SharedPtr response)
{
  fan_state_ = request->data;
  PublishIOState(fan_state_, aux_power_state_);
  response->success = true;
}

void TestSafetyBehaviorTree::AUXPowerServerCB(
  const SetBoolSrv::Request::SharedPtr & request, SetBoolSrv::Response::SharedPtr response)
{
  aux_power_state_ = request->data;
  PublishIOState(fan_state_, aux_power_state_);
  response->success = true;
}

void TestSafetyBehaviorTree::EStopTriggerSrvCB(
  const TriggerSrv::Request::SharedPtr & /*request*/, TriggerSrv::Response::SharedPtr response)
{
  e_stop_triggered_ = true;
  response->success = true;
}

TEST_F(TestSafetyBehaviorTree, TurnOnFanAtStartup)
{
  safety_manager_node_->Initialize();
  PublishDefaultStates();

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_TRUE(fan_state_);

  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);
}

TEST_F(TestSafetyBehaviorTree, TurnOnFanCpuTemp)
{
  const float high_cpu_temp = 70.1;
  const float edge_cpu_temp = 59.9;

  safety_manager_node_->Initialize();
  PublishDefaultStates();

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_TRUE(fan_state_);

  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);

  PublishSystemStatus((2.0 * high_cpu_temp));
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());

  EXPECT_TRUE(fan_state_);

  PublishSystemStatus(3.0 * edge_cpu_temp - 2.0 * high_cpu_temp);
  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);
}

TEST_F(TestSafetyBehaviorTree, TurnOnFanDriverTemp)
{
  const float high_driver_temp = 45.1;
  const float edge_driver_temp = 34.9;

  safety_manager_node_->Initialize();
  PublishDefaultStates();

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_TRUE(fan_state_);

  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);

  PublishDriverState((2.0 * high_driver_temp));
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());

  EXPECT_TRUE(fan_state_);

  PublishDriverState(3.0 * edge_driver_temp - 2.0 * high_driver_temp);
  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);
}

TEST_F(TestSafetyBehaviorTree, PowerSupplyHealthOvervoltage)
{
  safety_manager_node_->Initialize();
  PublishDefaultStates();
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(e_stop_triggered_);

  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE, 20.0);

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_TRUE(e_stop_triggered_);
}

TEST_F(TestSafetyBehaviorTree, PowerSupplyHealthDead)
{
  safety_manager_node_->Initialize();
  PublishDefaultStates();
  ASSERT_TRUE(SpinWhileRunning());

  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD,
    20.0);

  ASSERT_TRUE(SpinWhileRunning());
  // Shutdown of the robot should be invoked so the shutdown tree status should change
  EXPECT_NE(BT::NodeStatus::IDLE, safety_manager_node_->GetShutdownTreeStatus());
}

TEST_F(TestSafetyBehaviorTree, PowerSupplyHealthOverheatTurnOnFan)
{
  safety_manager_node_->Initialize();
  PublishDefaultStates();
  ASSERT_TRUE(SpinWhileRunning());

  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(e_stop_triggered_);
  EXPECT_TRUE(aux_power_state_);
  EXPECT_FALSE(fan_state_);

  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT,
    20.0);

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(e_stop_triggered_);
  EXPECT_TRUE(aux_power_state_);
  EXPECT_TRUE(fan_state_);

  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD,
    20.0);

  // check if change of power supply health turns of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);
}

TEST_F(TestSafetyBehaviorTree, PowerSupplyHealthOverheatCriticalTemp)
{
  const float critical_bat_temp = 55.1;

  safety_manager_node_->Initialize();
  PublishDefaultStates();
  ASSERT_TRUE(SpinWhileRunning());

  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(e_stop_triggered_);
  EXPECT_TRUE(aux_power_state_);
  EXPECT_FALSE(fan_state_);

  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT,
    2.0 * critical_bat_temp);

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_TRUE(e_stop_triggered_);
  EXPECT_FALSE(aux_power_state_);
  EXPECT_TRUE(fan_state_);
}

TEST_F(TestSafetyBehaviorTree, PowerSupplyHealthOverheatFatalTemp)
{
  const float fatal_bat_temp = 62.1;

  safety_manager_node_->Initialize();
  PublishDefaultStates();
  ASSERT_TRUE(SpinWhileRunning());

  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT,
    2.0 * fatal_bat_temp);

  ASSERT_TRUE(SpinWhileRunning());
  // Shutdown of the robot should be invoked so the shutdown tree status should change
  EXPECT_NE(BT::NodeStatus::IDLE, safety_manager_node_->GetShutdownTreeStatus());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
