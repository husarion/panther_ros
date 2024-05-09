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

#include "panther_manager/lights_manager_node.hpp"

#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_ros2/plugins.hpp"
#include "panther_manager/bt_utils.hpp"
#include "panther_utils/moving_average.hpp"

namespace panther_manager
{

LightsManagerNode::LightsManagerNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  DeclareParameters();

  const auto battery_percent_window_len =
    this->get_parameter("battery_percent_window_len").as_int();

  battery_percent_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    battery_percent_window_len, 1.0);

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void LightsManagerNode::Initialize()
{
  RegisterBehaviorTree();

  CreateLightsTree();

  using namespace std::placeholders;

  battery_sub_ = this->create_subscription<BatteryStateMsg>(
    "battery", 10, std::bind(&LightsManagerNode::BatteryCB, this, _1));
  e_stop_sub_ = this->create_subscription<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&LightsManagerNode::EStopCB, this, _1));

  const float timer_freq = this->get_parameter("timer_frequency").as_double();
  const auto timer_period_ms =
    std::chrono::milliseconds(static_cast<unsigned>(1.0f / timer_freq * 1000));

  lights_tree_timer_ = this->create_wall_timer(
    timer_period_ms, std::bind(&LightsManagerNode::LightsTreeTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

void LightsManagerNode::DeclareParameters()
{
  const auto panther_manager_pkg_path =
    ament_index_cpp::get_package_share_directory("panther_manager");
  const std::string default_bt_project_path = panther_manager_pkg_path +
                                              "/behavior_trees/PantherLightsBT.btproj";
  const std::vector<std::string> default_plugin_libs = {};

  this->declare_parameter<std::string>("bt_project_path", default_bt_project_path);
  this->declare_parameter<std::vector<std::string>>("plugin_libs", default_plugin_libs);
  this->declare_parameter<std::vector<std::string>>("ros_plugin_libs", default_plugin_libs);

  this->declare_parameter<int>("battery_percent_window_len", 6);
  this->declare_parameter<float>("critical_battery_anim_period", 15.0);
  this->declare_parameter<float>("critical_battery_threshold_percent", 0.1);
  this->declare_parameter<float>("battery_state_anim_period", 120.0);
  this->declare_parameter<float>("low_battery_anim_period", 30.0);
  this->declare_parameter<float>("low_battery_threshold_percent", 0.4);
  this->declare_parameter<float>("update_charging_anim_step", 0.1);
  this->declare_parameter<float>("timer_frequency", 10.0);
}

void LightsManagerNode::RegisterBehaviorTree()
{
  const auto bt_project_path = this->get_parameter("bt_project_path").as_string();
  const auto plugin_libs = this->get_parameter("plugin_libs").as_string_array();
  const auto ros_plugin_libs = this->get_parameter("ros_plugin_libs").as_string_array();

  RCLCPP_INFO(this->get_logger(), "Register BehaviorTree from: %s", bt_project_path.c_str());

  bt_utils::RegisterBehaviorTree(
    factory_, bt_project_path, plugin_libs, this->shared_from_this(), ros_plugin_libs);
}

void LightsManagerNode::CreateLightsTree()
{
  update_charging_anim_step_ = this->get_parameter("update_charging_anim_step").as_double();
  const float critical_battery_anim_period =
    this->get_parameter("critical_battery_anim_period").as_double();
  const float critical_battery_threshold_percent =
    this->get_parameter("critical_battery_threshold_percent").as_double();
  const float battery_state_anim_period =
    this->get_parameter("battery_state_anim_period").as_double();
  const float low_battery_anim_period = this->get_parameter("low_battery_anim_period").as_double();
  const float low_battery_threshold_percent =
    this->get_parameter("low_battery_threshold_percent").as_double();

  const std::map<std::string, std::any> lights_initial_bb = {
    {"charging_anim_percent", ""},
    {"current_anim_id", -1},
    {"BATTERY_STATE_ANIM_PERIOD", battery_state_anim_period},
    {"CRITICAL_BATTERY_ANIM_PERIOD", critical_battery_anim_period},
    {"CRITICAL_BATTERY_THRESHOLD_PERCENT", critical_battery_threshold_percent},
    {"LOW_BATTERY_ANIM_PERIOD", low_battery_anim_period},
    {"LOW_BATTERY_THRESHOLD_PERCENT", low_battery_threshold_percent},
    // anim constants
    {"E_STOP_ANIM_ID", unsigned(LEDAnimationMsg::E_STOP)},
    {"READY_ANIM_ID", unsigned(LEDAnimationMsg::READY)},
    {"ERROR_ANIM_ID", unsigned(LEDAnimationMsg::ERROR)},
    {"MANUAL_ACTION_ANIM_ID", unsigned(LEDAnimationMsg::MANUAL_ACTION)},
    {"AUTONOMOUS_ACTION_ANIM_ID", unsigned(LEDAnimationMsg::AUTONOMOUS_ACTION)},
    {"GOAL_ACHIEVED_ANIM_ID", unsigned(LEDAnimationMsg::GOAL_ACHIEVED)},
    {"LOW_BATTERY_ANIM_ID", unsigned(LEDAnimationMsg::LOW_BATTERY)},
    {"CRITICAL_BATTERY_ANIM_ID", unsigned(LEDAnimationMsg::CRITICAL_BATTERY)},
    {"BATTERY_STATE_ANIM_ID", unsigned(LEDAnimationMsg::BATTERY_STATE)},
    {"CHARGING_BATTERY_ANIM_ID", unsigned(LEDAnimationMsg::CHARGING_BATTERY)},
    // battery status constants
    {"POWER_SUPPLY_STATUS_UNKNOWN", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN)},
    {"POWER_SUPPLY_STATUS_CHARGING", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING)},
    {"POWER_SUPPLY_STATUS_DISCHARGING", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING)},
    {"POWER_SUPPLY_STATUS_NOT_CHARGING",
     unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING)},
    {"POWER_SUPPLY_STATUS_FULL", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_FULL)},
    // battery health constants
    {"POWER_SUPPLY_HEALTH_OVERHEAT", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT)},
  };

  lights_config_ = bt_utils::CreateBTConfig(lights_initial_bb);
  lights_tree_ = factory_.createTree("Lights", lights_config_.blackboard);
  lights_bt_publisher_ = std::make_unique<BT::Groot2Publisher>(lights_tree_, 5555);
}

void LightsManagerNode::BatteryCB(const BatteryStateMsg::SharedPtr battery_state)
{
  const auto battery_status = battery_state->power_supply_status;
  const auto battery_health = battery_state->power_supply_health;
  lights_config_.blackboard->set<unsigned>("battery_status", battery_status);
  lights_config_.blackboard->set<unsigned>("battery_health", battery_health);

  // don't update battery percentage if unknown status or health
  if (
    battery_status != BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN &&
    battery_health != BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
    battery_percent_ma_->Roll(battery_state->percentage);
  }

  lights_config_.blackboard->set<float>("battery_percent", battery_percent_ma_->GetAverage());
  lights_config_.blackboard->set<std::string>(
    "battery_percent_round",
    std::to_string(
      round(battery_percent_ma_->GetAverage() / update_charging_anim_step_) *
      update_charging_anim_step_));
}

void LightsManagerNode::EStopCB(const BoolMsg::SharedPtr e_stop)
{
  lights_config_.blackboard->set<bool>("e_stop_state", e_stop->data);
}

void LightsManagerNode::LightsTreeTimerCB()
{
  if (!SystemReady()) {
    return;
  }

  lights_tree_status_ = lights_tree_.tickOnce();

  if (lights_tree_status_ == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(this->get_logger(), "Lights behavior tree returned FAILURE status");
  }
}

bool LightsManagerNode::SystemReady()
{
  if (
    !lights_config_.blackboard->getEntry("e_stop_state") ||
    !lights_config_.blackboard->getEntry("battery_status")) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for required system messages to arrive");
    return false;
  }

  return true;
}

}  // namespace panther_manager
