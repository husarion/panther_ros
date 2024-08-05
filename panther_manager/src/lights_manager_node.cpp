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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include "panther_msgs/msg/led_animation.hpp"

#include "panther_manager/behavior_tree_manager.hpp"
#include "panther_manager/behavior_tree_utils.hpp"

namespace panther_manager
{

LightsManagerNode::LightsManagerNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  DeclareParameters();

  const auto initial_blackboard = CreateLightsInitialBlackboard();
  lights_tree_manager_ = std::make_unique<BehaviorTreeManager>("Lights", initial_blackboard, 5555);

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void LightsManagerNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  RegisterBehaviorTree();
  lights_tree_manager_->Initialize(factory_);

  bt_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  using namespace std::placeholders;

  const float timer_freq = this->get_parameter("timer_frequency").as_double();
  const auto timer_period_ms =
    std::chrono::milliseconds(static_cast<unsigned>(1.0f / timer_freq * 1000));

  lights_tree_timer_ = this->create_wall_timer(
    timer_period_ms, std::bind(&LightsManagerNode::LightsTreeTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
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

  this->declare_parameter<int>("battery.percent.window_len", 6);
  this->declare_parameter<float>("battery.percent.threshold.low", 0.4);
  this->declare_parameter<float>("battery.percent.threshold.critical", 0.1);
  this->declare_parameter<float>("battery.animation_period.low", 30.0);
  this->declare_parameter<float>("battery.animation_period.critical", 15.0);
  this->declare_parameter<float>("battery.charging_anim_step", 0.1);
  this->declare_parameter<float>("timer_frequency", 10.0);
}

void LightsManagerNode::RegisterBehaviorTree()
{
  const auto bt_project_path = this->get_parameter("bt_project_path").as_string();
  const auto plugin_libs = this->get_parameter("plugin_libs").as_string_array();
  const auto ros_plugin_libs = this->get_parameter("ros_plugin_libs").as_string_array();

  behavior_tree_utils::RegisterBehaviorTree(
    factory_, bt_project_path, plugin_libs, this->shared_from_this(), ros_plugin_libs);

  RCLCPP_INFO(
    this->get_logger(), "BehaviorTree registered from path '%s'", bt_project_path.c_str());
}

std::map<std::string, std::any> LightsManagerNode::CreateLightsInitialBlackboard()
{
  const std::string undefined_charging_anim_percent = "";
  const int undefined_anim_id = -1;

  const int battery_percent_window_len = this->get_parameter("battery.percent.window_len").as_int();
  const float battery_charging_anim_step =
    this->get_parameter("battery.charging_anim_step").as_double();

  const float critical_battery_anim_period =
    this->get_parameter("battery.animation_period.critical").as_double();
  const float critical_battery_threshold_percent =
    this->get_parameter("battery.percent.threshold.critical").as_double();
  const float low_battery_anim_period =
    this->get_parameter("battery.animation_period.low").as_double();
  const float low_battery_threshold_percent =
    this->get_parameter("battery.percent.threshold.low").as_double();

  using LEDAnimationMsg = panther_msgs::msg::LEDAnimation;
  using BatteryStateMsg = sensor_msgs::msg::BatteryState;

  const std::map<std::string, std::any> lights_initial_bb = {
    {"current_anim_id", undefined_anim_id},
    {"battery_percent_window_len", battery_percent_window_len},
    {"battery_charging_anim_setp", battery_charging_anim_step},
    {"charging_anim_percent", undefined_charging_anim_percent},
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

  RCLCPP_INFO(this->get_logger(), "Blackboard created.");
  return lights_initial_bb;
}

void LightsManagerNode::LightsTreeTimerCB()
{
  lights_tree_manager_->TickOnce();

  if (lights_tree_manager_->GetTreeStatus() == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(this->get_logger(), "Lights behavior tree returned FAILURE status");
  }
}

}  // namespace panther_manager
