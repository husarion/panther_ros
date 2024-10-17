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

#include "panther_manager/robot_states_manager_node.hpp"

#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

#include "panther_utils/moving_average.hpp"

#include <panther_manager/behavior_tree_manager.hpp>
#include <panther_manager/behavior_tree_utils.hpp>

namespace panther_manager
{

RobotStateManagerNode::RobotStateManagerNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  DeclareParameters();

  const auto battery_percent_window_len =
    this->get_parameter("battery.percent.window_len").as_int();

  battery_percent_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    battery_percent_window_len, 1.0);

  const auto initial_blackboard = CreateBlackboard();
  docking_tree_manager_ = std::make_unique<BehaviorTreeManager>(
    "RobotStates", initial_blackboard, 5557);

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void RobotStateManagerNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  RegisterBehaviorTree();
  docking_tree_manager_->Initialize(factory_);

  using namespace std::placeholders;

  battery_sub_ = this->create_subscription<BatteryStateMsg>(
    "battery/battery_status", 10, std::bind(&RobotStateManagerNode::BatteryCB, this, _1));
  e_stop_sub_ = this->create_subscription<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&RobotStateManagerNode::EStopCB, this, _1));
  robot_state_pub_ = this->create_publisher<Int8Msg>(
    "robot_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  docking_srv_ = this->create_service<SetBoolSrv>(
    "docking", std::bind(&RobotStateManagerNode::DockingSrvCB, this, _1, _2));

  const float timer_freq = this->get_parameter("timer_frequency").as_double();
  const auto timer_period_ms =
    std::chrono::milliseconds(static_cast<unsigned>(1.0f / timer_freq * 1000));

  docking_tree_timer_ = this->create_wall_timer(
    timer_period_ms, std::bind(&RobotStateManagerNode::DockingTreeTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void RobotStateManagerNode::DeclareParameters()
{
  const auto panther_manager_pkg_path =
    ament_index_cpp::get_package_share_directory("panther_manager");
  const std::string default_bt_project_path = panther_manager_pkg_path +
                                              "/behavior_trees/RobotStatesBT.btproj";
  const std::vector<std::string> default_plugin_libs = {};

  this->declare_parameter<std::string>("bt_project_path", default_bt_project_path);
  this->declare_parameter<std::vector<std::string>>("plugin_libs", default_plugin_libs);
  this->declare_parameter<std::vector<std::string>>("ros_plugin_libs", default_plugin_libs);
  this->declare_parameter<double>("ros_communication_timeout.availability", 1.0);
  this->declare_parameter<double>("ros_communication_timeout.response", 1.0);

  this->declare_parameter<int>("battery.percent.window_len", 6);
  this->declare_parameter<float>("battery.percent.threshold.low", 0.4);
  this->declare_parameter<float>("battery.percent.threshold.critical", 0.1);
  this->declare_parameter<float>("battery.animation_period.low", 30.0);
  this->declare_parameter<float>("battery.animation_period.critical", 15.0);
  this->declare_parameter<float>("battery.charging_anim_step", 0.1);
  this->declare_parameter<float>("timer_frequency", 10.0);
}

void RobotStateManagerNode::RegisterBehaviorTree()
{
  const auto bt_project_path = this->get_parameter("bt_project_path").as_string();

  const auto plugin_libs = this->get_parameter("plugin_libs").as_string_array();
  const auto ros_plugin_libs = this->get_parameter("ros_plugin_libs").as_string_array();

  const auto service_availability_timeout =
    this->get_parameter("ros_communication_timeout.availability").as_double();
  const auto service_response_timeout =
    this->get_parameter("ros_communication_timeout.response").as_double();

  BT::RosNodeParams params;
  params.nh = this->shared_from_this();
  params.wait_for_server_timeout =
    std::chrono::milliseconds(static_cast<int>(service_availability_timeout * 1000));
  params.server_timeout =
    std::chrono::milliseconds(static_cast<int>(service_response_timeout * 1000));

  behavior_tree_utils::RegisterBehaviorTree(
    factory_, bt_project_path, plugin_libs, params, ros_plugin_libs);

  RCLCPP_INFO(
    this->get_logger(), "BehaviorTree registered from path '%s'", bt_project_path.c_str());
}

std::map<std::string, std::any> RobotStateManagerNode::CreateBlackboard()
{
  update_charging_anim_step_ = this->get_parameter("battery.charging_anim_step").as_double();
  const float critical_battery_anim_period =
    this->get_parameter("battery.animation_period.critical").as_double();
  const float critical_battery_threshold_percent =
    this->get_parameter("battery.percent.threshold.critical").as_double();
  const float low_battery_anim_period =
    this->get_parameter("battery.animation_period.low").as_double();
  const float low_battery_threshold_percent =
    this->get_parameter("battery.percent.threshold.low").as_double();

  const std::string undefined_charging_anim_percent = "";
  const int undefined_anim_id = -1;

  const std::map<std::string, std::any> docking_initial_bb = {
    {"charging_anim_percent", undefined_charging_anim_percent},
    {"current_anim_id", undefined_anim_id},
    {"docking_cmd", 0},
    {"robot_state", 0},
    {"CRITICAL_BATTERY_ANIM_PERIOD", critical_battery_anim_period},
    {"CRITICAL_BATTERY_THRESHOLD_PERCENT", critical_battery_threshold_percent},
    {"LOW_BATTERY_ANIM_PERIOD", low_battery_anim_period},
    {"LOW_BATTERY_THRESHOLD_PERCENT", low_battery_threshold_percent},
    // robot states
    {"ROBOT_STATE_ERROR", -1},
    {"ROBOT_STATE_ESTOP", 0},
    {"ROBOT_STATE_STANDBY", 1},
    {"ROBOT_STATE_DOCKING", 2},
    {"ROBOT_STATE_SUCCESS", 3},
    // robot states
    {"DOCKING_CMD_NONE", 0},
    {"DOCKING_CMD_DOCK", 1},
    {"DOCKING_CMD_UNDOCK", 2},
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
  return docking_initial_bb;
}

void RobotStateManagerNode::BatteryCB(const BatteryStateMsg::SharedPtr battery_state)
{
  const auto battery_status = battery_state->power_supply_status;
  const auto battery_health = battery_state->power_supply_health;
  docking_tree_manager_->GetBlackboard()->set<unsigned>("battery_status", battery_status);
  docking_tree_manager_->GetBlackboard()->set<unsigned>("battery_health", battery_health);

  // don't update battery percentage if unknown status or health
  if (
    battery_status != BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN &&
    battery_health != BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
    battery_percent_ma_->Roll(battery_state->percentage);
  }

  docking_tree_manager_->GetBlackboard()->set<float>(
    "battery_percent", battery_percent_ma_->GetAverage());
  docking_tree_manager_->GetBlackboard()->set<std::string>(
    "battery_percent_round",
    std::to_string(
      round(battery_percent_ma_->GetAverage() / update_charging_anim_step_) *
      update_charging_anim_step_));
}

void RobotStateManagerNode::EStopCB(const BoolMsg::SharedPtr e_stop)
{
  docking_tree_manager_->GetBlackboard()->set<bool>("e_stop_state", e_stop->data);
}

void RobotStateManagerNode::DockingSrvCB(
  const SetBoolSrv::Request::SharedPtr & request, SetBoolSrv::Response::SharedPtr response)
{
  int robot_state;
  if (!docking_tree_manager_->GetBlackboard()->get("robot_state", robot_state)) {
    throw std::runtime_error("Cannot get robot state from blackboard.");
  }

  if (robot_state != 1) {
    response->success = false;
    response->message = "Cannot start docking/undocking robot is not in STANDBY state.";
    return;
  }

  if (request->data) {
    docking_tree_manager_->GetBlackboard()->set<int>("docking_cmd", 1);
    response->message = "Docking Started";
  } else {
    docking_tree_manager_->GetBlackboard()->set<int>("docking_cmd", 2);
    response->message = "Undocking Started";
  }
  response->success = true;
}

void RobotStateManagerNode::DockingTreeTimerCB()
{
  if (!SystemReady()) {
    return;
  }

  docking_tree_manager_->TickOnce();
  PublishRobotState();

  if (docking_tree_manager_->GetTreeStatus() == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(this->get_logger(), "Docking behavior tree returned FAILURE status");
  }
}

void RobotStateManagerNode::PublishRobotState()
{
  Int8Msg robot_state;
  if (docking_tree_manager_->GetBlackboard()->get("robot_state", robot_state.data)) {
    robot_state_pub_->publish(robot_state);
  }
}

bool RobotStateManagerNode::SystemReady()
{
  if (
    !docking_tree_manager_->GetBlackboard()->getEntry("e_stop_state") ||
    !docking_tree_manager_->GetBlackboard()->getEntry("battery_status")) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for required system messages to arrive.");
    return false;
  }

  return true;
}

}  // namespace panther_manager
