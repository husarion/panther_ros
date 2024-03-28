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

#include "panther_manager/manager_bt_node.hpp"

#include <algorithm>
#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
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

ManagerBTNode::ManagerBTNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  DeclareParameters();

  const auto battery_temp_window_len = this->get_parameter("battery_temp_window_len").as_int();
  const auto battery_percent_window_len =
    this->get_parameter("battery_percent_window_len").as_int();
  const auto cpu_temp_window_len = this->get_parameter("cpu_temp_window_len").as_int();
  const auto driver_temp_window_len = this->get_parameter("driver_temp_window_len").as_int();

  battery_temp_ma_ =
    std::make_unique<panther_utils::MovingAverage<double>>(battery_temp_window_len);
  battery_percent_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    battery_percent_window_len, 1.0);
  cpu_temp_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(cpu_temp_window_len);
  front_driver_temp_ma_ =
    std::make_unique<panther_utils::MovingAverage<double>>(driver_temp_window_len);
  rear_driver_temp_ma_ =
    std::make_unique<panther_utils::MovingAverage<double>>(driver_temp_window_len);

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void ManagerBTNode::Initialize()
{
  const auto launch_lights_tree = this->get_parameter("launch_lights_tree").as_bool();
  const auto launch_safety_tree = this->get_parameter("launch_safety_tree").as_bool();
  launch_shutdown_tree_ = this->get_parameter("launch_shutdown_tree").as_bool();

  if (launch_safety_tree && !launch_shutdown_tree_) {
    RCLCPP_ERROR(
      this->get_logger(), "Can't launch safety tree without shutdown tree. Killing node.");
    rclcpp::shutdown();
  }

  RegisterBehaviorTree();

  if (launch_lights_tree) {
    CreateLightsTree();
  }

  if (launch_safety_tree) {
    CreateSafetyTree();
  }

  if (launch_shutdown_tree_) {
    CreateShutdownTree();
  }

  // -------------------------------
  //   Subscribers
  // -------------------------------

  using namespace std::placeholders;

  battery_sub_ = this->create_subscription<BatteryStateMsg>(
    "battery", 10, std::bind(&ManagerBTNode::BatteryCB, this, _1));
  driver_state_sub_ = this->create_subscription<DriverStateMsg>(
    "driver/motor_controllers_state", 10, std::bind(&ManagerBTNode::DriverStateCB, this, _1));
  e_stop_sub_ = this->create_subscription<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&ManagerBTNode::EStopCB, this, _1));
  io_state_sub_ = this->create_subscription<IOStateMsg>(
    "hardware/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&ManagerBTNode::IOStateCB, this, _1));
  system_status_sub_ = this->create_subscription<SystemStatusMsg>(
    "system_status", 10, std::bind(&ManagerBTNode::SystemStatusCB, this, _1));

  // -------------------------------
  //   Timers
  // -------------------------------

  if (launch_lights_tree) {
    const float timer_freq = this->get_parameter("lights.timer_frequency").as_double();
    const auto timer_period_ms =
      std::chrono::milliseconds(static_cast<unsigned>(1.0f / timer_freq * 1000));

    lights_tree_timer_ = this->create_wall_timer(
      timer_period_ms, std::bind(&ManagerBTNode::LightsTreeTimerCB, this));
  }
  if (launch_safety_tree) {
    const float timer_freq = this->get_parameter("safety.timer_frequency").as_double();
    const auto timer_period_ms =
      std::chrono::milliseconds(static_cast<unsigned>(1.0f / timer_freq * 1000));

    safety_tree_timer_ = this->create_wall_timer(
      timer_period_ms, std::bind(&ManagerBTNode::SafetyTreeTimerCB, this));
  }

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

void ManagerBTNode::DeclareParameters()
{
  const auto panther_manager_pkg_path =
    ament_index_cpp::get_package_share_directory("panther_manager");
  const std::string default_bt_project_path = panther_manager_pkg_path +
                                              "/config/Panther12BT.btproj";
  const std::vector<std::string> default_plugin_libs = {};

  this->declare_parameter<bool>("launch_lights_tree", true);
  this->declare_parameter<bool>("launch_safety_tree", true);
  this->declare_parameter<bool>("launch_shutdown_tree", true);
  this->declare_parameter<std::string>("bt_project_path", default_bt_project_path);
  this->declare_parameter<std::vector<std::string>>("plugin_libs", default_plugin_libs);
  this->declare_parameter<std::vector<std::string>>("ros_plugin_libs", default_plugin_libs);
  this->declare_parameter<int>("battery_temp_window_len", 6);
  this->declare_parameter<int>("battery_percent_window_len", 6);
  this->declare_parameter<int>("cpu_temp_window_len", 6);
  this->declare_parameter<int>("driver_temp_window_len", 6);
  this->declare_parameter<std::string>("shutdown_hosts_path", "");

  // lights tree params
  this->declare_parameter<float>("lights.critical_battery_anim_period", 15.0);
  this->declare_parameter<float>("lights.critical_battery_threshold_percent", 0.1);
  this->declare_parameter<float>("lights.battery_state_anim_period", 120.0);
  this->declare_parameter<float>("lights.low_battery_anim_period", 30.0);
  this->declare_parameter<float>("lights.low_battery_threshold_percent", 0.4);
  this->declare_parameter<float>("lights.update_charging_anim_step", 0.1);
  this->declare_parameter<float>("lights.timer_frequency", 10.0);

  // safety tree params
  this->declare_parameter<float>("safety.cpu_fan_on_temp", 70.0);
  this->declare_parameter<float>("safety.cpu_fan_off_temp", 60.0);
  this->declare_parameter<float>("safety.driver_fan_on_temp", 45.0);
  this->declare_parameter<float>("safety.driver_fan_off_temp", 35.0);
  this->declare_parameter<float>("safety.timer_frequency", 10.0);
}

void ManagerBTNode::RegisterBehaviorTree()
{
  const auto bt_project_path = this->get_parameter("bt_project_path").as_string();
  const auto plugin_libs = this->get_parameter("plugin_libs").as_string_array();
  const auto ros_plugin_libs = this->get_parameter("ros_plugin_libs").as_string_array();

  RCLCPP_INFO(this->get_logger(), "Register BehaviorTree from: %s", bt_project_path.c_str());

  for (const auto & plugin : plugin_libs) {
    factory_.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
  }

  for (const auto & plugin : ros_plugin_libs) {
    BT::RosNodeParams params;
    params.nh = this->shared_from_this();
    RegisterRosNode(factory_, BT::SharedLibrary::getOSName(plugin), params);
  }

  factory_.registerBehaviorTreeFromFile(bt_project_path);
}

void ManagerBTNode::CreateLightsTree()
{
  update_charging_anim_step_ = this->get_parameter("lights.update_charging_anim_step").as_double();
  const float critical_battery_anim_period =
    this->get_parameter("lights.critical_battery_anim_period").as_double();
  const float critical_battery_threshold_percent =
    this->get_parameter("lights.critical_battery_threshold_percent").as_double();
  const float battery_state_anim_period =
    this->get_parameter("lights.battery_state_anim_period").as_double();
  const float low_battery_anim_period =
    this->get_parameter("lights.low_battery_anim_period").as_double();
  const float low_battery_threshold_percent =
    this->get_parameter("lights.low_battery_threshold_percent").as_double();

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

void ManagerBTNode::CreateSafetyTree()
{
  const float cpu_fan_on_temp = this->get_parameter("safety.cpu_fan_on_temp").as_double();
  const float cpu_fan_off_temp = this->get_parameter("safety.cpu_fan_off_temp").as_double();
  const float driver_fan_on_temp = this->get_parameter("safety.driver_fan_on_temp").as_double();
  const float driver_fan_off_temp = this->get_parameter("safety.driver_fan_off_temp").as_double();

  const std::map<std::string, std::any> safety_initial_bb = {
    {"CPU_FAN_OFF_TEMP", cpu_fan_off_temp},
    {"CPU_FAN_ON_TEMP", cpu_fan_on_temp},
    {"DRIVER_FAN_OFF_TEMP", driver_fan_off_temp},
    {"DRIVER_FAN_ON_TEMP", driver_fan_on_temp},
    {"CRITICAL_BAT_TEMP", kCriticalBatteryTemp},
    {"FATAL_BAT_TEMP", kFatalBatteryTemp},
    // battery health constants
    {"POWER_SUPPLY_HEALTH_UNKNOWN", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN)},
    {"POWER_SUPPLY_HEALTH_GOOD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD)},
    {"POWER_SUPPLY_HEALTH_OVERHEAT", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT)},
    {"POWER_SUPPLY_HEALTH_DEAD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD)},
    {"POWER_SUPPLY_HEALTH_OVERVOLTAGE", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE)},
    {"POWER_SUPPLY_HEALTH_UNSPEC_FAILURE",
     unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)},
    {"POWER_SUPPLY_HEALTH_COLD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD)},
    {"POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE",
     unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE)},
    {"POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE",
     unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE)},
  };

  safety_config_ = bt_utils::CreateBTConfig(safety_initial_bb);
  safety_tree_ = factory_.createTree("Safety", safety_config_.blackboard);
  safety_bt_publisher_ = std::make_unique<BT::Groot2Publisher>(safety_tree_, 6666);
}

void ManagerBTNode::CreateShutdownTree()
{
  const auto shutdown_hosts_path = this->get_parameter("shutdown_hosts_path").as_string();

  const std::map<std::string, std::any> shutdown_initial_bb = {
    {"SHUTDOWN_HOSTS_FILE", shutdown_hosts_path.c_str()},
  };

  shutdown_config_ = bt_utils::CreateBTConfig(shutdown_initial_bb);
  shutdown_tree_ = factory_.createTree("Shutdown", shutdown_config_.blackboard);
  shutdown_bt_publisher_ = std::make_unique<BT::Groot2Publisher>(shutdown_tree_, 7777);
}

void ManagerBTNode::BatteryCB(const BatteryStateMsg::SharedPtr battery)
{
  battery_status_ = battery->power_supply_status;
  battery_health_ = battery->power_supply_health;
  // don't update battery data if unknown status
  if (battery_status_ == BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN) {
    return;
  }
  if (battery_health_ == BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
    return;
  }

  battery_temp_ma_->Roll(battery->temperature);
  battery_percent_ma_->Roll(battery->percentage);
}

void ManagerBTNode::DriverStateCB(const DriverStateMsg::SharedPtr driver_state)
{
  front_driver_temp_ma_->Roll(driver_state->front.temperature);
  rear_driver_temp_ma_->Roll(driver_state->rear.temperature);
}

void ManagerBTNode::EStopCB(const BoolMsg::SharedPtr e_stop) { e_stop_state_ = e_stop->data; }

void ManagerBTNode::IOStateCB(const IOStateMsg::SharedPtr io_state)
{
  if (io_state->power_button && launch_shutdown_tree_) {
    ShutdownRobot("Power button pressed");
  }
  io_state_ = io_state;
}

void ManagerBTNode::SystemStatusCB(const SystemStatusMsg::SharedPtr system_status)
{
  cpu_temp_ma_->Roll(system_status->cpu_temp);
}

void ManagerBTNode::LightsTreeTimerCB()
{
  if (!SystemReady()) {
    return;
  }

  lights_config_.blackboard->set<bool>("e_stop_state", e_stop_state_.value());
  lights_config_.blackboard->set<unsigned>("battery_status", battery_status_.value());
  lights_config_.blackboard->set<unsigned>("battery_health", battery_health_.value());
  lights_config_.blackboard->set<float>("battery_percent", battery_percent_ma_->GetAverage());
  lights_config_.blackboard->set<std::string>(
    "battery_percent_round",
    std::to_string(
      round(battery_percent_ma_->GetAverage() / update_charging_anim_step_) *
      update_charging_anim_step_));

  lights_tree_status_ = lights_tree_.tickOnce();

  if (lights_tree_status_ == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(this->get_logger(), "Lights behavior tree returned FAILURE status");
  }
}

void ManagerBTNode::SafetyTreeTimerCB()
{
  if (!SystemReady()) {
    return;
  }

  safety_config_.blackboard->set<bool>("aux_state", io_state_.value()->aux_power);
  safety_config_.blackboard->set<bool>("e_stop_state", e_stop_state_.value());
  safety_config_.blackboard->set<bool>("fan_state", io_state_.value()->fan);
  safety_config_.blackboard->set<unsigned>("battery_status", battery_status_.value());
  safety_config_.blackboard->set<unsigned>("battery_health", battery_health_.value());
  safety_config_.blackboard->set<double>("bat_temp", battery_temp_ma_->GetAverage());
  safety_config_.blackboard->set<double>("cpu_temp", cpu_temp_ma_->GetAverage());
  // to simplify conditions pass only higher temp of motor drivers
  safety_config_.blackboard->set<double>(
    "driver_temp",
    std::max({front_driver_temp_ma_->GetAverage(), rear_driver_temp_ma_->GetAverage()}));

  safety_tree_status_ = safety_tree_.tickOnce();

  if (safety_tree_status_ == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(this->get_logger(), "Safety behavior tree returned FAILURE status");
  }

  std::pair<bool, std::string> signal_shutdown;
  if (safety_config_.blackboard->get<std::pair<bool, std::string>>(
        "signal_shutdown", signal_shutdown)) {
    if (signal_shutdown.first) {
      ShutdownRobot(signal_shutdown.second);
    }
  }
}

bool ManagerBTNode::SystemReady()
{
  if (e_stop_state_.has_value() && io_state_.has_value() && battery_status_.has_value()) {
    return true;
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000, "Waiting for required system messages to arrive");
  return false;
}

void ManagerBTNode::ShutdownRobot(const std::string & reason)
{
  RCLCPP_WARN(this->get_logger(), "Soft shutdown initialized. %s", reason.c_str());
  lights_tree_timer_->cancel();
  lights_tree_.haltTree();
  safety_tree_timer_->cancel();
  safety_tree_.haltTree();

  // tick shutdown tree
  shutdown_tree_status_ = BT::NodeStatus::RUNNING;
  auto start_time = this->get_clock()->now();
  rclcpp::Rate rate(30.0);  // 30 Hz
  while (rclcpp::ok() && shutdown_tree_status_ == BT::NodeStatus::RUNNING) {
    shutdown_tree_status_ = shutdown_tree_.tickOnce();
    rate.sleep();
  }

  if (shutdown_tree_status_ == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(
      this->get_logger(),
      "Shutdown behavior tree returned FAILURE status, robot may not be shutdown correctly");
  }

  rclcpp::shutdown();
}

}  // namespace panther_manager
