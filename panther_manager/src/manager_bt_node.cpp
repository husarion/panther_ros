#include <panther_manager/manager_bt_node.hpp>

#include <algorithm>
#include <any>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/utils/shared_library.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>

#include <panther_msgs/DriverState.h>
#include <panther_msgs/IOState.h>
#include <panther_msgs/LEDAnimation.h>
#include <panther_msgs/SystemStatus.h>

#include <panther_manager/moving_average.hpp>

namespace panther_manager
{

ManagerBTNode::ManagerBTNode(
  const std::shared_ptr<ros::NodeHandle> nh, const std::shared_ptr<ros::NodeHandle> ph)
: nh_(std::move(nh)), ph_(std::move(ph))
{
  node_name_ = ros::this_node::getName();

  const std::string default_xml =
    ros::package::getPath("panther_manager") + "/config/PantherManagerBT.xml";
  const std::vector<std::string> default_plugin_libs = {};

  const auto xml_filename = ph_->param<std::string>("xml_filename", default_xml);
  const auto plugin_libs = ph_->param<std::vector<std::string>>("plugin_libs", default_plugin_libs);
  const auto battery_temp_window_len = ph_->param<int>("battery_temp_window_len", 6);
  const auto battery_percent_window_len = ph->param<int>("battery_percent_window_len", 6);
  const auto cpu_temp_window_len = ph_->param<int>("cpu_temp_window_len", 6);
  const auto driver_temp_window_len = ph_->param<int>("driver_temp_window_len", 6);
  const auto shutdown_hosts_file = ph_->param<std::string>("shutdown_hosts_file", "");
  shutdown_timeout_ = ph_->param<float>("shutdown_timeout", 15.0);

  // lights tree params
  const auto critical_battery_anim_period =
    ph_->param<float>("lights/critical_battery_anim_period", 15.0);
  const auto critical_battery_threshold_percent =
    ph_->param<float>("lights/critical_battery_threshold_percent", 0.1);
  const auto battery_state_anim_period =
    ph_->param<float>("lights/battery_state_anim_period", 120.0);
  const auto low_battery_anim_period = ph_->param<float>("lights/low_battery_anim_period", 30.0);
  const auto low_battery_threshold_percent =
    ph_->param<float>("lights/low_battery_threshold_percent", 0.4);
  update_charging_anim_step_ = ph_->param<float>("lights/update_charging_anim_step", 0.1);

  // safety tree params
  const auto high_bat_temp = ph_->param<float>("safety/high_bat_temp", 55.0);
  const auto critical_bat_temp = ph_->param<float>("safety/critical_bat_temp", 59.0);
  const auto fatal_bat_temp = ph_->param<float>("safety/fatal_bat_temp", 62.0);
  const auto cpu_fan_on_temp = ph_->param<float>("safety/cpu_fan_on_temp", 70.0);
  const auto cpu_fan_off_temp = ph_->param<float>("safety/cpu_fan_off_temp", 60.0);
  const auto driver_fan_on_temp = ph_->param<float>("safety/driver_fan_on_temp", 45.0);
  const auto driver_fan_off_temp = ph_->param<float>("safety/driver_fan_off_temp", 35.0);

  battery_temp_ma_ = std::make_unique<MovingAverage<double>>(battery_temp_window_len);
  battery_percent_ma_ = std::make_unique<MovingAverage<double>>(battery_percent_window_len, 1.0);
  cpu_temp_ma_ = std::make_unique<MovingAverage<double>>(cpu_temp_window_len);
  front_driver_temp_ma_ = std::make_unique<MovingAverage<double>>(driver_temp_window_len);
  rear_driver_temp_ma_ = std::make_unique<MovingAverage<double>>(driver_temp_window_len);

  ROS_INFO("[%s] Register BehaviorTree from: %s", node_name_.c_str(), xml_filename.c_str());

  // export plugins for a behaviour tree
  for (const auto & p : plugin_libs) {
    factory_.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  factory_.registerBehaviorTreeFromFile(xml_filename);

  const std::map<std::string, std::any> lights_initial_bb = {
    {"CRITICAL_BATTERY_ANIM_PERIOD", critical_battery_anim_period},
    {"CRITICAL_BATTERY_THRESHOLD_PERCENT", critical_battery_threshold_percent},
    {"BATTERY_STATE_ANIM_PERIOD", battery_state_anim_period},
    {"LOW_BATTERY_ANIM_PERIOD", low_battery_anim_period},
    {"LOW_BATTERY_THRESHOLD_PERCENT", low_battery_threshold_percent},
    {"current_anim_id", -1},
    {"charging_anim_percent", ""},
    // anim constants
    {"E_STOP_ANIM_ID", unsigned(panther_msgs::LEDAnimation::E_STOP)},
    {"READY_ANIM_ID", unsigned(panther_msgs::LEDAnimation::READY)},
    {"ERROR_ANIM_ID", unsigned(panther_msgs::LEDAnimation::ERROR)},
    {"MANUAL_ACTION_ANIM_ID", unsigned(panther_msgs::LEDAnimation::MANUAL_ACTION)},
    {"AUTONOMOUS_ACTION_ANIM_ID", unsigned(panther_msgs::LEDAnimation::AUTONOMOUS_ACTION)},
    {"GOAL_ACHIEVED_ANIM_ID", unsigned(panther_msgs::LEDAnimation::GOAL_ACHIEVED)},
    {"LOW_BATTERY_ANIM_ID", unsigned(panther_msgs::LEDAnimation::LOW_BATTERY)},
    {"CRITICAL_BATTERY_ANIM_ID", unsigned(panther_msgs::LEDAnimation::CRITICAL_BATTERY)},
    {"BATTERY_STATE_ANIM_ID", unsigned(panther_msgs::LEDAnimation::BATTERY_STATE)},
    {"CHARGING_BATTERY_ANIM_ID", unsigned(panther_msgs::LEDAnimation::CHARGING_BATTERY)},
    // battery constants
    {"POWER_SUPPLY_STATUS_UNKNOWN",
     unsigned(sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN)},
    {"POWER_SUPPLY_STATUS_CHARGING",
     unsigned(sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)},
    {"POWER_SUPPLY_STATUS_DISCHARGING",
     unsigned(sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING)},
    {"POWER_SUPPLY_STATUS_NOT_CHARGING",
     unsigned(sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING)},
    {"POWER_SUPPLY_STATUS_FULL", unsigned(sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL)},
  };
  const std::map<std::string, std::any> safety_initial_bb = {
    {"HIGH_BAT_TEMP", high_bat_temp},
    {"CRITICAL_BAT_TEMP", critical_bat_temp},
    {"FATAL_BAT_TEMP", fatal_bat_temp},
    {"CPU_FAN_ON_TEMP", cpu_fan_on_temp},
    {"CPU_FAN_OFF_TEMP", cpu_fan_off_temp},
    {"DRIVER_FAN_ON_TEMP", driver_fan_on_temp},
    {"DRIVER_FAN_OFF_TEMP", driver_fan_off_temp},
  };
  const std::map<std::string, std::any> shutdown_initial_bb = {
    {"SHUTDOWN_HOSTS_FILE", shutdown_hosts_file.c_str()},
  };

  lights_config_ = create_bt_config(lights_initial_bb);
  safety_config_ = create_bt_config(safety_initial_bb);
  shutdown_config_ = create_bt_config(shutdown_initial_bb);

  lights_tree_ = factory_.createTree("Lights", lights_config_.blackboard);
  safety_tree_ = factory_.createTree("Safety", safety_config_.blackboard);
  shutdown_tree_ = factory_.createTree("Shutdown", shutdown_config_.blackboard);

  battery_sub_ = nh_->subscribe("battery", 10, &ManagerBTNode::battery_cb, this);
  driver_state_sub_ =
    nh_->subscribe("driver/motor_controllers_state", 10, &ManagerBTNode::driver_state_cb, this);
  e_stop_sub_ = nh_->subscribe("hardware/e_stop", 2, &ManagerBTNode::e_stop_cb, this);
  io_state_sub_ = nh_->subscribe("hardware/io_state", 2, &ManagerBTNode::io_state_cb, this);
  system_status_sub_ = nh_->subscribe("system_status", 10, &ManagerBTNode::system_status_cb, this);

  ros::Rate rate(10.0);  // 10Hz
  while (ros::ok() && !e_stop_state_.has_value()) {
    ROS_INFO_THROTTLE(5.0, "[%s] Waiting for e_stop message to arrive", node_name_.c_str());
    rate.sleep();
    ros::spinOnce();
  }

  while (ros::ok() && !io_state_.has_value()) {
    ROS_INFO_THROTTLE(5.0, "[%s] Waiting for io_state message to arrive", node_name_.c_str());
    rate.sleep();
    ros::spinOnce();
  }

  while (ros::ok() && !battery_status_.has_value()) {
    ROS_INFO_THROTTLE(5.0, "[%s] Waiting for battery message to arrive", node_name_.c_str());
    rate.sleep();
    ros::spinOnce();
  }

  lights_tree_timer_ =
    nh_->createTimer(ros::Duration(0.1), std::bind(&ManagerBTNode::lights_tree_timer_cb, this));
  safety_tree_timer_ =
    nh_->createTimer(ros::Duration(0.1), std::bind(&ManagerBTNode::safety_tree_timer_cb, this));

  ROS_INFO("[%s] Node started", node_name_.c_str());
}

BT::NodeConfig ManagerBTNode::create_bt_config(
  const std::map<std::string, std::any> bb_values) const
{
  BT::NodeConfig config;
  config.blackboard = BT::Blackboard::create();
  // update blackboard
  config.blackboard->set("nh", nh_);
  for (auto & item : bb_values) {
    const std::type_info & type = item.second.type();
    if (type == typeid(bool)) {
      config.blackboard->set<bool>(item.first, std::any_cast<bool>(item.second));
    } else if (type == typeid(int)) {
      config.blackboard->set<int>(item.first, std::any_cast<int>(item.second));
    } else if (type == typeid(unsigned)) {
      config.blackboard->set<unsigned>(item.first, std::any_cast<unsigned>(item.second));
    } else if (type == typeid(float)) {
      config.blackboard->set<float>(item.first, std::any_cast<float>(item.second));
    } else if (type == typeid(double)) {
      config.blackboard->set<double>(item.first, std::any_cast<double>(item.second));
    } else if (type == typeid(long)) {
      config.blackboard->set<long>(item.first, std::any_cast<long>(item.second));
    } else if (type == typeid(const char *)) {
      config.blackboard->set<std::string>(item.first, std::any_cast<const char *>(item.second));
    } else if (type == typeid(std::string)) {
      config.blackboard->set<std::string>(item.first, std::any_cast<std::string>(item.second));
    } else {
      throw std::invalid_argument(
        "Invalid type for blackboard entry. Valid types are: bool, int, unsigned, float, double,"
        " long, const char*, std::string");
    }
  }

  return config;
}

void ManagerBTNode::battery_cb(const sensor_msgs::BatteryState::ConstPtr & battery)
{
  battery_temp_ma_->roll(battery->temperature);
  battery_percent_ma_->roll(battery->percentage);
  battery_status_ = battery->power_supply_status;
}

void ManagerBTNode::driver_state_cb(const panther_msgs::DriverState::ConstPtr & driver_state)
{
  front_driver_temp_ma_->roll(driver_state->front.temperature);
  rear_driver_temp_ma_->roll(driver_state->rear.temperature);
}

void ManagerBTNode::e_stop_cb(const std_msgs::Bool::ConstPtr & e_stop)
{
  e_stop_state_ = e_stop->data;
}

void ManagerBTNode::io_state_cb(const panther_msgs::IOState::ConstPtr & io_state)
{
  if (io_state->power_button) {
    shutdown_robot("Power button pressed");
  }
  io_state_ = io_state;
}

void ManagerBTNode::system_status_cb(const panther_msgs::SystemStatus::ConstPtr & system_status)
{
  cpu_temp_ma_->roll(system_status->cpu_temp);
}

void ManagerBTNode::lights_tree_timer_cb()
{
  // update blackboard
  lights_config_.blackboard->set<bool>("e_stop_state", e_stop_state_.value());
  lights_config_.blackboard->set<unsigned>("battery_status", battery_status_.value());
  lights_config_.blackboard->set<float>("battery_percent", battery_percent_ma_->get_average());
  lights_config_.blackboard->set<std::string>(
    "battery_percent_round",
    std::to_string(
      round(battery_percent_ma_->get_average() / update_charging_anim_step_) *
      update_charging_anim_step_));

  // BT::StdCoutLogger logger_cout(lights_tree_);  // debugging
  lights_tree_status_ = lights_tree_.tickOnce();
}

void ManagerBTNode::safety_tree_timer_cb()
{
  // update blackboard
  safety_config_.blackboard->set<bool>("aux_state", io_state_.value()->aux_power);
  safety_config_.blackboard->set<bool>("e_stop_state", e_stop_state_.value());
  safety_config_.blackboard->set<bool>("fan_state", io_state_.value()->fan);
  safety_config_.blackboard->set<double>("bat_temp", battery_temp_ma_->get_average());
  safety_config_.blackboard->set<double>("cpu_temp", cpu_temp_ma_->get_average());
  // to simplify conditions pass only higher temp of motor drivers
  safety_config_.blackboard->set<double>(
    "driver_temp",
    std::max({front_driver_temp_ma_->get_average(), rear_driver_temp_ma_->get_average()}));

  // BT::StdCoutLogger logger_cout(safety_tree_);  // debugging
  safety_tree_status_ = safety_tree_.tickOnce();

  std::string signal_shutdown;
  if (safety_config_.blackboard->get<std::string>("signal_shutdown", signal_shutdown)) {
    shutdown_robot(signal_shutdown);
  }
}

void ManagerBTNode::shutdown_robot(const std::string & message)
{
  ROS_WARN("[%s] %s. Soft shutdown initialized.", node_name_.c_str(), message.c_str());
  lights_tree_timer_.stop();
  lights_tree_.haltTree();
  safety_tree_timer_.stop();
  safety_tree_.haltTree();

  // tick shutdown tree
  // BT::StdCoutLogger logger_cout(shutdown_tree_);  // debugging
  shutdown_tree_status_ = BT::NodeStatus::RUNNING;
  auto start_time = ros::Time::now();
  ros::Rate rate(10.0);  // 10 Hz
  while (ros::ok() && shutdown_tree_status_ == BT::NodeStatus::RUNNING) {
    auto shutdown_timeout = (ros::Time::now() - start_time) > ros::Duration(shutdown_timeout_);
    shutdown_config_.blackboard->set<bool>("shutdown_timeout", shutdown_timeout);
    shutdown_tree_status_ = shutdown_tree_.tickOnce();
    rate.sleep();
  }
  ros::requestShutdown();
}

}  // namespace panther_manager
