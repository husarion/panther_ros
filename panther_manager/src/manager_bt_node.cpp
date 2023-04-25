#include <panther_manager/manager_bt_node.hpp>

namespace panther_manager
{

ManagerNode::ManagerNode(
  const std::shared_ptr<ros::NodeHandle> nh, const std::shared_ptr<ros::NodeHandle> ph)
: nh_(std::move(nh)), ph_(std::move(ph))
{
  node_name_ = ros::this_node::getName();

  const std::string default_xml =
    ros::package::getPath("panther_manager") + "/config/PantherManagerBT.xml";
  const std::vector<std::string> default_plugin_libs = {};

  auto xml_filename = ph_->param<std::string>("xml_filename", default_xml);
  auto plugin_libs = ph_->param<std::vector<std::string>>("plugin_libs", default_plugin_libs);
  auto battery_temp_window_len = ph_->param<int>("battery_temp_window_len", 6);
  auto cpu_temp_window_len = ph_->param<int>("cpu_temp_window_len", 6);
  auto driver_temp_window_len = ph_->param<int>("driver_temp_window_len", 6);
  auto shutdown_hosts_file = ph_->param<std::string>("shutdown_hosts_file", "");
  shutdown_timeout_ = ph_->param<float>("shutdown_timeout", 15.0);
  debug_tree_ = ph_->param<std::string>("debug_tree", "");

  // lights tree params
  auto critical_battery_anim_period =
    ph_->param<float>("lights/critical_battery_anim_period", 15.0);
  auto critical_battery_threshold_percent =
    ph_->param<float>("lights/critical_battery_threshold_percent", 0.1);
  auto battery_state_anim_period = ph_->param<float>("lights/battery_state_anim_period", 120.0);
  auto low_battery_anim_period = ph_->param<float>("lights/low_battery_anim_period", 30.0);
  auto low_battery_threshold_percent =
    ph_->param<float>("lights/low_battery_threshold_percent", 0.4);
  update_charging_anim_step_ = ph_->param<float>("lights/update_charging_anim_step", 0.1);

  // safety tree params
  auto high_bat_temp = ph_->param<float>("safety/high_bat_temp", 55.0);
  auto critical_bat_temp = ph_->param<float>("safety/critical_bat_temp", 59.0);
  auto fatal_bat_temp = ph_->param<float>("safety/fatal_bat_temp", 62.0);
  auto cpu_fan_on_temp = ph_->param<float>("safety/cpu_fan_on_temp", 70.0);
  auto cpu_fan_off_temp = ph_->param<float>("safety/cpu_fan_off_temp", 60.0);
  auto driver_fan_on_temp = ph_->param<float>("safety/driver_fan_on_temp", 45.0);
  auto driver_fan_off_temp = ph_->param<float>("safety/driver_fan_off_temp", 35.0);

  battery_temp_ma_ = MovingAverage<double>(battery_temp_window_len);
  cpu_temp_ma_ = MovingAverage<double>(cpu_temp_window_len);
  front_driver_temp_ma_ = MovingAverage<double>(driver_temp_window_len);
  rear_driver_temp_ma_ = MovingAverage<double>(driver_temp_window_len);

  ROS_INFO("[%s] Register BehaviorTree from: %s", node_name_.c_str(), xml_filename.c_str());

  // export plugins for a behaviour tree
  for (const auto & p : plugin_libs) {
    factory_.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  factory_.registerBehaviorTreeFromFile(xml_filename);

  const std::map<std::string, std::any> lights_initial_bb = {
    {"critical_battery_anim_period", critical_battery_anim_period},
    {"critical_battery_threshold_percent", critical_battery_threshold_percent},
    {"battery_state_anim_period", battery_state_anim_period},
    {"low_battery_anim_period", low_battery_anim_period},
    {"low_battery_threshold_percent", low_battery_threshold_percent},
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
    {"high_bat_temp", high_bat_temp},
    {"critical_bat_temp", critical_bat_temp},
    {"fatal_bat_temp", fatal_bat_temp},
    {"cpu_fan_on_temp", cpu_fan_on_temp},
    {"cpu_fan_off_temp", cpu_fan_off_temp},
    {"driver_fan_on_temp", driver_fan_on_temp},
    {"driver_fan_off_temp", driver_fan_off_temp},
  };
  const std::map<std::string, std::any> shutdown_initial_bb = {
    {"shutdown_hosts_file", shutdown_hosts_file.c_str()},
  };

  lights_config_ = create_bt_config(lights_initial_bb);
  safety_config_ = create_bt_config(safety_initial_bb);
  shutdown_config_ = create_bt_config(shutdown_initial_bb);

  lights_tree_ = factory_.createTree("Lights", lights_config_.blackboard);
  safety_tree_ = factory_.createTree("Safety", safety_config_.blackboard);
  shutdown_tree_ = factory_.createTree("Shutdown", shutdown_config_.blackboard);

  battery_sub_ = nh_->subscribe("battery", 10, &ManagerNode::battery_cb, this);
  driver_state_sub_ =
    nh_->subscribe("driver/motor_controllers_state", 10, &ManagerNode::driver_state_cb, this);
  e_stop_sub_ = nh_->subscribe("hardware/e_stop", 2, &ManagerNode::e_stop_cb, this);
  io_state_sub_ = nh_->subscribe("hardware/io_state", 2, &ManagerNode::io_state_cb, this);
  system_status_sub_ = nh_->subscribe("system_status", 10, &ManagerNode::system_status_cb, this);

  while (ros::ok() && !e_stop_state_.has_value()) {
    ROS_INFO_THROTTLE(5.0, "[%s] Waiting for e_stop message to arrive", node_name_.c_str());
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  while (ros::ok() && !io_state_.has_value()) {
    ROS_INFO_THROTTLE(5.0, "[%s] Waiting for io_state message to arrive", node_name_.c_str());
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  while (ros::ok() && !battery_status_.has_value()) {
    ROS_INFO_THROTTLE(5.0, "[%s] Waiting for battery message to arrive", node_name_.c_str());
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  lights_tree_timer_ =
    nh_->createTimer(ros::Duration(0.1), std::bind(&ManagerNode::lights_tree_timer_cb, this));
  safety_tree_timer_ =
    nh_->createTimer(ros::Duration(0.1), std::bind(&ManagerNode::safety_tree_timer_cb, this));

  ROS_INFO("[%s] Node started", node_name_.c_str());
}

BT::NodeConfig ManagerNode::create_bt_config(const std::map<std::string, std::any> bb_values)
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

void ManagerNode::battery_cb(const sensor_msgs::BatteryState::ConstPtr & battery)
{
  battery_temp_ma_.roll(battery->temperature);
  battery_status_ = battery->power_supply_status;
  battery_percent_ = battery->percentage;
}

void ManagerNode::driver_state_cb(const panther_msgs::DriverState::ConstPtr & driver_state)
{
  front_driver_temp_ma_.roll(driver_state->front.temperature);
  rear_driver_temp_ma_.roll(driver_state->rear.temperature);
}

void ManagerNode::e_stop_cb(const std_msgs::Bool::ConstPtr & e_stop)
{
  e_stop_state_ = e_stop->data;
}

void ManagerNode::io_state_cb(const panther_msgs::IOState::ConstPtr & io_state)
{
  if (io_state->power_button) {
    shutdown_robot("Power button pressed");
  }
  io_state_ = io_state;
}

void ManagerNode::system_status_cb(const panther_msgs::SystemStatus::ConstPtr & system_status)
{
  cpu_temp_ma_.roll(system_status->cpu_temp);
}

void ManagerNode::lights_tree_timer_cb()
{
  // update blackboard
  lights_config_.blackboard->set<bool>("e_stop_state", e_stop_state_.value());
  lights_config_.blackboard->set<unsigned>("battery_status", battery_status_.value());
  lights_config_.blackboard->set<float>("battery_percent", battery_percent_);
  lights_config_.blackboard->set<std::string>(
    "battery_percent_round",
    std::to_string(
      round(battery_percent_ / update_charging_anim_step_) * update_charging_anim_step_));

  // BT::StdCoutLogger logger_cout(lights_tree_);  // debugging
  lights_tree_status_ = lights_tree_.tickOnce();
}

void ManagerNode::safety_tree_timer_cb()
{
  // update blackboard
  safety_config_.blackboard->set<bool>("aux_state", io_state_.value()->aux_power);
  safety_config_.blackboard->set<bool>("e_stop_state", e_stop_state_.value());
  safety_config_.blackboard->set<bool>("fan_state", io_state_.value()->fan);
  safety_config_.blackboard->set<double>("bat_temp", battery_temp_ma_.get_average());
  safety_config_.blackboard->set<double>("cpu_temp", cpu_temp_ma_.get_average());
  // to simplify conditions pass only higher temp of motor drivers
  safety_config_.blackboard->set<double>(
    "driver_temp",
    std::max({front_driver_temp_ma_.get_average(), rear_driver_temp_ma_.get_average()}));

  // BT::StdCoutLogger logger_cout(safety_tree_);  // debugging
  safety_tree_status_ = safety_tree_.tickOnce();

  std::string signal_shutdown;
  if (safety_config_.blackboard->get<std::string>("signal_shutdown", signal_shutdown)) {
    shutdown_robot(signal_shutdown);
  }
}

void ManagerNode::shutdown_robot(const std::string & message)
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
  while (ros::ok() && shutdown_tree_status_ == BT::NodeStatus::RUNNING) {
    auto shutdown_timeout = (ros::Time::now() - start_time) > ros::Duration(shutdown_timeout_);
    shutdown_config_.blackboard->set<bool>("shutdown_timeout", shutdown_timeout);
    shutdown_tree_status_ = shutdown_tree_.tickOnce();
  }
  ros::requestShutdown();
}

}  // namespace panther_manager
