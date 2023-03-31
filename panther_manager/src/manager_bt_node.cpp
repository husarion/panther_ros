#include <panther_manager/manager_bt_node.hpp>

namespace panther_manager
{

ManagerNode::ManagerNode(std::shared_ptr<ros::NodeHandle> nh) : nh_(std::move(nh))
{
  node_name_ = ros::this_node::getName();

  const std::string default_xml =
    ros::package::getPath("panther_manager") + "/config/PantherManagerBT.xml";
  const std::vector<std::string> default_plugin_libs = {};

  xml_filename_ = nh_->param<std::string>("xml_filename", default_xml);
  plugin_libs_ = nh_->param<std::vector<std::string>>("plugin_libs", default_plugin_libs);

  ROS_INFO("[%s] Register BehaviorTree from: %s", node_name_.c_str(), xml_filename_.c_str());

  for (const auto & p : plugin_libs_) {
    factory_.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  factory_.registerBehaviorTreeFromFile(xml_filename_);

  setup_behavior_tree(lights_tree_, lights_config_, "Lights");
  setup_behavior_tree(safety_tree_, safety_config_, "Safety");
  setup_behavior_tree(shutdown_tree_, shutdown_config_, "Shutdown");

  e_stop_sub_ = nh_->subscribe("hardware/e_stop", 2, &ManagerNode::e_stop_cb, this);
  io_state_sub_ = nh_->subscribe("hardware/io_state", 2, &ManagerNode::io_state_cb, this);

  while (ros::ok() && !e_stop_state_.has_value()) {
    ROS_INFO("[%s] Waiting for e_stop message to arrive", node_name_.c_str());
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  while (ros::ok() && !io_state_.has_value()) {
    ROS_INFO("[%s] Waiting for io_state message to arrive", node_name_.c_str());
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  lights_tree_timer_ =
    nh_->createTimer(ros::Duration(0.1), std::bind(&ManagerNode::lights_tree_timer_cb, this));
  safety_tree_timer_ =
    nh_->createTimer(ros::Duration(0.1), std::bind(&ManagerNode::safety_tree_timer_cb, this));

  ROS_INFO("[%s] Node started", node_name_.c_str());
}

void ManagerNode::e_stop_cb(const std_msgs::Bool::ConstPtr & e_stop)
{
  e_stop_state_ = e_stop->data;
}

void ManagerNode::io_state_cb(const panther_msgs::IOState::ConstPtr & io_state)
{
  if (io_state->power_button) {
    ROS_WARN("[%s] Power button pressed. Shutting down robot.", node_name_.c_str());
    lights_tree_timer_.stop();
    lights_tree_.haltTree();
    safety_tree_timer_.stop();
    safety_tree_.haltTree();

    // tick shutdown tree
    // BT::StdCoutLogger logger_cout(shutdown_tree_);  // debuging
    shutdown_tree_status_ = BT::NodeStatus::RUNNING;
    while (ros::ok() && shutdown_tree_status_ == BT::NodeStatus::RUNNING) {
      shutdown_tree_status_ = shutdown_tree_.tickOnce();
    }
  }
  io_state_ = io_state;
}

void ManagerNode::lights_tree_timer_cb()
{
  // update blackboard
  lights_config_.blackboard->set<bool>("e_stop_state", e_stop_state_.value());
  lights_config_.blackboard->set<bool>("charging_state", io_state_.value()->charger_connected);

  // BT::StdCoutLogger logger_cout(lights_tree_);  // debuging
  lights_tree_status_ = lights_tree_.tickOnce();
}

void ManagerNode::safety_tree_timer_cb()
{
  // update blackboard
  safety_config_.blackboard->set<bool>("fan_state", io_state_.value()->fan);
  safety_config_.blackboard->set<bool>("aux_state", io_state_.value()->aux_power);

  // BT::StdCoutLogger logger_cout(safety_tree_);  // debuging
  safety_tree_status_ = safety_tree_.tickOnce();
}

void ManagerNode::setup_behavior_tree(
  BT::Tree & tree, BT::NodeConfig & config, const std::string tree_name)
{
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("nh", nh_);
  tree = factory_.createTree(tree_name, config.blackboard);
}

}  // namespace panther_manager
