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
#include <panther_manager/types.hpp>

namespace panther_manager
{

RobotStatesManagerNode::RobotStatesManagerNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  DeclareParameters();

  const auto initial_blackboard = CreateBlackboard();
  docking_tree_manager_ = std::make_unique<BehaviorTreeManager>(
    "RobotStates", initial_blackboard, 5555);

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void RobotStatesManagerNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  RegisterBehaviorTree();
  docking_tree_manager_->Initialize(factory_);

  using namespace std::placeholders;

  e_stop_sub_ = this->create_subscription<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&RobotStatesManagerNode::EStopCB, this, _1));
  robot_state_pub_ = this->create_publisher<RobotStateMsg>(
    "robot_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  const float timer_freq = this->get_parameter("timer_frequency").as_double();
  const auto timer_period_ms =
    std::chrono::milliseconds(static_cast<unsigned>(1.0f / timer_freq * 1000));

  docking_tree_timer_ = this->create_wall_timer(
    timer_period_ms, std::bind(&RobotStatesManagerNode::RobotStatesTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void RobotStatesManagerNode::DeclareParameters()
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

  this->declare_parameter<float>("timer_frequency", 20.0);
}

void RobotStatesManagerNode::RegisterBehaviorTree()
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

std::map<std::string, std::any> RobotStatesManagerNode::CreateBlackboard()
{
  const std::map<std::string, std::any> docking_initial_bb = {
    // docking states
    {"docking_cmd", unsigned(DockingCmd::DOCKING_CMD_NONE)},
    {"DOCKING_CMD_NONE", unsigned(DockingCmd::DOCKING_CMD_NONE)},
    {"DOCKING_CMD_DOCK", unsigned(DockingCmd::DOCKING_CMD_DOCK)},
    {"DOCKING_CMD_UNDOCK", unsigned(DockingCmd::DOCKING_CMD_UNDOCK)},
    // robot states
    {"robot_state", int(RobotStateMsg::E_STOP)},
    {"ROBOT_STATE_ERROR", int(RobotStateMsg::ERROR)},
    {"ROBOT_STATE_ESTOP", int(RobotStateMsg::E_STOP)},
    {"ROBOT_STATE_STANDBY", int(RobotStateMsg::STANDBY)},
    {"ROBOT_STATE_DOCKING", int(RobotStateMsg::DOCKING)},
    {"ROBOT_STATE_SUCCESS", int(RobotStateMsg::SUCCESS)},
  };

  RCLCPP_INFO(this->get_logger(), "Blackboard created.");
  return docking_initial_bb;
}

void RobotStatesManagerNode::EStopCB(const BoolMsg::SharedPtr e_stop)
{
  docking_tree_manager_->GetBlackboard()->set<bool>("e_stop_state", e_stop->data);
}

void RobotStatesManagerNode::RobotStatesTimerCB()
{
  if (!SystemReady()) {
    return;
  }

  docking_tree_manager_->TickOnce();
  PublishRobotStateMsg();

  if (docking_tree_manager_->GetTreeStatus() == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(this->get_logger(), "Docking behavior tree returned FAILURE status");
  }
}

void RobotStatesManagerNode::PublishRobotStateMsg()
{
  int8_t state_id;
  if (docking_tree_manager_->GetBlackboard()->get("robot_state", state_id)) {
    auto msg = CreateRobotStateMsg(state_id);
    robot_state_pub_->publish(msg);
  }
}

bool RobotStatesManagerNode::SystemReady()
{
  if (!docking_tree_manager_->GetBlackboard()->getEntry("e_stop_state")) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for required system messages to arrive.");
    return false;
  }

  return true;
}

RobotStateMsg RobotStatesManagerNode::CreateRobotStateMsg(int8_t state_id)
{
  RobotStateMsg msg;

  if (state_id < 0) {
    msg.state_id = RobotStateMsg::ERROR;
    msg.state_name = "ERROR";
    return msg;
  }

  msg.state_id = state_id;
  switch (state_id) {
    case RobotStateMsg::E_STOP:
      msg.state_name = "E_STOP";
      break;
    case RobotStateMsg::STANDBY:
      msg.state_name = "STANDBY";
      break;
    case RobotStateMsg::DOCKING:
      msg.state_name = "DOCKING";
      break;
    case RobotStateMsg::SUCCESS:
      msg.state_name = "SUCCESS";
      break;
    default:
      throw std::runtime_error("Invalid state_id");
  }

  return msg;
}

}  // namespace panther_manager
