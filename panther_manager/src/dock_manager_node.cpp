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

#include "panther_manager/dock_manager_node.hpp"

#include <algorithm>
#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"


#include <panther_manager/behavior_tree_manager.hpp>
#include <panther_manager/behavior_tree_utils.hpp>

namespace panther_manager
{

DockManagerNode::DockManagerNode(const std::string& node_name, const rclcpp::NodeOptions& options)
  : Node(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  DeclareParameters();

  const auto dock_initial_blackboard = CreateDockInitialBlackboard();
  dock_tree_manager_ = std::make_unique<BehaviorTreeManager>("Dock", dock_initial_blackboard, 7777);

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void DockManagerNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  RegisterBehaviorTree();
  dock_tree_manager_->Initialize(factory_);



  using namespace std::placeholders;

  const float timer_freq = this->get_parameter("timer_frequency").as_double();
  const auto timer_period_ms = std::chrono::milliseconds(static_cast<unsigned>(1.0f / timer_freq * 1000));

  dock_tree_timer_ = this->create_wall_timer(timer_period_ms, std::bind(&DockManagerNode::DockTreeTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void DockManagerNode::DeclareParameters()
{
  const auto panther_manager_pkg_path = ament_index_cpp::get_package_share_directory("panther_manager");
  const std::string default_bt_project_path = panther_manager_pkg_path + "/behavior_trees/PantherDockBT.btproj";
  const std::vector<std::string> default_plugin_libs = {};

  this->declare_parameter<std::string>("bt_project_path", default_bt_project_path);
  this->declare_parameter<std::vector<std::string>>("plugin_libs", default_plugin_libs);
  this->declare_parameter<std::vector<std::string>>("ros_plugin_libs", default_plugin_libs);
  this->declare_parameter<double>("ros_communication_timeout.availability", 1.0);
  this->declare_parameter<double>("ros_communication_timeout.response", 1.0);
  this->declare_parameter<float>("timer_frequency", 1.0);
}

void DockManagerNode::RegisterBehaviorTree()
{
  const auto bt_project_path = this->get_parameter("bt_project_path").as_string();

  const auto plugin_libs = this->get_parameter("plugin_libs").as_string_array();
  const auto ros_plugin_libs = this->get_parameter("ros_plugin_libs").as_string_array();

  const auto service_availability_timeout = this->get_parameter("ros_communication_timeout.availability").as_double();
  const auto service_response_timeout = this->get_parameter("ros_communication_timeout.response").as_double();

  BT::RosNodeParams params;
  params.nh = this->shared_from_this();
  params.wait_for_server_timeout = std::chrono::milliseconds(static_cast<int>(service_availability_timeout * 1000));
  params.server_timeout = std::chrono::milliseconds(static_cast<int>(service_response_timeout * 1000));

  behavior_tree_utils::RegisterBehaviorTree(factory_, bt_project_path, plugin_libs, params, ros_plugin_libs);

  RCLCPP_INFO(this->get_logger(), "BehaviorTree registered from path '%s'", bt_project_path.c_str());
}

std::map<std::string, std::any> DockManagerNode::CreateDockInitialBlackboard()
{
  const std::map<std::string, std::any> dock_initial_bb = {};

  RCLCPP_INFO(this->get_logger(), "Blackboard created.");
  return dock_initial_bb;
}


bool DockManagerNode::SystemReady()
{
  return true;
}

void DockManagerNode::DockTreeTimerCB()
{
  if (!SystemReady()) {
    return;
  }

  dock_tree_manager_->TickOnce();

  if (dock_tree_manager_->GetTreeStatus() == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(this->get_logger(), "Dock behavior tree returned FAILURE status");
  }
}

}  // namespace panther_manager
