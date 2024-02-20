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

#ifndef PANTHER_MANAGER_SHUTDOWN_HOSTS_NODE_HPP_
#define PANTHER_MANAGER_SHUTDOWN_HOSTS_NODE_HPP_

#include <algorithm>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <rclcpp/rclcpp.hpp>

#include <panther_manager/plugins/shutdown_host.hpp>

namespace panther_manager
{

class ShutdownHosts : public BT::StatefulActionNode
{
public:
  explicit ShutdownHosts(const std::string& name, const BT::NodeConfig& conf) : BT::StatefulActionNode(name, conf)
  {
    // TODO: @delihus What is the name of ros::this_node?
    node_name_ = name;
    logger_ = std::make_shared<rclcpp::Logger>({ rclcpp::get_logger("node_name_") });
  }

  virtual ~ShutdownHosts() = default;

  // method to be implemented by user
  virtual void update_hosts(std::vector<std::shared_ptr<ShutdownHost>>& hosts) = 0;

  // method that can be overriden by user
  virtual BT::NodeStatus post_process()
  {
    // return success only when all hosts succeeded
    if (failed_hosts_.size() == 0)
    {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  std::string get_node_name() const
  {
    return node_name_;
  }
  std::vector<std::size_t> const get_failed_hosts()
  {
    return failed_hosts_;
  }

private:
  int check_host_index_ = 0;
  std::string node_name_;
  std::vector<std::shared_ptr<ShutdownHost>> hosts_;
  std::vector<std::size_t> hosts_to_check_;
  std::vector<std::size_t> skipped_hosts_;
  std::vector<std::size_t> succeeded_hosts_;
  std::vector<std::size_t> failed_hosts_;
  rclcpp::Logger::SharedPtr logger_;

  BT::NodeStatus onStart()
  {
    update_hosts(hosts_);
    remove_duplicate_hosts(hosts_);
    if (hosts_.size() <= 0)
    {
      RCLCPP_ERROR_STREAM(&logger_, "Hosts list is empty! Check configuration!");
      return BT::NodeStatus::FAILURE;
    }
    hosts_to_check_.resize(hosts_.size());
    std::iota(hosts_to_check_.begin(), hosts_to_check_.end(), 0);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning()
  {
    if (hosts_to_check_.size() <= 0)
    {
      return post_process();
    }

    if (check_host_index_ >= hosts_to_check_.size())
    {
      check_host_index_ = 0;
    }

    auto host_index = hosts_to_check_[check_host_index_];
    auto host = hosts_[host_index];
    host->call();

    switch (host->get_state())
    {
      case ShutdownHostState::RESPONSE_RECEIVED:
        RCLCPP_INFO_STREAM(&logger_, "Device at: " << host->get_ip() << " response:\n" << host->get_response());

        check_host_index_++;
        break;

      case ShutdownHostState::SUCCESS:
        RCLCPP_INFO_STREAM(&logger_, "Successfully shutdown device at: " << host->get_ip());
        succeeded_hosts_.push_back(host_index);
        hosts_to_check_.erase(hosts_to_check_.begin() + check_host_index_);
        break;

      case ShutdownHostState::FAILURE:
        ROS_WARN("[%s] Failed to shutdown device at: %s. Error: %s", node_name_.c_str(), host->get_ip().c_str(),
                 host->get_error().c_str());
        failed_hosts_.push_back(host_index);
        hosts_to_check_.erase(hosts_to_check_.begin() + check_host_index_);
        break;

      case ShutdownHostState::SKIPPED:
        RCLCPP_WARN_STREAM(&logger_, "Device at: " << host->get_ip() << " not available, skipping...");

        skipped_hosts_.push_back(host_index);
        hosts_to_check_.erase(hosts_to_check_.begin() + check_host_index_);
        break;

      default:
        check_host_index_++;
        break;
    }

    return BT::NodeStatus::RUNNING;
  }

  void remove_duplicate_hosts(std::vector<std::shared_ptr<ShutdownHost>>& hosts)
  {
    std::set<ShutdownHost> seen;

    hosts.erase(std::remove_if(hosts.begin(), hosts.end(),
                               [&](const std::shared_ptr<ShutdownHost>& host) {
                                 if (!seen.count(*host))
                                 {
                                   seen.insert(*host);
                                   return false;
                                 }
                                 else
                                 {
                                   RCLCPP_WARN_STREAM(&logger_, "Found duplicate host: " << host->get_ip()
                                                                                         << " Processing only the "
                                                                                            "first "
                                                                                            "occurrence.");
                                   return true;
                                 }
                               }),
                hosts.end());
  }

  void onHalted()
  {
    for (auto& host : hosts_)
    {
      host->close_connection();
    }
  }
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SHUTDOWN_HOSTS_NODE_HPP_
