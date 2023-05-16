#ifndef PANTHER_MANAGER_SHUTDOWN_HOSTS_NODE_HPP_
#define PANTHER_MANAGER_SHUTDOWN_HOSTS_NODE_HPP_

#include <algorithm>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <ros/ros.h>

#include <panther_manager/plugins/shutdown_host.hpp>

namespace panther_manager
{

class ShutdownHosts : public BT::StatefulActionNode
{
public:
  explicit ShutdownHosts(const std::string & name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
  {
    node_name_ = ros::this_node::getName();
  }

  virtual ~ShutdownHosts() = default;

  // method to be implemented by user
  virtual void update_hosts(std::vector<ShutdownHost> & hosts) = 0;

  // method that can be overriden by user
  virtual BT::NodeStatus on_end()
  {
    // return success only when all hosts succeded
    if (failed_hosts_.size() == 0) return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
  }

  std::string get_node_name() const { return node_name_; }
  std::vector<std::shared_ptr<ShutdownHost>> const get_failed_hosts() { return failed_hosts_; }

private:
  int host_index_ = 0;
  std::string node_name_;
  std::vector<ShutdownHost> hosts_;
  std::vector<std::shared_ptr<ShutdownHost>> hosts_to_check_;
  std::vector<std::shared_ptr<ShutdownHost>> failed_hosts_;

  BT::NodeStatus onStart()
  {
    update_hosts(hosts_);
    remove_duplicate_hosts(hosts_);
    if (hosts_.size() <= 0) {
      ROS_ERROR("[%s] Hosts list is empty! Check configuration!", node_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    for (auto & host : hosts_) {
      hosts_to_check_.push_back(std::make_shared<ShutdownHost>(host));
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning()
  {
    if (hosts_to_check_.size() <= 0) return on_end();

    if (host_index_ >= hosts_to_check_.size()) host_index_ = 0;

    auto host = hosts_to_check_.at(host_index_);
    host->call();

    switch (host->get_state()) {
      case ShutdownHostState::RESPONSE_RECEIVED:

        ROS_INFO(
          "[%s] Device at: %s response:\n%s", node_name_.c_str(), host->get_ip().c_str(),
          host->get_response().c_str());
        host_index_++;
        break;

      case ShutdownHostState::SUCCESS:
        ROS_INFO(
          "[%s] Successfuly shutdown device at: %s", node_name_.c_str(), host->get_ip().c_str());
        hosts_to_check_.erase(hosts_to_check_.begin() + host_index_);
        break;

      case ShutdownHostState::FAILURE:
        ROS_WARN(
          "[%s] Failed to shutdown device at: %s\n%s", node_name_.c_str(), host->get_ip().c_str(),
          host->get_error().c_str());
        failed_hosts_.push_back(host);
        hosts_to_check_.erase(hosts_to_check_.begin() + host_index_);
        break;

      case ShutdownHostState::SKIPPED:
        ROS_WARN(
          "[%s] Davice at: %s not available, skipping", node_name_.c_str(), host->get_ip().c_str());
        hosts_to_check_.erase(hosts_to_check_.begin() + host_index_);
        break;

      default:
        host_index_++;
        break;
    }

    return BT::NodeStatus::RUNNING;
  }

  void remove_duplicate_hosts(std::vector<ShutdownHost> & hosts)
  {
    std::set<ShutdownHost> seen;
    std::vector<ShutdownHost> result;

    for (const auto & host : hosts) {
      if (!seen.count(host)) {
        seen.insert(host);
        result.push_back(host);
      } else {
        ROS_WARN(
          "Found duplicate host: %s\nProcessing only the first occurence.", host.get_ip().c_str());
      }
    }

    hosts = std::move(result);
  }

  void onHalted()
  {
    for (auto & host : hosts_) {
      host.close_connection();
    }
  }
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SHUTDOWN_HOSTS_NODE_HPP_
