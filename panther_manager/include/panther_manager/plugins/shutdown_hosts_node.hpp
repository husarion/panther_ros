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
  virtual void update_hosts(std::vector<std::shared_ptr<ShutdownHost>> & hosts) = 0;

  // method that can be overriden by user
  virtual BT::NodeStatus post_process()
  {
    // return success only when all hosts succeded
    if (failed_hosts_.size() == 0) return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
  }

  std::string get_node_name() const { return node_name_; }
  std::vector<std::size_t> const get_failed_hosts() { return failed_hosts_; }

private:
  int check_host_index_ = 0;
  std::string node_name_;
  std::vector<std::shared_ptr<ShutdownHost>> hosts_;
  std::vector<std::size_t> hosts_to_check_;
  std::vector<std::size_t> skipped_hosts_;
  std::vector<std::size_t> succeeded_hosts_;
  std::vector<std::size_t> failed_hosts_;

  BT::NodeStatus onStart()
  {
    update_hosts(hosts_);
    remove_duplicate_hosts(hosts_);
    if (hosts_.size() <= 0) {
      ROS_ERROR("[%s] Hosts list is empty! Check configuration!", node_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    for (std::size_t i = 0; i < hosts_.size(); i++) {
      hosts_to_check_.push_back(i);
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning()
  {
    if (hosts_to_check_.size() <= 0) return post_process();

    if (check_host_index_ >= hosts_to_check_.size()) check_host_index_ = 0;

    auto host_index = hosts_to_check_.at(check_host_index_);
    auto host = hosts_[host_index];
    host->call();

    switch (host->get_state()) {
      case ShutdownHostState::RESPONSE_RECEIVED:
        ROS_INFO(
          "[%s] Device at: %s response:\n%s", node_name_.c_str(), host->get_ip().c_str(),
          host->get_response().c_str());
        check_host_index_++;
        break;

      case ShutdownHostState::SUCCESS:
        ROS_INFO(
          "[%s] Successfuly shutdown device at: %s", node_name_.c_str(), host->get_ip().c_str());
        succeeded_hosts_.push_back(host_index);
        hosts_to_check_.erase(hosts_to_check_.begin() + check_host_index_);
        break;

      case ShutdownHostState::FAILURE:
        ROS_WARN(
          "[%s] Failed to shutdown device at: %s\n%s", node_name_.c_str(), host->get_ip().c_str(),
          host->get_error().c_str());
        failed_hosts_.push_back(host_index);
        hosts_to_check_.erase(hosts_to_check_.begin() + check_host_index_);
        break;

      case ShutdownHostState::SKIPPED:
        ROS_WARN(
          "[%s] Davice at: %s not available, skipping", node_name_.c_str(), host->get_ip().c_str());
        skipped_hosts_.push_back(host_index);
        hosts_to_check_.erase(hosts_to_check_.begin() + check_host_index_);
        break;

      default:
        check_host_index_++;
        break;
    }

    return BT::NodeStatus::RUNNING;
  }

  void remove_duplicate_hosts(std::vector<std::shared_ptr<ShutdownHost>> & hosts)
  {
    std::set<ShutdownHost> seen;
    std::vector<std::shared_ptr<ShutdownHost>> result;

    for (const auto & host : hosts) {
      if (!seen.count(*host)) {
        seen.insert(*host);
        result.push_back(host);
      } else {
        ROS_WARN(
          "Found duplicate host: %s\nProcessing only the first occurence.", host->get_ip().c_str());
      }
    }

    hosts = std::move(result);
  }

  void onHalted()
  {
    for (auto & host : hosts_) {
      host->close_connection();
    }
  }
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SHUTDOWN_HOSTS_NODE_HPP_
