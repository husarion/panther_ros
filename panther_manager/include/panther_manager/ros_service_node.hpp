#ifndef PANTHER_MANAGER_ROS_SERVICE_NODE_HPP_
#define PANTHER_MANAGER_ROS_SERVICE_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <ros/ros.h>
#include <ros/service_client.h>

namespace panther_manager
{

template <class ServiceT>
class RosServiceNode : public BT::SyncActionNode
{
protected:
  RosServiceNode(
    std::shared_ptr<ros::NodeHandle> & nh, const std::string & name, const BT::NodeConfig & conf)
  : BT::SyncActionNode(name, conf), nh_(nh)
  {
    if (!getInput<std::string>("service_name", srv_name_) || srv_name_ == "") {
      throw BT::RuntimeError("[", name, "] Failed to get input [service_name]");
    }

    unsigned srv_timeout_ms;
    if (!getInput<unsigned>("timeout", srv_timeout_ms)) {
      throw BT::RuntimeError("[", name, "] Failed to get input [timeout]");
    }
    srv_timeout_ = ros::Duration(static_cast<double>(srv_timeout_ms) * 1e-3);

    node_name_ = ros::this_node::getName();
  }

public:
  using ServiceType = ServiceT;
  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("service_name", "ROS service name"),
      BT::InputPort<float>("timeout", 100, "time in ms to wait for service to be active"),
    };
  }

  // methods to be implemented by user
  virtual void update_request(RequestType & request) = 0;
  virtual BT::NodeStatus on_response(const ResponseType & response) = 0;

  std::string get_node_name() { return node_name_; }

private:
  std::string node_name_;
  std::string srv_name_;

  ros::Duration srv_timeout_;
  ros::ServiceClient srv_client_;
  std::shared_ptr<ros::NodeHandle> & nh_;

  BT::NodeStatus tick() override
  {
    if (!srv_client_.isValid()) {
      srv_client_ = nh_->serviceClient<ServiceT>(srv_name_);
    }

    if (!srv_client_.waitForExistence(srv_timeout_)) {
      ROS_ERROR("[%s] Timeout waiting for service %s", node_name_.c_str(), srv_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RequestType request;
    ResponseType response;
    update_request(request);
    if (!srv_client_.call(request, response)) {
      ROS_ERROR("[%s] Failed to call service %s", node_name_.c_str(), srv_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return on_response(response);
  }
};

template <class ServiceNodeT>
static void RegisterRosService(
  BT::BehaviorTreeFactory & factory, const std::string & registration_ID)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfig & config) {
    return std::make_unique<ServiceNodeT>(name, config);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<ServiceNodeT>();
  manifest.ports = ServiceNodeT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto & basic_ports = RosServiceNode<typename ServiceNodeT::ServiceType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());

  factory.registerBuilder(manifest, builder);
}

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_ROS_SERVICE_NODE_HPP_