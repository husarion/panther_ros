#ifndef PANTHER_MANAGER_CALL_SET_BOOL_SERVICE_NODE_HPP_
#define PANTHER_MANAGER_CALL_SET_BOOL_SERVICE_NODE_HPP_

#include <std_srvs/SetBool.h>

#include <panther_manager/ros_service_node.hpp>

namespace panther_manager
{

class CallSetBoolService : public RosServiceNode<std_srvs::SetBool>
{
public:
  CallSetBoolService(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("data", "true / false value")};
  }

  std::string srv_name_;
  std::shared_ptr<ros::NodeHandle> nh_;

  void update_request(RequestType & request) override;
  virtual BT::NodeStatus on_response(const ResponseType & response);
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_CALL_SET_BOOL_SERVICE_NODE_HPP_