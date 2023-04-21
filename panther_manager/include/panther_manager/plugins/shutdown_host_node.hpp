#ifndef PANTHER_MANAGER_SHUTDOWN_HOST_NODE_HPP_
#define PANTHER_MANAGER_SHUTDOWN_HOST_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <libssh/libssh.h>

#include <ros/ros.h>

namespace panther_manager
{

class ShutdownHost : public BT::StatefulActionNode
{
public:
  explicit ShutdownHost(const std::string & name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
  {
    node_name_ = ros::this_node::getName();
  }

  virtual ~ShutdownHost() = default;

  ssh_session session_;
  ssh_channel channel_;

  bool check_ip(const std::string ip)
  {
    return system(("ping -c 1 -w 1 " + ip + " > /dev/null").c_str()) == 0;
  }

  int ssh_execute_command(const char * host, const char * user, const char * command)
  {
    ROS_INFO("[%s] Shutting down device at: %s", node_name_.c_str(), host);

    session_ = ssh_new();
    if (session_ == NULL) return -1;

    ssh_options_set(session_, SSH_OPTIONS_HOST, host);
    ssh_options_set(session_, SSH_OPTIONS_USER, user);
    ssh_options_set(session_, SSH_OPTIONS_PORT, &port_);
    ssh_options_set(session_, SSH_OPTIONS_LOG_VERBOSITY, &verbosity_);

    if (ssh_connect(session_) != SSH_OK) {
      ROS_ERROR("[%s] Error connecting to host: %s", node_name_.c_str(), ssh_get_error(session_));
      ssh_free(session_);
      return -1;
    }

    if (ssh_userauth_publickey_auto(session_, NULL, NULL) != SSH_AUTH_SUCCESS) {
      ROS_ERROR(
        "[%s] Error authenticating with public key: %s", node_name_.c_str(),
        ssh_get_error(session_));
      ssh_disconnect(session_);
      ssh_free(session_);
      return -1;
    }

    channel_ = ssh_channel_new(session_);
    if (channel_ == NULL) {
      ROS_ERROR(
        "[%s] Failed to create ssh channel: %s", node_name_.c_str(), ssh_get_error(session_));
      ssh_disconnect(session_);
      ssh_free(session_);
      return -1;
    }

    if (ssh_channel_open_session(channel_) != SSH_OK) {
      ROS_ERROR("[%s] Failed to open ssh channel: %s", node_name_.c_str(), ssh_get_error(session_));
      ssh_channel_free(channel_);
      ssh_disconnect(session_);
      ssh_free(session_);
      return -1;
    }

    if (ssh_channel_request_exec(channel_, command) != SSH_OK) {
      ROS_ERROR(
        "[%s] Failed to execute ssh command: %s", node_name_.c_str(), ssh_get_error(session_));
      ssh_channel_close(channel_);
      ssh_channel_free(channel_);
      ssh_disconnect(session_);
      ssh_free(session_);
      return -1;
    }

    return 0;
  }

  void close_connection(){
    ssh_channel_send_eof(channel_);
    ssh_channel_close(channel_);
    ssh_channel_free(channel_);
    ssh_disconnect(session_);
    ssh_free(session_);
  }

  std::string get_node_name() { return node_name_; }

private:
  const int verbosity_ = SSH_LOG_NOLOG;
  int port_ = 22;
  std::string node_name_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SHUTDOWN_HOST_NODE_HPP_
