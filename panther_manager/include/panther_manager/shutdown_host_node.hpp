#ifndef PANTHER_MANAGER_SHUTDOWN_HOST_NODE_HPP_
#define PANTHER_MANAGER_SHUTDOWN_HOST_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <libssh/libssh.h>

#include <ros/ros.h>

namespace panther_manager
{

class ShutdownHost : public BT::SyncActionNode
{
public:
  ShutdownHost(const std::string & name, const BT::NodeConfig & conf)
  : BT::SyncActionNode(name, conf)
  {
    node_name_ = ros::this_node::getName();
  }

  bool check_ip(const std::string ip)
  {
    return system(("ping -c 1 -w 1 " + ip + " > /dev/null").c_str()) == 0;
  }

  int ssh_execute_command(const char * host, const char * user, const char * command)
  {
    ROS_INFO("[%s] Shutting down device at: %s", node_name_.c_str(), host);

    ssh_session session;

    session = ssh_new();
    if (session == NULL) return -1;

    ssh_options_set(session, SSH_OPTIONS_HOST, host);
    ssh_options_set(session, SSH_OPTIONS_USER, user);
    ssh_options_set(session, SSH_OPTIONS_PORT, &port_);
    ssh_options_set(session, SSH_OPTIONS_LOG_VERBOSITY, &verbosity_);

    int rc = ssh_connect(session);
    if (rc != SSH_OK) {
      ROS_ERROR("[%s] Error connecting to host: %s", node_name_.c_str(), ssh_get_error(session));
      ssh_free(session);
      return -1;
    }

    rc = ssh_userauth_publickey_auto(session, NULL, NULL);
    if (rc != SSH_AUTH_SUCCESS) {
      ROS_ERROR(
        "[%s] Error authenticating with public key: %s", node_name_.c_str(),
        ssh_get_error(session));
      ssh_disconnect(session);
      ssh_free(session);
      return -1;
    }

    ssh_channel channel = ssh_channel_new(session);
    if (channel == NULL) {
      ROS_ERROR(
        "[%s] Failed to create ssh channel: %s", node_name_.c_str(), ssh_get_error(session));
      ssh_disconnect(session);
      ssh_free(session);
      return -1;
    }

    rc = ssh_channel_open_session(channel);
    if (rc != SSH_OK) {
      ROS_ERROR("[%s] Failed to open ssh channel: %s", node_name_.c_str(), ssh_get_error(session));
      ssh_channel_free(channel);
      ssh_disconnect(session);
      ssh_free(session);
      return -1;
    }

    rc = ssh_channel_request_exec(channel, command);
    if (rc != SSH_OK) {
      ROS_ERROR(
        "[%s] Failed to execute ssh command: %s", node_name_.c_str(), ssh_get_error(session));
      ssh_channel_close(channel);
      ssh_channel_free(channel);
      ssh_disconnect(session);
      ssh_free(session);
      return -1;
    }

    char buffer[1024];
    int nbytes;
    std::string output;
    while ((nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0)) > 0) {
      output.append(buffer, nbytes);
    }
    ROS_INFO("[%s] Device response:\n%s", node_name_.c_str(), output.c_str());

    // close and disconnect
    ssh_channel_send_eof(channel);
    ssh_channel_close(channel);
    ssh_channel_free(channel);
    ssh_disconnect(session);
    ssh_free(session);

    return 0;
  }

  std::string get_node_name() { return node_name_; }

private:
  int verbosity_ = SSH_LOG_NOLOG;
  int port_ = 22;
  std::string node_name_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SHUTDOWN_HOST_NODE_HPP_
