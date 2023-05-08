#ifndef PANTHER_MANAGER_SHUTDOWN_HOST_HPP_
#define PANTHER_MANAGER_SHUTDOWN_HOST_HPP_

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <libssh/libssh.h>

#include <ros/time.h>

#include <panther_manager/plugins/host.hpp>

namespace panther_manager
{

class ShutdownHost : public Host
{
public:
  // default constructor
  ShutdownHost() : Host("", ""), command_(""), timeout_(5.0), ping_for_success_(true) {}
  ShutdownHost(
    const std::string ip, const std::string user, const int port = 22,
    const std::string command = "sudo shutdown now", const float timeout = 5.0,
    const bool ping_for_success = true)
  : Host(ip, user, port), command_(command), timeout_(timeout), ping_for_success_(ping_for_success)
  {
  }

  ~ShutdownHost() {}

  void request_shutdown()
  {
    ssh_execute_command(get_ip().c_str(), get_user().c_str(), get_command().c_str(), get_port());
    command_time_ = ros::Time::now();
    command_executed_ = true;
  }

  bool update_response()
  {
    if (!is_available()) {
      close_connection();
      throw std::runtime_error("Lost connection");
    }

    if (!ssh_channel_is_open(channel_)) {
      throw std::runtime_error("Channel closed");
    }

    if (timeout_exceeded()) {
      close_connection();
      throw std::runtime_error("Timeout exceeded");
    }

    if ((nbytes_ = ssh_channel_read_nonblocking(channel_, buffer_, sizeof(buffer_), 0)) >= 0) {
      output_.append(buffer_, nbytes_);
      return true;
    }
    close_connection();
    response_received_ = true;
    return false;
  }

  void close_connection()
  {
    if (ssh_channel_is_closed(channel_)) return;

    ssh_channel_send_eof(channel_);
    ssh_channel_close(channel_);
    ssh_channel_free(channel_);
    ssh_disconnect(session_);
    ssh_free(session_);
  }

  std::string get_command() const { return command_; }
  std::string get_response() const { return output_; }
  bool command_executed() const { return command_executed_; }
  bool response_received() const { return response_received_; }

  bool timeout_exceeded()
  {
    return (ros::Time::now() - command_time_) > ros::Duration(timeout_) && is_available();
  }

  bool success()
  {
    if (ping_for_success_) return command_executed_ && response_received_ && !is_available();
    return command_executed_ && response_received_;
  }

private:
  bool command_executed_ = false;
  bool response_received_ = false;
  const bool ping_for_success_;
  const float timeout_;
  const int verbosity_ = SSH_LOG_NOLOG;
  char buffer_[1024];
  int nbytes_;
  const std::string command_;
  std::string output_;
  ros::Time command_time_;

  ssh_session session_;
  ssh_channel channel_;

  void ssh_execute_command(
    const char * host, const char * user, const char * command, const int port)
  {
    session_ = ssh_new();
    if (session_ == NULL) {
      throw std::runtime_error("Failed to open session");
    };

    ssh_options_set(session_, SSH_OPTIONS_HOST, host);
    ssh_options_set(session_, SSH_OPTIONS_USER, user);
    ssh_options_set(session_, SSH_OPTIONS_PORT, &port);
    ssh_options_set(session_, SSH_OPTIONS_LOG_VERBOSITY, &verbosity_);

    if (ssh_connect(session_) != SSH_OK) {
      std::string err = ssh_get_error(session_);
      ssh_free(session_);
      throw std::runtime_error("Error connecting to host: " + err);
    }

    if (ssh_userauth_publickey_auto(session_, NULL, NULL) != SSH_AUTH_SUCCESS) {
      std::string err = ssh_get_error(session_);
      ssh_disconnect(session_);
      ssh_free(session_);
      throw std::runtime_error("Error authenticating with public key: " + err);
    }

    channel_ = ssh_channel_new(session_);
    if (channel_ == NULL) {
      std::string err = ssh_get_error(session_);
      ssh_disconnect(session_);
      ssh_free(session_);
      throw std::runtime_error("Failed to create ssh channel: " + err);
    }

    if (ssh_channel_open_session(channel_) != SSH_OK) {
      std::string err = ssh_get_error(session_);
      ssh_channel_free(channel_);
      ssh_disconnect(session_);
      ssh_free(session_);
      throw std::runtime_error("Failed to open ssh channel: " + err);
    }

    if (ssh_channel_request_exec(channel_, command) != SSH_OK) {
      std::string err = ssh_get_error(session_);
      ssh_channel_close(channel_);
      ssh_channel_free(channel_);
      ssh_disconnect(session_);
      ssh_free(session_);
      throw std::runtime_error("Failed to execute ssh command: " + err);
    }
  }
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SHUTDOWN_HOST_NODE_HPP_
