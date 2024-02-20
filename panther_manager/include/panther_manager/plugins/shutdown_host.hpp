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

#ifndef PANTHER_MANAGER_SHUTDOWN_HOST_HPP_
#define PANTHER_MANAGER_SHUTDOWN_HOST_HPP_

#include <cstdlib>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

#include <libssh/libsshpp.hpp>

namespace panther_manager
{

enum class ShutdownHostState
{
  IDLE = 0,
  COMMAND_EXECUTED,
  RESPONSE_RECEIVED,
  PINGING,
  SKIPPED,
  SUCCESS,
  FAILURE,
};

class ShutdownHost
{
public:
  // default constructor
  ShutdownHost()
    : ip_("")
    , user_("")
    , port_(22)
    , command_("")
    , timeout_(5000)
    , ping_for_success_(true)
    , hash_(std::hash<std::string>{}(""))
  {
  }
  ShutdownHost(const std::string ip, const std::string user, const int port = 22,
               const std::string command = "sudo shutdown now", const float timeout = 5.0,
               const bool ping_for_success = true)
    : ip_(ip)
    , user_(user)
    , port_(port)
    , command_(command)
    , timeout_(static_cast<long long>(timeout * 1000))
    , ping_for_success_(ping_for_success)
    , hash_(std::hash<std::string>{}(ip + user + std::to_string(port)))
    , state_(ShutdownHostState::IDLE)
  {
  }

  ~ShutdownHost()
  {
  }

  void call()
  {
    switch (state_)
    {
      case ShutdownHostState::IDLE:
        if (!is_available())
        {
          state_ = ShutdownHostState::SKIPPED;
          break;
        }

        try
        {
          request_shutdown();
        }
        catch (std::runtime_error err)
        {
          state_ = ShutdownHostState::FAILURE;
          failure_reason_ = err.what();
          break;
        }
        state_ = ShutdownHostState::COMMAND_EXECUTED;
        break;

      case ShutdownHostState::COMMAND_EXECUTED:
        try
        {
          if (update_response())
          {
            break;
          }
        }
        catch (std::runtime_error err)
        {
          state_ = ShutdownHostState::FAILURE;
          failure_reason_ = err.what();
          break;
        }
        state_ = ShutdownHostState::RESPONSE_RECEIVED;
        break;

      case ShutdownHostState::RESPONSE_RECEIVED:
        state_ = ShutdownHostState::PINGING;
        break;
      case ShutdownHostState::PINGING:
        if (ping_for_success_ ? !is_available() : true)
        {
          state_ = ShutdownHostState::SUCCESS;
          break;
        }
        if (timeout_exceeded())
        {
          state_ = ShutdownHostState::FAILURE;
          failure_reason_ = "Timeout exceeded";
        }
        break;

      default:
        break;
    }
  }

  bool is_available() const
  {
    return system(("ping -c 1 -w 1 " + ip_ + " > /dev/null").c_str()) == 0;
  }

  void close_connection()
  {
    if (ssh_channel_is_closed(channel_))
    {
      return;
    }

    ssh_channel_send_eof(channel_);
    ssh_channel_close(channel_);
    ssh_channel_free(channel_);
    ssh_disconnect(session_);
    ssh_free(session_);
  }

  int get_port() const
  {
    return port_;
  }

  std::string get_ip() const
  {
    return ip_;
  }

  std::string get_user() const
  {
    return user_;
  }

  std::string get_command() const
  {
    return command_;
  }

  std::string get_error() const
  {
    return failure_reason_;
  }

  std::string get_response() const
  {
    return output_;
  }

  ShutdownHostState get_state() const
  {
    return state_;
  }

  bool operator==(const ShutdownHost& other) const
  {
    return hash_ == other.hash_;
  }

  bool operator!=(const ShutdownHost& other) const
  {
    return hash_ != other.hash_;
  }

  bool operator<(const ShutdownHost& other) const
  {
    return hash_ < other.hash_;
  }

private:
  const std::string ip_;
  const std::string user_;
  const std::string command_;
  const std::size_t hash_;
  const int port_;
  const bool ping_for_success_;
  const std::chrono::milliseconds timeout_;

  char buffer_[1024];
  const int verbosity_ = SSH_LOG_NOLOG;
  int nbytes_;
  std::string output_;
  std::string failure_reason_;
  std::chrono::time_point<std::chrono::steady_clock> command_time_;
  ShutdownHostState state_;

  ssh_session session_;
  ssh_channel channel_;

  void request_shutdown()
  {
    ssh_execute_command(command_);
    command_time_ = std::chrono::steady_clock::now();
  }

  bool update_response()
  {
    if (!is_available())
    {
      close_connection();
      throw std::runtime_error("Lost connection");
    }

    if (!ssh_channel_is_open(channel_))
    {
      throw std::runtime_error("Channel closed");
    }

    if (timeout_exceeded())
    {
      close_connection();
      throw std::runtime_error("Timeout exceeded");
    }

    if ((nbytes_ = ssh_channel_read_nonblocking(channel_, buffer_, sizeof(buffer_), 0)) >= 0)
    {
      output_.append(buffer_, nbytes_);
      return true;
    }
    close_connection();
    return false;
  }

  bool timeout_exceeded()
  {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - command_time_);

    return elapsed > timeout_ && is_available();
  }

  void ssh_execute_command(const std::string& command)
  {
    session_ = ssh_new();
    if (session_ == NULL)
    {
      throw std::runtime_error("Failed to open session");
    };

    ssh_options_set(session_, SSH_OPTIONS_HOST, ip_.c_str());
    ssh_options_set(session_, SSH_OPTIONS_USER, user_.c_str());
    ssh_options_set(session_, SSH_OPTIONS_PORT, &port_);
    ssh_options_set(session_, SSH_OPTIONS_LOG_VERBOSITY, &verbosity_);

    if (ssh_connect(session_) != SSH_OK)
    {
      std::string err = ssh_get_error(session_);
      ssh_free(session_);
      throw std::runtime_error("Error connecting to host: " + err);
    }

    if (ssh_userauth_publickey_auto(session_, NULL, NULL) != SSH_AUTH_SUCCESS)
    {
      std::string err = ssh_get_error(session_);
      ssh_disconnect(session_);
      ssh_free(session_);
      throw std::runtime_error("Error authenticating with public key: " + err);
    }

    channel_ = ssh_channel_new(session_);
    if (channel_ == NULL)
    {
      std::string err = ssh_get_error(session_);
      ssh_disconnect(session_);
      ssh_free(session_);
      throw std::runtime_error("Failed to create ssh channel: " + err);
    }

    if (ssh_channel_open_session(channel_) != SSH_OK)
    {
      std::string err = ssh_get_error(session_);
      ssh_channel_free(channel_);
      ssh_disconnect(session_);
      ssh_free(session_);
      throw std::runtime_error("Failed to open ssh channel: " + err);
    }

    if (ssh_channel_request_exec(channel_, command.c_str()) != SSH_OK)
    {
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
