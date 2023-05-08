#ifndef PANTHER_MANAGER_HOST_HPP_
#define PANTHER_MANAGER_HOST_HPP_

#include <cstdlib>
#include <string>

namespace panther_manager
{

class Host
{
public:
  Host(const std::string ip, const std::string user, const int port = 22)
  : ip_(ip), user_(user), port_(port)
  {
  }

  std::string get_ip() const { return ip_; }
  std::string get_user() const { return user_; }
  int get_port() const { return port_; }

  bool is_available() { return system(("ping -c 1 -w 1 " + ip_ + " > /dev/null").c_str()) == 0; }

  bool operator==(const Host & other)
  {
    return (ip_ == other.ip_) && (user_ == other.user_) && (port_ == other.port_);
  }

  bool operator!=(const Host & other)
  {
    return (ip_ != other.ip_) || (user_ != other.user_) || (port_ != other.port_);
  }

  bool operator<(const Host & other) const { return ip_ < other.ip_; }

private:
  const std::string ip_;
  const std::string user_;
  const int port_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_HOST_HPP_