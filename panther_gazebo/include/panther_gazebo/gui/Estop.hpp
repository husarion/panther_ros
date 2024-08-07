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

#ifndef PANTHER_GAZEBO__GUI__ESTOP_HPP_
#define PANTHER_GAZEBO__GUI__ESTOP_HPP_

#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <string>

namespace ignition
{
namespace gui
{
class Estop : public Plugin
{
  Q_OBJECT

  Q_PROPERTY(QString ns READ getNamespace WRITE setNamespace NOTIFY changedNamespace)

public:
  Estop();
  virtual ~Estop();
  void LoadConfig(const tinyxml2::XMLElement * pluginElem) override;
  Q_INVOKABLE QString getNamespace() const;

public slots:
  void setNamespace(const QString & ns);

signals:
  void changedNamespace();

protected slots:
  void buttonPressed(bool pressed);

private:
  rclcpp::Node::SharedPtr node_;
  std::string namespace_ = "";
  std::string e_stop_reset_service_ = "/hardware/e_stop_reset";
  std::string e_stop_trigger_service_ = "/hardware/e_stop_trigger";

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr e_stop_reset_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr e_stop_trigger_client_;
};
}  // namespace gui
}  // namespace ignition

#endif  // PANTHER_GAZEBO__GUI__ESTOP_HPP_
