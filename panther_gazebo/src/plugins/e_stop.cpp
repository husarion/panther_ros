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

#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>

#include "panther_gazebo/plugins/e_stop.hpp"

namespace panther_gazebo
{

Estop::Estop() : ignition::gui::Plugin() { rclcpp::init(0, nullptr); }

Estop::~Estop() { rclcpp::shutdown(); }

void Estop::LoadConfig(const tinyxml2::XMLElement * plugin_elem)
{
  node_ = rclcpp::Node::make_shared("gz_estop_gui");
  e_stop_reset_client_ = node_->create_client<std_srvs::srv::Trigger>(e_stop_reset_service_);
  e_stop_trigger_client_ = node_->create_client<std_srvs::srv::Trigger>(e_stop_trigger_service_);

  if (title.empty()) {
    title = "E-stop";
  }

  if (plugin_elem) {
    auto namespace_elem = plugin_elem->FirstChildElement("namespace");
    if (namespace_elem != nullptr && namespace_elem->GetText() != nullptr) {
      SetNamespace(namespace_elem->GetText());
    }
  }
}

void Estop::buttonPressed(bool pressed)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto client = pressed ? e_stop_trigger_client_ : e_stop_reset_client_;

  if (!client->service_is_ready()) {
    ignwarn << "Unavailable service: "
            << (pressed ? e_stop_reset_service_ : e_stop_trigger_service_) << std::endl;
    return;
  }

  auto result_future = client->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(node_, result_future, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    ignwarn << "Service call failed for: " << client->get_service_name() << "'!" << std::endl;
    return;
  }

  const auto result = result_future.get();
  if (!result->success) {
    ignwarn << "Service call did not succeed: " << result->message << std::endl;
  }
  ignmsg << "Estop: " << result->message << std::endl;
}

QString Estop::GetNamespace() const { return QString::fromStdString(namespace_); }

void Estop::SetNamespace(const QString & ns)
{
  namespace_ = ns.toStdString();
  e_stop_reset_service_ = namespace_ + kDefaultEStopResetService;
  e_stop_trigger_service_ = namespace_ + kDefaultEStopTriggerService;

  e_stop_reset_client_ = node_->create_client<std_srvs::srv::Trigger>(e_stop_reset_service_);
  e_stop_trigger_client_ = node_->create_client<std_srvs::srv::Trigger>(e_stop_trigger_service_);

  ChangedNamespace();
  ignmsg << "Changed namespace to: " << namespace_ << std::endl;
}

}  // namespace panther_gazebo

IGNITION_ADD_PLUGIN(panther_gazebo::Estop, ignition::gui::Plugin)
