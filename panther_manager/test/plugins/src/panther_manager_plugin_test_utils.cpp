#include <panther_manager_plugin_test_utils.hpp>

namespace panther_manager_plugin_test
{

PantherManagerPluginTestUtils::PantherManagerPluginTestUtils()
{
}

std::string PantherManagerPluginTestUtils::BuildBehaviorTree(
    const std::string& plugin_name, const std::map<std::string, std::string>& name_and_data_map)
{
  std::stringstream bt;
  auto header = R"(
      <root BTCPP_format="4">
        <BehaviorTree>
          <Sequence>
  )";

  auto footer = R"(
            </Sequence>
        </BehaviorTree>
      </root>
  )";

  bt << header << std::endl;

  for (auto const& [name, value] : name_and_data_map)
  {
    bt << "\t\t\t\t<" << plugin_name << " name=\"" << name << "\" service_name=\"" << name << "\" data=\"" << value
       << "\" />" << std::endl;
  }

  bt << footer;

  return bt.str();
}

BT::Tree &PantherManagerPluginTestUtils::CreateTree(const std::string& plugin_name,
                                                   const std::map<std::string, std::string>& name_and_data_map)
{
  auto xml_text = BuildBehaviorTree(plugin_name, name_and_data_map);
  tree_ = factory_.createTreeFromText(xml_text);
  return tree_;
}

BT::BehaviorTreeFactory& PantherManagerPluginTestUtils::GetFactory()
{
  return factory_;
}

void PantherManagerPluginTestUtils::Start()
{
  rclcpp::init(0, nullptr);
  bt_node_ = std::make_shared<rclcpp::Node>("test_panther_manager_node");
  BT::RosNodeParams params;
  params.nh = bt_node_;

  factory_.registerNodeType<panther_manager::CallSetBoolService>("CallSetBoolService", params);
  factory_.registerNodeType<panther_manager::CallTriggerService>("CallTriggerService", params);
  factory_.registerNodeType<panther_manager::CallSetLedAnimationService>("CallSetLedAnimationService", params);
}

void PantherManagerPluginTestUtils::Stop()
{
  bt_node_.reset();
  rclcpp::shutdown();
  if (executor_thread_)
  {
    executor_.reset();
    executor_thread_->join();
  }
}

void PantherManagerPluginTestUtils::CreateSetBoolServiceServer(
    std::function<void(std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr)>
        service_callback)
{
  service_server_node_ = std::make_shared<rclcpp::Node>("test_set_bool_service");
  set_bool_server_ = service_server_node_->create_service<std_srvs::srv::SetBool>("set_bool", service_callback);
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(service_server_node_);
  executor_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
}

void PantherManagerPluginTestUtils::CreateTriggerServiceServer(std::function<void()> service_callback)
{
}

void PantherManagerPluginTestUtils::CreateSetLEDAnimationServiceServer(std::function<void()> service_callback)
{
}

void PantherManagerPluginTestUtils::spin_executor()
{
  executor_->spin();
}

}  // namespace panther_manager_plugin_test
