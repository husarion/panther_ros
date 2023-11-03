#include <panther_hardware_interfaces/panther_system_node.hpp>

namespace panther_hardware_interfaces
{

void PantherSystemNode::Initialize()
{
  node_ = std::make_shared<rclcpp::Node>("panther_system_node");
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  executor_thread_ = std::make_unique<std::thread>([this]() {
    while (!stop_executor_) {
      executor_->spin_some();
    }
  });
}

void PantherSystemNode::Activate(std::function<void()> clear_errors)
{
  clear_errors_ = clear_errors;

  driver_state_publisher_ = node_->create_publisher<panther_msgs::msg::DriverState>(
    "~/driver/motor_controllers_state", rclcpp::SensorDataQoS());
  realtime_driver_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<panther_msgs::msg::DriverState>>(
      driver_state_publisher_);

  // TODO: Is it RT safe?
  clear_errors_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "~/clear_errors",
    std::bind(
      &PantherSystemNode::ClearErrorsCb, this, std::placeholders::_1, std::placeholders::_2));
}

void PantherSystemNode::Deactivate()
{
  realtime_driver_state_publisher_.reset();
  driver_state_publisher_.reset();
  clear_errors_srv_.reset();
}

void PantherSystemNode::Deinitialize()
{
  stop_executor_.store(true);
  executor_thread_->join();
  stop_executor_.store(false);

  executor_.reset();
  node_.reset();
}

void PantherSystemNode::UpdateMsgErrorFlags(const RoboteqData & front, const RoboteqData & rear)
{
  auto & driver_state = realtime_driver_state_publisher_->msg_;

  driver_state.front.fault_flag = front.GetFaultFlag().GetMessage();
  driver_state.front.script_flag = front.GetScriptFlag().GetMessage();
  driver_state.front.left_motor.runtime_error = front.GetLeftRuntimeError().GetMessage();
  driver_state.front.right_motor.runtime_error = front.GetRightRuntimeError().GetMessage();

  driver_state.rear.fault_flag = rear.GetFaultFlag().GetMessage();
  driver_state.rear.script_flag = rear.GetScriptFlag().GetMessage();
  driver_state.rear.left_motor.runtime_error = rear.GetLeftRuntimeError().GetMessage();
  driver_state.rear.right_motor.runtime_error = rear.GetRightRuntimeError().GetMessage();
}

void PantherSystemNode::UpdateMsgDriversParameters(
  const DriverState & front, const DriverState & rear)
{
  auto & driver_state = realtime_driver_state_publisher_->msg_;

  driver_state.front.voltage = front.GetVoltage();
  driver_state.front.current = front.GetCurrent();
  driver_state.front.temperature = front.GetTemperature();

  driver_state.rear.voltage = rear.GetVoltage();
  driver_state.rear.current = rear.GetCurrent();
  driver_state.rear.temperature = rear.GetTemperature();
}

void PantherSystemNode::UpdateMsgErrors(
  bool is_error, bool is_write_sdo_error, bool is_read_sdo_error, bool is_read_pdo_error,
  bool front_old_data, bool rear_old_data)
{
  // TODO is_error names
  realtime_driver_state_publisher_->msg_.error = is_error;
  realtime_driver_state_publisher_->msg_.write_sdo_error = is_write_sdo_error;
  realtime_driver_state_publisher_->msg_.read_sdo_error = is_read_sdo_error;
  realtime_driver_state_publisher_->msg_.read_pdo_error = is_read_pdo_error;

  realtime_driver_state_publisher_->msg_.front.old_data = front_old_data;
  realtime_driver_state_publisher_->msg_.rear.old_data = rear_old_data;
}

void PantherSystemNode::PublishDriverState()
{
  if (realtime_driver_state_publisher_->trylock()) {
    realtime_driver_state_publisher_->unlockAndPublish();
  }
}

void PantherSystemNode::ClearErrorsCb(
  std_srvs::srv::Trigger::Request::ConstSharedPtr /* request */,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Clearing errors");
  clear_errors_();
  response->success = true;
}

}  // namespace panther_hardware_interfaces