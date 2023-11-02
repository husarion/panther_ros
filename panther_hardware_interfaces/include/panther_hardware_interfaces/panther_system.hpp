#ifndef PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_
#define PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <realtime_tools/realtime_publisher.h>

#include <std_srvs/srv/trigger.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <panther_hardware_interfaces/gpio_driver.hpp>
#include <panther_hardware_interfaces/panther_wheels_controller.hpp>
#include <panther_hardware_interfaces/canopen_error_filter.hpp>

namespace panther_hardware_interfaces
{
using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

class PantherSystemNode
{
public:
  PantherSystemNode() {}

  void UpdateMsgDriversErrorsState(const RoboteqData & front, const RoboteqData & rear)
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

  void UpdateMsgDriversState(const DriverState & front, const DriverState & rear)
  {
    auto & driver_state = realtime_driver_state_publisher_->msg_;

    driver_state.front.voltage = front.GetVoltage();
    driver_state.front.current = front.GetCurrent();
    driver_state.front.temperature = front.GetTemperature();

    driver_state.rear.voltage = rear.GetVoltage();
    driver_state.rear.current = rear.GetCurrent();
    driver_state.rear.temperature = rear.GetTemperature();
  }

  void UpdateMsgErrors(
    bool is_error, bool is_write_sdo_error, bool is_read_sdo_error, bool is_read_pdo_error,
    bool front_old_data, bool rear_old_data)
  {
    realtime_driver_state_publisher_->msg_.error = is_error;
    // TODO rename
    realtime_driver_state_publisher_->msg_.write_error = is_write_sdo_error;
    realtime_driver_state_publisher_->msg_.read_error_sdo = is_read_sdo_error;
    realtime_driver_state_publisher_->msg_.read_error_pdo = is_read_pdo_error;

    realtime_driver_state_publisher_->msg_.front.old_data = front_old_data;
    realtime_driver_state_publisher_->msg_.rear.old_data = rear_old_data;
  }

  void PublishDriverState()
  {
    if (realtime_driver_state_publisher_->trylock()) {
      realtime_driver_state_publisher_->unlockAndPublish();
    }
  }

  void Configure()
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

  void Activate(std::function<void()> clear_errors)
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

  void ResetPublishers()
  {
    realtime_driver_state_publisher_.reset();
    driver_state_publisher_.reset();
    clear_errors_srv_.reset();
  }

  void DestroyNode()
  {
    stop_executor_.store(true);
    // TODO: check
    executor_thread_->join();
    stop_executor_.store(false);

    executor_.reset();
    node_.reset();
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;
  std::unique_ptr<std::thread> executor_thread_;

  std::atomic_bool stop_executor_ = false;

  rclcpp::Publisher<panther_msgs::msg::DriverState>::SharedPtr driver_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<panther_msgs::msg::DriverState>>
    realtime_driver_state_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_errors_srv_;

  std::function<void()> clear_errors_;

  void ClearErrorsCb(
    std_srvs::srv::Trigger::Request::ConstSharedPtr /* request */,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Clearing errors");
    clear_errors_();
    response->success = true;
  }
};

class PantherSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PantherSystem)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<StateInterface> export_state_interfaces() override;
  std::vector<CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void CheckJointSize();
  void SortJointNames();
  void CheckJointNames();
  void SetInitialValues();
  void CheckInterfaces();
  void ReadDrivetrainSettings();
  void ReadCanOpenSettings();
  void ReadInitializationActivationAttempts();
  void ReadParametersAndCreateCanOpenErrorFilter();

  void UpdateHwStates();

  static constexpr size_t kJointsSize = 4;

  // consider adding position and torque mode after updating roboteq firmware to 2.1a
  // In 2.1 both position and torque mode aren't really stable and safe
  // in torque mode sometimes after killing software motor moves and it generally isn't well tuned
  // position mode also isn't really stable (reacts abruptly to spikes, which we hope will be fixed
  // in the new firmware)

  double hw_commands_velocities_[kJointsSize];

  double hw_states_positions_[kJointsSize];
  double hw_states_velocities_[kJointsSize];
  double hw_states_efforts_[kJointsSize];

  // Define expected joint order, so that it doesn't mattter order defined in the panther_macro
  // it is expected that joint name should contain these specifiers
  std::string joint_order_[kJointsSize] = {"fl", "fr", "rl", "rr"};
  std::string joints_names_sorted_[kJointsSize];

  std::unique_ptr<GPIOController> gpio_controller_;
  std::shared_ptr<PantherWheelsController> roboteq_controller_;

  DrivetrainSettings drivetrain_settings_;
  CanOpenSettings canopen_settings_;

  std::shared_ptr<CanOpenErrorFilter> canopen_error_filter_;

  PantherSystemNode panther_system_node_;

  // Sometimes SDO errors can happen during initialization and activation of roboteqs, in this cases it is better to retry
  // [ros2_control_node-1] error: SDO abort code 05040000 received on upload request of object 1000 (Device type) to node 02: SDO protocol timed out
  // [ros2_control_node-1] error: SDO abort code 05040000 received on upload request of sub-object 1018:01 (Vendor-ID) to node 02: SDO protocol timed out
  unsigned max_roboteq_initialization_attempts_;
  unsigned max_roboteq_activation_attempts_;

  const unsigned max_safety_stop_attempts_ = 20;

  bool OperationWithAttempts(
    std::function<void()> operation, unsigned max_attempts, std::function<void()> on_error);
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_SYSTEM_HPP_