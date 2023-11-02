#include <panther_hardware_interfaces/panther_wheels_controller.hpp>

namespace panther_hardware_interfaces
{

// TODO
int const kSchedPriority = 55;

PantherWheelsController::PantherWheelsController(
  CanOpenSettings canopen_settings, DrivetrainSettings drivetrain_settings)
: canopen_controller_(canopen_settings),
  front_data_(drivetrain_settings),
  rear_data_(drivetrain_settings),
  roboteq_command_converter_(drivetrain_settings),
  pdo_feedback_timeout_(canopen_settings.pdo_feedback_timeout)
{
}

void PantherWheelsController::Initialize() { canopen_controller_.Initialize(); }

void PantherWheelsController::Deinitialize() { canopen_controller_.Deinitialize(); }

void PantherWheelsController::Activate()
{
  // Activation procedure - it is necessary to first reset scripts, wait for a bit (1 second)
  // and then send 0 commands for some time (also 1 second)

  try {
    canopen_controller_.GetFrontDriver()->ResetRoboteqScript();
  } catch (std::runtime_error & err) {
    throw std::runtime_error(
      "Front driver reset roboteq script exception: " + std::string(err.what()));
  }

  try {
    canopen_controller_.GetRearDriver()->ResetRoboteqScript();
  } catch (std::runtime_error & err) {
    throw std::runtime_error(
      "Rear driver reset roboteq script exception: " + std::string(err.what()));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  try {
    canopen_controller_.GetFrontDriver()->SendRoboteqCmd(0, 0);
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Front driver send 0 command exception: " + std::string(err.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->SendRoboteqCmd(0, 0);
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Rear driver send 0 command exception: " + std::string(err.what()));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void PantherWheelsController::UpdateSystemFeedback()
{
  RoboteqDriverFeedback front_driver_feedback =
    canopen_controller_.GetFrontDriver()->ReadRoboteqDriverFeedback();
  RoboteqDriverFeedback rear_driver_feedback =
    canopen_controller_.GetRearDriver()->ReadRoboteqDriverFeedback();

  timespec front_driver_ts = front_driver_feedback.timestamp;
  timespec rear_driver_ts = rear_driver_feedback.timestamp;
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  bool front_data_too_old =
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(front_driver_ts) >
     pdo_feedback_timeout_);
  bool rear_data_too_old =
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(rear_driver_ts) >
     pdo_feedback_timeout_);

  // Channel 1 - right, Channel 2 - left
  front_data_.SetMotorStates(
    front_driver_feedback.motor_2, front_driver_feedback.motor_1, front_data_too_old);
  rear_data_.SetMotorStates(
    rear_driver_feedback.motor_2, rear_driver_feedback.motor_1, rear_data_too_old);

  bool front_can_error = canopen_controller_.GetFrontDriver()->get_can_error();
  bool rear_can_error = canopen_controller_.GetRearDriver()->get_can_error();

  front_data_.SetFlags(
    front_driver_feedback.fault_flags, front_driver_feedback.script_flags,
    front_driver_feedback.runtime_stat_flag_motor_2,
    front_driver_feedback.runtime_stat_flag_motor_1, front_can_error);

  rear_data_.SetFlags(
    rear_driver_feedback.fault_flags, rear_driver_feedback.script_flags,
    rear_driver_feedback.runtime_stat_flag_motor_2, rear_driver_feedback.runtime_stat_flag_motor_1,
    rear_can_error);

  if (front_can_error || rear_can_error) {
    throw std::runtime_error("CAN error detected when trying to read roboteq feedback");
  }
}

bool PantherWheelsController::UpdateDriversState()
{
  // TODO: bat_amps 1 and 2 are read in separate iterations and in the result reading
  // is not correct for one iteraction
  try {
    switch (current_update_) {
      case 0:
        front_data_.SetTemperature(canopen_controller_.GetFrontDriver()->ReadTemperature());
        break;
      case 1:
        front_data_.SetVoltage(canopen_controller_.GetFrontDriver()->ReadVoltage());
        break;
      case 2:
        front_data_.SetBatAmps1(canopen_controller_.GetFrontDriver()->ReadBatAmps1());
        break;
      case 3:
        front_data_.SetBatAmps2(canopen_controller_.GetFrontDriver()->ReadBatAmps2());
        break;
      case 4:
        rear_data_.SetTemperature(canopen_controller_.GetRearDriver()->ReadTemperature());
        break;
      case 5:
        rear_data_.SetVoltage(canopen_controller_.GetRearDriver()->ReadVoltage());
        break;
      case 6:
        rear_data_.SetBatAmps1(canopen_controller_.GetRearDriver()->ReadBatAmps1());
        break;
      case 7:
        rear_data_.SetBatAmps2(canopen_controller_.GetRearDriver()->ReadBatAmps2());
        break;
    }

    ++current_update_;
    if (current_update_ > 7) {
      current_update_ = 0;
      return true;
    }

    return false;

  } catch (std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to read roboteq drivers feedback: " + std::string(e.what()));
  }
}

void PantherWheelsController::WriteSpeed(
  double speed_fl, double speed_fr, double speed_rl, double speed_rr)
{
  // Channel 1 - right, Channel 2 - left
  try {
    canopen_controller_.GetFrontDriver()->SendRoboteqCmd(
      roboteq_command_converter_.Convert(speed_fr), roboteq_command_converter_.Convert(speed_fl));
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Front driver send roboteq cmd failed: " + std::string(err.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->SendRoboteqCmd(
      roboteq_command_converter_.Convert(speed_rr), roboteq_command_converter_.Convert(speed_rl));
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Rear driver send roboteq cmd failed: " + std::string(err.what()));
  }

  // TODO
  if (
    canopen_controller_.GetFrontDriver()->get_can_error() ||
    canopen_controller_.GetRearDriver()->get_can_error()) {
    throw std::runtime_error("CAN error detected when trying to write speed commands");
  }
}

void PantherWheelsController::TurnOnEstop()
{
  try {
    canopen_controller_.GetFrontDriver()->TurnOnEstop();
    canopen_controller_.GetRearDriver()->TurnOnEstop();
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Exception when trying to turn on estop: " + std::string(err.what()));
  }
}

void PantherWheelsController::TurnOffEstop()
{
  // TODO: separate
  try {
    canopen_controller_.GetFrontDriver()->TurnOffEstop();
    canopen_controller_.GetRearDriver()->TurnOffEstop();
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Exception when trying to turn off estop: " + std::string(err.what()));
  }
}

// TODO: add this info to readme
// Safety stop is turned off when 0 command is published
void PantherWheelsController::TurnOnSafetyStop()
{
  try {
    canopen_controller_.GetFrontDriver()->TurnOnSafetyStop();
    canopen_controller_.GetRearDriver()->TurnOnSafetyStop();
  } catch (std::runtime_error & err) {
    throw std::runtime_error(
      "Exception when trying to turn on safety stop: " + std::string(err.what()));
  }
}

}  // namespace panther_hardware_interfaces