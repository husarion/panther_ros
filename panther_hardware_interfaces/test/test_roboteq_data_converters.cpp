// Copyright 2023 Husarion sp. z o.o.
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

#include <string>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/roboteq_data_converters.hpp>

TEST(TestRoboteqDataConverters, test_command_converter)
{
  // TODO move it
  panther_hardware_interfaces::DrivetrainSettings settings;
  settings.motor_torque_constant = 0.11;
  settings.gear_ratio = 30.08;
  settings.gearbox_efficiency = 0.75;
  settings.encoder_resolution = 1600;
  settings.max_rpm_motor_speed = 3600.0;

  panther_hardware_interfaces::RoboteqVeloctiyCommandConverter cmd_converter(settings);

  // radians_per_second_to_roboteq_cmd = 79.789678137

  ASSERT_EQ(cmd_converter.Convert(100.0), 1000);
  ASSERT_EQ(cmd_converter.Convert(10.0), 797);
  ASSERT_EQ(cmd_converter.Convert(0.0), 0);
  ASSERT_EQ(cmd_converter.Convert(-10.0), -797);
  ASSERT_EQ(cmd_converter.Convert(-100.0), -1000);
}

TEST(TestRoboteqDataConverters, test_motor_state)
{
  // TODO move it

  panther_hardware_interfaces::DrivetrainSettings settings;
  settings.motor_torque_constant = 0.11;
  settings.gear_ratio = 30.08;
  settings.gearbox_efficiency = 0.75;
  settings.encoder_resolution = 1600;
  settings.max_rpm_motor_speed = 3600.0;

  panther_hardware_interfaces::MotorState motor_state(settings);

  ASSERT_FLOAT_EQ(motor_state.GetPosition(), 0.0);
  ASSERT_FLOAT_EQ(motor_state.GetVelocity(), 0.0);
  ASSERT_FLOAT_EQ(motor_state.GetTorque(), 0.0);

  panther_hardware_interfaces::RoboteqMotorState feedback1;
  feedback1.pos = 48128;
  feedback1.vel = 1000;
  feedback1.current = 1;
  motor_state.SetData(feedback1);

  // roboteq_pos_feedback_to_radians = 0.000130552
  // roboteq_vel_feedback_to_radians_per_second = 0.003481375
  // roboteq_current_feedback_to_newton_meters = 0.24816

  ASSERT_FLOAT_EQ(motor_state.GetPosition(), 6.283185307);
  ASSERT_FLOAT_EQ(motor_state.GetVelocity(), 3.481375);
  ASSERT_FLOAT_EQ(motor_state.GetTorque(), 0.24816);

  panther_hardware_interfaces::RoboteqMotorState feedback2;
  feedback2.pos = -48128;
  feedback2.vel = -1000;
  feedback2.current = -1;
  motor_state.SetData(feedback2);

  ASSERT_FLOAT_EQ(motor_state.GetPosition(), -6.283185307);
  ASSERT_FLOAT_EQ(motor_state.GetVelocity(), -3.481375);
  ASSERT_FLOAT_EQ(motor_state.GetTorque(), -0.24816);
}

TEST(TestRoboteqDataConverters, test_flag_error)
{
  panther_hardware_interfaces::FlagError flag_error(
    {"error1", "error2", "error3", "error4", "error5", "error6", "error7", "error8"});

  ASSERT_FALSE(flag_error.IsError());
  ASSERT_EQ(flag_error.GetErrorLog(), "");

  flag_error.SetSurpressedFlags(0b11011101);
  flag_error.SetData(0b00000001);
  ASSERT_TRUE(flag_error.IsError());
  ASSERT_EQ(flag_error.GetErrorLog(), "error1 ");

  flag_error.SetData(0b00100010);
  ASSERT_FALSE(flag_error.IsError());
  ASSERT_EQ(flag_error.GetErrorLog(), "");

  flag_error.SetData(0b10000001);
  ASSERT_TRUE(flag_error.IsError());
  ASSERT_EQ(flag_error.GetErrorLog(), "error1 error8 ");
}

void TestFaultFlagMsg(
  const panther_msgs::msg::FaultFlag & msg, const std::vector<bool> & expected_values)
{
  if (expected_values.size() != 9) {
    throw std::runtime_error("Wrong size of expected_values in TestFaultFlagMsg");
  }

  ASSERT_EQ(msg.overheat, expected_values[0]);
  ASSERT_EQ(msg.overvoltage, expected_values[1]);
  ASSERT_EQ(msg.undervoltage, expected_values[2]);
  ASSERT_EQ(msg.short_circuit, expected_values[3]);
  ASSERT_EQ(msg.emergency_stop, expected_values[4]);
  ASSERT_EQ(msg.motor_or_sensor_setup_fault, expected_values[5]);
  ASSERT_EQ(msg.mosfet_failure, expected_values[6]);
  ASSERT_EQ(msg.default_config_loaded_at_startup, expected_values[7]);
  ASSERT_EQ(msg.can_net_err, expected_values[8]);
}

TEST(TestRoboteqDataConverters, test_fault_flag)
{
  panther_hardware_interfaces::FaultFlag fault_flag;
  panther_msgs::msg::FaultFlag msg;

  fault_flag.SetData(0b00000001, false);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {true, false, false, false, false, false, false, false, false});
  fault_flag.SetData(0b00000010, false);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, true, false, false, false, false, false, false, false});
  fault_flag.SetData(0b00000100, false);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, true, false, false, false, false, false, false});
  fault_flag.SetData(0b00001000, false);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, true, false, false, false, false, false});
  fault_flag.SetData(0b00010000, false);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, true, false, false, false, false});
  fault_flag.SetData(0b00100000, false);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, false, true, false, false, false});
  fault_flag.SetData(0b01000000, false);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, false, false, true, false, false});
  fault_flag.SetData(0b10000000, false);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, false, false, false, true, false});
  fault_flag.SetData(0b00000000, true);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, false, false, false, false, true});

  fault_flag.SetData(0b00100010, true);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, true, false, false, false, true, false, false, true});

  fault_flag.SetSurpressedFlags(0b11111110);
  fault_flag.SetData(0b00000001, false);
  // fault flag still should be set, it just won't be treated as an error
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {true, false, false, false, false, false, false, false, false});
}

void TestScriptFlagMsg(
  const panther_msgs::msg::ScriptFlag & msg, const std::vector<bool> & expected_values)
{
  if (expected_values.size() != 3) {
    throw std::runtime_error("Wrong size of expected_values in TestScriptFlagMsg");
  }

  ASSERT_EQ(msg.loop_error, expected_values[0]);
  ASSERT_EQ(msg.encoder_disconected, expected_values[1]);
  ASSERT_EQ(msg.amp_limiter, expected_values[2]);
}

TEST(TestRoboteqDataConverters, test_script_flag)
{
  panther_hardware_interfaces::ScriptFlag script_flag;
  panther_msgs::msg::ScriptFlag msg;

  script_flag.SetData(0b00000001);
  TestScriptFlagMsg(script_flag.GetMessage(), {true, false, false});
  script_flag.SetData(0b00000010);
  TestScriptFlagMsg(script_flag.GetMessage(), {false, true, false});
  script_flag.SetData(0b00000100);
  TestScriptFlagMsg(script_flag.GetMessage(), {false, false, true});

  script_flag.SetData(0b00100110);
  TestScriptFlagMsg(script_flag.GetMessage(), {false, true, true});

  script_flag.SetSurpressedFlags(0b11111101);
  script_flag.SetData(0b00000010);
  // fault flag still should be set, it just won't be treated as an error
  TestScriptFlagMsg(script_flag.GetMessage(), {false, true, false});
}

void TestRuntimeErrorMsg(
  const panther_msgs::msg::RuntimeError & msg, const std::vector<bool> & expected_values)
{
  if (expected_values.size() != 7) {
    throw std::runtime_error("Wrong size of expected_values in TestFaultFlagMsg");
  }

  ASSERT_EQ(msg.amps_limit_active, expected_values[0]);
  ASSERT_EQ(msg.motor_stall, expected_values[1]);
  ASSERT_EQ(msg.loop_error, expected_values[2]);
  ASSERT_EQ(msg.safety_stop_active, expected_values[3]);
  ASSERT_EQ(msg.forward_limit_triggered, expected_values[4]);
  ASSERT_EQ(msg.reverse_limit_triggered, expected_values[5]);
  ASSERT_EQ(msg.amps_trigger_activated, expected_values[6]);
}

TEST(TestRoboteqDataConverters, test_runtime_error)
{
  panther_hardware_interfaces::RuntimeError runtime_error;
  panther_msgs::msg::RuntimeError msg;

  runtime_error.SetData(0b00000001);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {true, false, false, false, false, false, false});
  runtime_error.SetData(0b00000010);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, true, false, false, false, false, false});
  runtime_error.SetData(0b00000100);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, true, false, false, false, false});
  runtime_error.SetData(0b00001000);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, false, true, false, false, false});
  runtime_error.SetData(0b00010000);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, false, false, true, false, false});
  runtime_error.SetData(0b00100000);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, false, false, false, true, false});
  runtime_error.SetData(0b01000000);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, false, false, false, false, true});

  runtime_error.SetData(0b00100010);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, true, false, false, false, true, false});

  runtime_error.SetSurpressedFlags(0b11111011);
  runtime_error.SetData(0b00000100);
  // fault flag still should be set, it just won't be treated as an error
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, true, false, false, false, false});
}

TEST(TestRoboteqDataConverters, test_driver_state)
{
  panther_hardware_interfaces::DriverState driver_state;

  ASSERT_FLOAT_EQ(driver_state.GetTemperature(), 0.0);
  ASSERT_FLOAT_EQ(driver_state.GetVoltage(), 0.0);
  ASSERT_FLOAT_EQ(driver_state.GetCurrent(), 0.0);

  driver_state.SetTemperature(32);
  driver_state.SetVoltage(365);
  driver_state.SetBatAmps1(15);
  driver_state.SetBatAmps2(20);

  ASSERT_FLOAT_EQ(driver_state.GetTemperature(), 32);
  ASSERT_FLOAT_EQ(driver_state.GetVoltage(), 36.5);
  ASSERT_FLOAT_EQ(driver_state.GetCurrent(), 3.5);
}

TEST(TestRoboteqDataConverters, test_roboteq_data)
{
  // TODO move it
  panther_hardware_interfaces::DrivetrainSettings settings;
  settings.motor_torque_constant = 0.11;
  settings.gear_ratio = 30.08;
  settings.gearbox_efficiency = 0.75;
  settings.encoder_resolution = 1600;
  settings.max_rpm_motor_speed = 3600.0;

  panther_hardware_interfaces::RoboteqData roboteq_data(settings);

  ASSERT_FALSE(roboteq_data.IsError());

  panther_hardware_interfaces::RoboteqMotorState left_state = {48128, 1000, 1};
  panther_hardware_interfaces::RoboteqMotorState right_state = {0, 0, 0};
  roboteq_data.SetMotorStates(left_state, right_state, true);

  ASSERT_TRUE(roboteq_data.IsError());
  ASSERT_TRUE(roboteq_data.IsDataTimedOut());

  roboteq_data.SetMotorStates(left_state, right_state, false);
  ASSERT_FALSE(roboteq_data.IsError());
  ASSERT_FLOAT_EQ(roboteq_data.GetRightMotorState().GetPosition(), 0.0);
  ASSERT_FLOAT_EQ(roboteq_data.GetRightMotorState().GetVelocity(), 0.0);
  ASSERT_FLOAT_EQ(roboteq_data.GetRightMotorState().GetTorque(), 0.0);
  ASSERT_FLOAT_EQ(roboteq_data.GetLeftMotorState().GetPosition(), 6.283185307);
  ASSERT_FLOAT_EQ(roboteq_data.GetLeftMotorState().GetVelocity(), 3.481375);
  ASSERT_FLOAT_EQ(roboteq_data.GetLeftMotorState().GetTorque(), 0.24816);

  roboteq_data.SetFlags(0b00000001, 0b00000010, 0b00000100, 0b00010000, true);
  ASSERT_TRUE(roboteq_data.IsError());

  ASSERT_TRUE(roboteq_data.GetFaultFlag().GetMessage().overheat);
  ASSERT_TRUE(roboteq_data.GetFaultFlag().GetMessage().can_net_err);
  ASSERT_TRUE(roboteq_data.GetScriptFlag().GetMessage().encoder_disconected);
  ASSERT_TRUE(roboteq_data.GetLeftRuntimeError().GetMessage().loop_error);
  ASSERT_TRUE(roboteq_data.GetRightRuntimeError().GetMessage().forward_limit_triggered);

  roboteq_data.SetTemperature(32);
  roboteq_data.SetVoltage(365);
  roboteq_data.SetBatAmps1(15);
  roboteq_data.SetBatAmps2(20);

  ASSERT_FLOAT_EQ(roboteq_data.GetDriverState().GetTemperature(), 32);
  ASSERT_FLOAT_EQ(roboteq_data.GetDriverState().GetVoltage(), 36.5);
  ASSERT_FLOAT_EQ(roboteq_data.GetDriverState().GetCurrent(), 3.5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
