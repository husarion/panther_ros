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

#include <test_constants.hpp>

TEST(TestRoboteqDataConverters, test_command_converter)
{
  panther_hardware_interfaces::RoboteqVelocityCommandConverter cmd_converter(
    panther_hardware_interfaces_test::kDrivetrainSettings);

  // radians_per_second_to_roboteq_cmd = 79.789678137

  ASSERT_EQ(cmd_converter.Convert(100.0), 1000);
  ASSERT_EQ(cmd_converter.Convert(10.0), 797);
  ASSERT_EQ(cmd_converter.Convert(0.0), 0);
  ASSERT_EQ(cmd_converter.Convert(-10.0), -797);
  ASSERT_EQ(cmd_converter.Convert(-100.0), -1000);
}

TEST(TestRoboteqDataConverters, test_motor_state)
{
  panther_hardware_interfaces::MotorState motor_state(
    panther_hardware_interfaces_test::kDrivetrainSettings);

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
    {"error1", "error2", "error3", "error4", "error5", "error6", "error7", "error8"},
    {"error2", "error6"});

  ASSERT_FALSE(flag_error.IsError());
  ASSERT_EQ(flag_error.GetErrorLog(), "");

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
  if (expected_values.size() != 8) {
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
}

TEST(TestRoboteqDataConverters, test_fault_flag)
{
  panther_hardware_interfaces::FaultFlag fault_flag;
  panther_msgs::msg::FaultFlag msg;

  fault_flag.SetData(0b00000001);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {true, false, false, false, false, false, false, false});
  fault_flag.SetData(0b00000010);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, true, false, false, false, false, false, false});
  fault_flag.SetData(0b00000100);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, true, false, false, false, false, false});
  fault_flag.SetData(0b00001000);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, true, false, false, false, false});
  fault_flag.SetData(0b00010000);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, true, false, false, false});
  fault_flag.SetData(0b00100000);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, false, true, false, false});
  fault_flag.SetData(0b01000000);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, false, false, true, false});
  fault_flag.SetData(0b10000000);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, false, false, false, true});

  fault_flag.SetData(0b00100010);
  TestFaultFlagMsg(fault_flag.GetMessage(), {false, true, false, false, false, true, false, false});
}

void TestScriptFlagMsg(
  const panther_msgs::msg::ScriptFlag & msg, const std::vector<bool> & expected_values)
{
  if (expected_values.size() != 3) {
    throw std::runtime_error("Wrong size of expected_values in TestScriptFlagMsg");
  }

  ASSERT_EQ(msg.loop_error, expected_values[0]);
  ASSERT_EQ(msg.encoder_disconnected, expected_values[1]);
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

  runtime_error.SetData(0b00001000);
  // fault flag still should be set, it just won't be treated as an error
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, false, true, false, false, false});
}

TEST(TestRoboteqDataConverters, test_driver_state)
{
  panther_hardware_interfaces::DriverState driver_state;

  ASSERT_FLOAT_EQ(driver_state.GetTemperature(), 0.0);
  ASSERT_FLOAT_EQ(driver_state.GetVoltage(), 0.0);
  ASSERT_FLOAT_EQ(driver_state.GetCurrent(), 0.0);

  driver_state.SetTemperature(32);
  driver_state.SetVoltage(365);
  driver_state.SetBatteryCurrent1(15);
  driver_state.SetBatteryCurrent2(20);

  ASSERT_FLOAT_EQ(driver_state.GetTemperature(), 32);
  ASSERT_FLOAT_EQ(driver_state.GetVoltage(), 36.5);
  ASSERT_FLOAT_EQ(driver_state.GetCurrent(), 3.5);
}

TEST(TestRoboteqDataConverters, test_roboteq_data)
{
  panther_hardware_interfaces::RoboteqData roboteq_data(
    panther_hardware_interfaces_test::kDrivetrainSettings);

  ASSERT_FALSE(roboteq_data.IsError());

  panther_hardware_interfaces::RoboteqMotorState left_state = {48128, 1000, 1};
  panther_hardware_interfaces::RoboteqMotorState right_state = {0, 0, 0};
  roboteq_data.SetMotorsStates(left_state, right_state, true);

  ASSERT_TRUE(roboteq_data.IsError());
  ASSERT_TRUE(roboteq_data.IsMotorStatesDataTimedOut());

  roboteq_data.SetMotorsStates(left_state, right_state, false);

  ASSERT_FALSE(roboteq_data.IsError());

  roboteq_data.SetCANNetErr(true);

  ASSERT_TRUE(roboteq_data.IsError());
  ASSERT_TRUE(roboteq_data.IsCANNetErr());

  roboteq_data.SetCANNetErr(false);
  ASSERT_FALSE(roboteq_data.IsError());

  ASSERT_FLOAT_EQ(roboteq_data.GetRightMotorState().GetPosition(), 0.0);
  ASSERT_FLOAT_EQ(roboteq_data.GetRightMotorState().GetVelocity(), 0.0);
  ASSERT_FLOAT_EQ(roboteq_data.GetRightMotorState().GetTorque(), 0.0);
  ASSERT_FLOAT_EQ(roboteq_data.GetLeftMotorState().GetPosition(), 6.283185307);
  ASSERT_FLOAT_EQ(roboteq_data.GetLeftMotorState().GetVelocity(), 3.481375);
  ASSERT_FLOAT_EQ(roboteq_data.GetLeftMotorState().GetTorque(), 0.24816);

  panther_hardware_interfaces::RoboteqDriverState state;

  state.fault_flags = 0b00000001;
  state.script_flags = 0b00000010;
  state.runtime_stat_flag_motor_1 = 0b00010000;
  state.runtime_stat_flag_motor_2 = 0b00000100;

  state.mcu_temp = 32;
  state.heatsink_temp = 31;
  state.battery_voltage = 365;
  state.battery_current_1 = 15;
  state.battery_current_2 = 20;

  roboteq_data.SetDriverState(state, false);

  ASSERT_TRUE(roboteq_data.IsError());

  ASSERT_TRUE(roboteq_data.GetFaultFlag().GetMessage().overheat);
  ASSERT_TRUE(roboteq_data.GetScriptFlag().GetMessage().encoder_disconnected);
  ASSERT_TRUE(roboteq_data.GetLeftRuntimeError().GetMessage().loop_error);
  ASSERT_TRUE(roboteq_data.GetRightRuntimeError().GetMessage().forward_limit_triggered);

  ASSERT_FLOAT_EQ(roboteq_data.GetDriverState().GetTemperature(), 32);
  ASSERT_FLOAT_EQ(roboteq_data.GetDriverState().GetHeatsinkTemperature(), 31);
  ASSERT_FLOAT_EQ(roboteq_data.GetDriverState().GetVoltage(), 36.5);
  ASSERT_FLOAT_EQ(roboteq_data.GetDriverState().GetCurrent(), 3.5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
