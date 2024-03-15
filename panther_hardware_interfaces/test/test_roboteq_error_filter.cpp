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

#include <panther_hardware_interfaces/roboteq_error_filter.hpp>

TEST(TestRoboteqErrorFilter, test_initial_state)
{
  using panther_hardware_interfaces::ErrorFilter;
  using panther_hardware_interfaces::ErrorsFilterIds;

  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, test_filter_error)
{
  using panther_hardware_interfaces::ErrorFilter;
  using panther_hardware_interfaces::ErrorsFilterIds;

  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, false);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, test_error)
{
  using panther_hardware_interfaces::ErrorFilter;
  using panther_hardware_interfaces::ErrorsFilterIds;

  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  ASSERT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, test_filter_second_error)
{
  using panther_hardware_interfaces::ErrorFilter;
  using panther_hardware_interfaces::ErrorsFilterIds;

  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, false);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, test_second_error)
{
  using panther_hardware_interfaces::ErrorFilter;
  using panther_hardware_interfaces::ErrorsFilterIds;

  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, test_error_single)
{
  using panther_hardware_interfaces::ErrorFilter;
  using panther_hardware_interfaces::ErrorsFilterIds;

  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, test_clear_errors)
{
  using panther_hardware_interfaces::ErrorFilter;
  using panther_hardware_interfaces::ErrorsFilterIds;

  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, true);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  ASSERT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.SetClearErrorsFlag();
  // Has to trigger at least one update to clear errors
  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, test_clear_errors_counters)
{
  using panther_hardware_interfaces::ErrorFilter;
  using panther_hardware_interfaces::ErrorsFilterIds;

  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.SetClearErrorsFlag();

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  ASSERT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
