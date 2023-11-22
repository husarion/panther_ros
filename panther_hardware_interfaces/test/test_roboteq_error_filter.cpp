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
  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(
    std::vector<ErrorFilter>{ErrorFilter(2), ErrorFilter(2)});

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));
}

TEST(TestRoboteqErrorFilter, test_filter_error)
{
  using panther_hardware_interfaces::ErrorFilter;
  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(
    std::vector<ErrorFilter>{ErrorFilter(2), ErrorFilter(2)});

  roboteq_error_filter.UpdateError(0, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));

  roboteq_error_filter.UpdateError(0, false);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));

  roboteq_error_filter.UpdateError(0, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));
}

TEST(TestRoboteqErrorFilter, test_error)
{
  using panther_hardware_interfaces::ErrorFilter;
  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(
    std::vector<ErrorFilter>{ErrorFilter(2), ErrorFilter(2)});

  roboteq_error_filter.UpdateError(0, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));

  roboteq_error_filter.UpdateError(0, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  ASSERT_TRUE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));
}

TEST(TestRoboteqErrorFilter, test_filter_second_error)
{
  using panther_hardware_interfaces::ErrorFilter;
  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(
    std::vector<ErrorFilter>{ErrorFilter(2), ErrorFilter(2)});

  roboteq_error_filter.UpdateError(1, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));

  roboteq_error_filter.UpdateError(1, false);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));

  roboteq_error_filter.UpdateError(1, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));
}

TEST(TestRoboteqErrorFilter, test_second_error)
{
  using panther_hardware_interfaces::ErrorFilter;
  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(
    std::vector<ErrorFilter>{ErrorFilter(2), ErrorFilter(2)});

  roboteq_error_filter.UpdateError(1, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));

  roboteq_error_filter.UpdateError(1, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_TRUE(roboteq_error_filter.IsError(1));
}

TEST(TestRoboteqErrorFilter, test_error_single)
{
  using panther_hardware_interfaces::ErrorFilter;
  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(
    std::vector<ErrorFilter>{ErrorFilter(2), ErrorFilter(2), ErrorFilter(1)});

  roboteq_error_filter.UpdateError(2, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));
  ASSERT_TRUE(roboteq_error_filter.IsError(2));
}

TEST(TestRoboteqErrorFilter, test_clear_errors)
{
  using panther_hardware_interfaces::ErrorFilter;
  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(
    std::vector<ErrorFilter>{ErrorFilter(2), ErrorFilter(2), ErrorFilter(1)});

  roboteq_error_filter.UpdateError(0, true);
  roboteq_error_filter.UpdateError(0, true);

  roboteq_error_filter.UpdateError(1, true);
  roboteq_error_filter.UpdateError(1, true);

  roboteq_error_filter.UpdateError(2, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  ASSERT_TRUE(roboteq_error_filter.IsError(0));
  ASSERT_TRUE(roboteq_error_filter.IsError(1));
  ASSERT_TRUE(roboteq_error_filter.IsError(2));

  roboteq_error_filter.SetClearErrorsFlag();
  // Has to trigger at least one update to clear errors
  roboteq_error_filter.UpdateError(0, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));
  ASSERT_FALSE(roboteq_error_filter.IsError(2));
}

TEST(TestRoboteqErrorFilter, test_clear_errors_counters)
{
  using panther_hardware_interfaces::ErrorFilter;
  panther_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(
    std::vector<ErrorFilter>{ErrorFilter(2), ErrorFilter(2)});

  roboteq_error_filter.UpdateError(0, true);
  roboteq_error_filter.UpdateError(1, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));

  roboteq_error_filter.SetClearErrorsFlag();

  roboteq_error_filter.UpdateError(0, true);
  roboteq_error_filter.UpdateError(1, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  ASSERT_FALSE(roboteq_error_filter.IsError(0));
  ASSERT_FALSE(roboteq_error_filter.IsError(1));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
