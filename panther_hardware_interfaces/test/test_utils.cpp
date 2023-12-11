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

#include <bitset>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/utils.hpp>

TEST(TestUtils, test_get_byte)
{
  using panther_hardware_interfaces::GetByte;

  ASSERT_EQ(GetByte(static_cast<std::int32_t>(0xFA3B4186), 0), 0x86);
  ASSERT_EQ(GetByte(static_cast<std::int32_t>(0xFA3B4186), 1), 0x41);
  ASSERT_EQ(GetByte(static_cast<std::int32_t>(0xFA3B4186), 2), 0x3B);
  ASSERT_EQ(GetByte(static_cast<std::int32_t>(0xFA3B4186), 3), 0xFA);
}

TEST(TestUtils, test_get_byte_out_of_range)
{
  using panther_hardware_interfaces::GetByte;

  ASSERT_THROW(GetByte(static_cast<std::int32_t>(0xFA3B4186), 4), std::runtime_error);
  ASSERT_THROW(GetByte(static_cast<std::int32_t>(0xFA3B4186), -1), std::runtime_error);
}

TEST(TestUtils, operation_with_attempts_fail_test)
{
  unsigned max_attempts = 5;
  unsigned attempts_counter = 0;
  unsigned on_error_counter = 0;

  ASSERT_FALSE(panther_hardware_interfaces::OperationWithAttempts(
    [&attempts_counter]() {
      ++attempts_counter;
      throw std::runtime_error("");
    },
    max_attempts, [&on_error_counter]() { ++on_error_counter; }));
  ASSERT_EQ(attempts_counter, max_attempts);
  ASSERT_EQ(on_error_counter, max_attempts);
}

TEST(TestUtils, operation_with_attempts_success_test)
{
  unsigned max_attempts = 5;
  unsigned attempts_counter = 0;

  ASSERT_TRUE(panther_hardware_interfaces::OperationWithAttempts(
    [&attempts_counter, &max_attempts]() {
      ++attempts_counter;
      if (attempts_counter < max_attempts) {
        throw std::runtime_error("");
      }
    },
    max_attempts, []() {}));
  ASSERT_EQ(attempts_counter, max_attempts);
}

TEST(TestUtils, operation_with_attempts_on_error_throw_test)
{
  ASSERT_FALSE(panther_hardware_interfaces::OperationWithAttempts(
    []() { throw std::runtime_error(""); }, 5, []() { throw std::runtime_error(""); }));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
