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

#include <sstream>
#include <system_error>
#include <utility>

#include <gtest/gtest.h>

#include <catch2/catch.hpp>
#include "gpiod.hpp"
#include "gpiosim.h"
#include "gpiosim.hpp"

#include "panther_gpiod/gpio_driver.hpp"

TEST(TestGPIODBindings, TestLibPermission)
{
  auto sim = gpiosim::make_sim()
               .set_num_lines(8)
               .set_line_name(0, "foo")
               .set_line_name(2, "bar")
               .set_line_name(3, "baz")
               .set_line_name(5, "xyz")
               .build();

  const ::gpiod::line::offset line_offset = 3;

  auto request = ::gpiod::chip(sim.dev_path())
                   .prepare_request()
                   .set_consumer("get-line-value")
                   .add_line_settings(
                     line_offset,
                     ::gpiod::line_settings().set_direction(::gpiod::line::direction::INPUT))
                   .do_request();

  ::std::cout << line_offset << "="
              << (request.get_value(line_offset) == ::gpiod::line::value::ACTIVE ? "Active"
                                                                                 : "Inactive")
              << ::std::endl;

  request.set_value(line_offset, gpiod::line::value::ACTIVE);

  ::std::cout << line_offset << "="
              << (request.get_value(line_offset) == ::gpiod::line::value::ACTIVE ? "Active"
                                                                                 : "Inactive")
              << ::std::endl;

  EXPECT_FALSE(false);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto run_tests = RUN_ALL_TESTS();

  return run_tests;
}
