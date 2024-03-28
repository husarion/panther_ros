// Copyright 2024 Husarion sp. z o.o.
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

#include <map>
#include <string>

#include "gtest/gtest.h"

#include "panther_utils/common_utilities.hpp"

TEST(TestPrefixMapKeys, CorrectlyPrefixesKeys)
{
  std::map<std::string, int> input_map = {{"key1", 1}, {"key2", 2}};
  std::string prefix = "prefix_";
  std::map<std::string, int> expected_map = {{"prefix_key1", 1}, {"prefix_key2", 2}};

  auto result_map = panther_utils::common_utilities::PrefixMapKeys(input_map, prefix);

  EXPECT_EQ(result_map, expected_map);
}

TEST(TestPrefixMapKeys, HandlesEmptyPrefix)
{
  std::map<std::string, int> input_map = {{"key1", 1}, {"key2", 2}};
  std::string prefix = "";
  std::map<std::string, int> expected_map = input_map;  // The maps should be identical

  auto result_map = panther_utils::common_utilities::PrefixMapKeys(input_map, prefix);

  EXPECT_EQ(result_map, expected_map);
}

TEST(PrefixMapKeysTest, HandlesEmptyMap)
{
  std::map<std::string, int> input_map;
  std::string prefix = "prefix_";
  std::map<std::string, int> expected_map;

  auto result_map = panther_utils::common_utilities::PrefixMapKeys(input_map, prefix);

  EXPECT_EQ(result_map, expected_map);
}

TEST(PrefixMapKeysTest, HandlesNonAlphanumericPrefix)
{
  std::map<std::string, int> input_map = {{"key1", 1}, {"key2", 2}};
  std::string prefix = "prefix_@#";
  std::map<std::string, int> expected_map = {{"prefix_@#key1", 1}, {"prefix_@#key2", 2}};

  auto result_map = panther_utils::common_utilities::PrefixMapKeys(input_map, prefix);

  EXPECT_EQ(result_map, expected_map);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
