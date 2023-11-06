#include <string>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/utils.hpp>

TEST(TestUtils, test_get_byte)
{
  using panther_hardware_interfaces::GetByte;

  ASSERT_EQ(GetByte(0xFA3B4186, 0), 0x86);
  ASSERT_EQ(GetByte(0xFA3B4186, 1), 0x41);
  ASSERT_EQ(GetByte(0xFA3B4186, 2), 0x3B);
  ASSERT_EQ(GetByte(0xFA3B4186, 3), 0xFA);
}

TEST(TestUtils, test_get_byte_out_of_range)
{
  using panther_hardware_interfaces::GetByte;

  ASSERT_THROW(GetByte(0xFA3B4186, 4), std::runtime_error);
  ASSERT_THROW(GetByte(0xFA3B4186, -1), std::runtime_error);
}

TEST(TestUtils, test_bit_set)
{
  using panther_hardware_interfaces::IsBitSet;

  ASSERT_EQ(IsBitSet(0b01101001, 0), true);
  ASSERT_EQ(IsBitSet(0b01101001, 1), false);
  ASSERT_EQ(IsBitSet(0b01101001, 2), false);
  ASSERT_EQ(IsBitSet(0b01101001, 3), true);
  ASSERT_EQ(IsBitSet(0b01101001, 4), false);
  ASSERT_EQ(IsBitSet(0b01101001, 5), true);
  ASSERT_EQ(IsBitSet(0b01101001, 6), true);
  ASSERT_EQ(IsBitSet(0b01101001, 7), false);
}

TEST(TestUtils, test_bit_set_out_of_range)
{
  using panther_hardware_interfaces::IsBitSet;

  ASSERT_THROW(IsBitSet(0b01101001, 8), std::runtime_error);
  ASSERT_THROW(IsBitSet(0b01101001, -1), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}