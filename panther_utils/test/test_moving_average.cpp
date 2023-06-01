#include <gtest/gtest.h>

#include <panther_utils/moving_average.hpp>

TEST(TestMovingAverage, TestOutputValues)
{
  std::unique_ptr<panther_utils::MovingAverage<double>> ma;
  ma = std::make_unique<panther_utils::MovingAverage<double>>(4);
  EXPECT_EQ(0.0, ma->GetAverage());

  ma->Roll(1.0);
  EXPECT_EQ(1.0, ma->GetAverage());

  ma->Roll(2.0);
  EXPECT_EQ(1.5, ma->GetAverage());

  ma->Roll(3.0);
  ma->Roll(4.0);
  EXPECT_EQ(2.5, ma->GetAverage());

  ma->Roll(5.0);
  ma->Roll(5.0);
  ma->Roll(5.0);
  ma->Roll(5.0);
  EXPECT_EQ(5.0, ma->GetAverage());
}

TEST(TestMovingAverage, TestIntialValue)
{
  std::unique_ptr<panther_utils::MovingAverage<float>> ma;
  ma = std::make_unique<panther_utils::MovingAverage<float>>(4, 1.0);
  EXPECT_EQ(1.0, ma->GetAverage());
}

TEST(TestMovingAverage, TestIntType)
{
  std::unique_ptr<panther_utils::MovingAverage<int>> ma;
  ma = std::make_unique<panther_utils::MovingAverage<int>>();
  ma->Roll(1);
  ma->Roll(2);
  EXPECT_EQ(1, ma->GetAverage());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}