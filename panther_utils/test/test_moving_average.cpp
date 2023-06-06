#include <gtest/gtest.h>

#include <panther_utils/moving_average.hpp>

TEST(TestMovingAverage, TestOutputValues)
{
  panther_utils::MovingAverage<double> ma(4);
  EXPECT_EQ(0.0, ma.GetAverage());

  ma.Roll(1.0);
  EXPECT_EQ(1.0, ma.GetAverage());

  ma.Roll(2.0);
  EXPECT_EQ(1.5, ma.GetAverage());

  ma.Roll(3.0);
  ma.Roll(4.0);
  EXPECT_EQ(2.5, ma.GetAverage());

  ma.Roll(5.0);
  ma.Roll(5.0);
  ma.Roll(5.0);
  ma.Roll(5.0);
  EXPECT_EQ(5.0, ma.GetAverage());
}

TEST(TestMovingAverage, TestHighOverload)
{
  panther_utils::MovingAverage<double> ma(1000);

  double sum;
  for (std::size_t i = 1; i <= 10000; i++) {
    sum += double(i);
    ma.Roll(double(i));

    // test every 1000 rolls expected average
    if (i % 1000 == 0) {
      EXPECT_EQ(sum / 1000, ma.GetAverage());
      sum = 0.0;
    }
  }
}

TEST(TestMovingAverage, TestIntialValue)
{
  panther_utils::MovingAverage<float> ma(4, 1.0);
  EXPECT_EQ(1.0, ma.GetAverage());

  panther_utils::MovingAverage<double> ma_1(10, 3.7);
  EXPECT_EQ(3.7, ma_1.GetAverage());

  panther_utils::MovingAverage<int> ma_2(3, 4);
  EXPECT_EQ(4, ma_2.GetAverage());
}

TEST(TestMovingAverage, TestIntType)
{
  panther_utils::MovingAverage<int> ma;
  ma.Roll(1);
  ma.Roll(2);
  EXPECT_EQ(1, ma.GetAverage());
}

TEST(TestMovingAverage, TestReset)
{
  panther_utils::MovingAverage<double> ma(4);
  ma.Roll(1.0);
  ma.Roll(2.0);
  ma.Roll(1.0);
  ma.Roll(2.0);
  EXPECT_EQ(1.5, ma.GetAverage());

  ma.Reset();
  EXPECT_EQ(0.0, ma.GetAverage());

  ma.Roll(2.0);
  ma.Roll(4.0);
  EXPECT_EQ(3.0, ma.GetAverage());

  panther_utils::MovingAverage<double> ma_1(4, 7.0);
  ma_1.Roll(1.0);
  ma_1.Roll(2.0);
  ma_1.Reset();
  EXPECT_EQ(7.0, ma_1.GetAverage());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}