#ifndef PANTHER_MANAGER_MOVING_AVERAGE_HPP_
#define PANTHER_MANAGER_MOVING_AVERAGE_HPP_

#include <deque>

namespace panther_manager
{

template <typename T>
class MovingAverage
{
public:
  MovingAverage(const unsigned window_size = 5, const T initial_value = T(0))
  : window_size_(window_size),
    sum_(initial_value * static_cast<T>(window_size_)),
    values_(window_size, initial_value)
  {
  }

  void roll(const T value)
  {
    sum_ -= values_.front();
    values_.pop_front();
    values_.push_back(value);
    sum_ += value;
  }

  T get_average() const { return sum_ / static_cast<T>(window_size_); }

private:
  const unsigned window_size_;
  std::deque<T> values_;
  T sum_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_MOVING_AVERAGE_HPP_