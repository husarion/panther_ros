#ifndef PANTHER_UTILS_MOVING_AVERAGE_HPP_
#define PANTHER_UTILS_MOVING_AVERAGE_HPP_

#include <deque>

namespace panther_utils
{

template <typename T>
class MovingAverage
{
public:
  MovingAverage(const unsigned window_size = 5, const T initial_value = T(0))
  : window_size_(window_size), initial_value_(initial_value), sum_(T(0))
  {
  }

  void Roll(const T value)
  {
    values_.push_back(value);
    sum_ += value;

    if (values_.size() > window_size_) {
      sum_ -= values_.front();
      values_.pop_front();
    }
  }

  void Reset() {
    values_.erase(values_.begin(), values_.end());
    sum_ = T(0);
  }

  T GetAverage() const
  {
    if (values_.size() == 0) {
      return initial_value_;
    }
    return sum_ / static_cast<T>(values_.size());
  }

private:
  const unsigned window_size_;
  std::deque<T> values_;
  const T initial_value_;
  T sum_;
};

}  // namespace panther_utils

#endif  // PANTHER_UTILS_MOVING_AVERAGE_HPP_