#ifndef PANTHER_MANAGER_MOVING_AVERAGE_HPP_
#define PANTHER_MANAGER_MOVING_AVERAGE_HPP_

namespace panther_manager
{

template <typename T>
class MovingAverage
{
public:
  MovingAverage(const unsigned window_size = 5, const T initial_value = T(0))
  : window_size_(window_size), sum_(initial_value)
  {
  }

  void roll(const T value)
  {
    if (values.size() >= window_size_) {
      sum_ -= values.front();
      values.pop_front();
    }
    values.push_back(value);
    sum_ += value;
  }

  T get_average() const
  {
    return sum_ / static_cast<T>(window_size_);
  }

private:
  unsigned window_size_;
  std::deque<T> values;
  T sum_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_MOVING_AVERAGE_HPP_