#ifndef PANTHER_MANAGER_MOVING_AVERAGE_HPP_
#define PANTHER_MANAGER_MOVING_AVERAGE_HPP_

namespace panther_manager
{

template <typename T>
class MovingAverage
{
public:
  MovingAverage(const int window_size = 5) : window_size_(window_size), sum_(T(0)) {}

  void add_value(const T value)
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
    if (values.size() == 0) {
      return T(0);
    }
    return sum_ / static_cast<T>(values.size());
  }

private:
  int window_size_;
  std::deque<T> values;
  T sum_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_MOVING_AVERAGE_HPP_