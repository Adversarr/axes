#pragma once
#include <random>
namespace axes {
// Random Number Generator, only support for integers.
template <typename Integer,
          typename = std::enable_if_t<std::is_integral_v<Integer>>>
class Random {
public:
  explicit Random(Integer low = std::numeric_limits<Integer>::min(),
                  Integer high = std::numeric_limits<Integer>::max())
      : gen_(device_), dist_(low, high) {}

  inline Integer Next() noexcept { return dist_(gen_); }

  inline Integer operator()() noexcept { return Next(); }

private:
  // Distribution
  std::uniform_int_distribution<Integer> dist_;
  std::random_device device_;
  std::mt19937 gen_;
};
}  // namespace axes
