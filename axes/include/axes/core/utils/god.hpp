#pragma once
#include <tuple>

namespace axes::utils {

template <typename... Args> constexpr auto product(Args&&... arg) {
  return (arg * ...);
}

template <typename... Args> constexpr auto sum(Args&&... arg) {
  return (arg + ...);
}

template <typename T, int alpha> constexpr T pow(T x) {
  if constexpr (alpha == 0) {
    return static_cast<T>(1);
  } else {
    return x * pow<T, alpha - 1>(x);
  }
}
}  // namespace axes::utils
