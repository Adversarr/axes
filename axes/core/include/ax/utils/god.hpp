#pragma once
namespace ax::utils::god {

template <typename T>
constexpr T pow(T x, int n) {
  T output = 1;
  for (int i = 0; i < n; ++i) {
    output *= x;
  }
  return output;
}

template <typename Front, typename... Args>
AX_HOST_DEVICE AX_CONSTEXPR bool all_equal(Front&& front, Args&&... args) noexcept {
  return (sizeof...(args) == 0) || ((front == args) && ...);
}

}