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

}