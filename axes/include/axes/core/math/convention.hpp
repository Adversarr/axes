#pragma once
#include <array>

#include "axes/core/math/common.hpp"

namespace axes::math {

namespace details {

template <typename Vec, std::size_t... I>
std::array<typename Vec::Scalar, Vec::ColsAtCompileTime> make_array_impl(
    const Vec& v, std::index_sequence<I...>) {
  return {{v[I]...}};
}

}  // namespace details

/**
 * @brief Create an stl array from a compile-time shaped vector.
 *
 * @tparam Vec
 * @param v
 * @return
 */
template <typename Vec>
std::array<typename Vec::Scalar, get_cols<Vec>()> make_array(const Vec& v) {
  static_assert(
      get_cols<Vec>() > 0,
      "You cannot create an array from vector without Compile Time Shape");
  return details::make_array_impl(
      v, std::make_index_sequence<Vec::ColsAtCompileTime>());
}

}  // namespace axes::math
