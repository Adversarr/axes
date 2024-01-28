#pragma once

#include "common.hpp"  // IWYU pragma: export

namespace ax::math {

/****************************** Scalar Type For ******************************/

template <typename T> struct scalar_of {
  using type = T::Scalar;  // For Eigen type.
};

template <> struct scalar_of<f32> {
  using type = f32;
};

template <> struct scalar_of<f64> {
  using type = f64;
};

template <typename T> struct ensure_floating {
  static_assert(std::is_same_v<std::remove_cvref_t<T>, f32>
                    || std::is_same_v<std::remove_cvref_t<T>, f64>,
                "The type must be floating point type.");
};
}  // namespace ax::math
