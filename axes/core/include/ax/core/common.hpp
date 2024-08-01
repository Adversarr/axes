#pragma once

#include <memory>  // IWYU pragma: export
#include <vector>  // IWYU pragma: export

#include "ax/core/config.hpp"  // IWYU pragma: export
#include "ax/core/macros.hpp"  // IWYU pragma: export

namespace ax {

template <typename T> AX_FORCE_INLINE AX_HOST_DEVICE AX_CONSTEXPR tpl_size_t to_tpl_size(T &&v) {
  return static_cast<tpl_size_t>(v);
}

template <typename T> AX_FORCE_INLINE AX_HOST_DEVICE AX_CONSTEXPR vec_size_t to_vec_size(T &&v) {
  return static_cast<vec_size_t>(v);
}

template <typename T> AX_FORCE_INLINE AX_HOST_DEVICE AX_CONSTEXPR idx to_std_size(T &&v) {
  return static_cast<size_t>(v);
}

template <typename T> AX_FORCE_INLINE AX_HOST_DEVICE AX_CONSTEXPR real to_real(T &&v) {
  return static_cast<real>(v);
}

}  // namespace ax
