#pragma once
#include "ax/core/common.hpp"

namespace ax {

/**
 * @brief A class to represent the dimension of a block.
 *
 * @tparam N The largest dimension of the block, N <= 3 is supported.
 */
template <size_t N>
struct Dim;

template <>
struct Dim<1> {
  size_t sizes_[1];

  AX_HOST_DEVICE AX_HOST_DEVICE AX_CONSTEXPR /* NOLINT: google-explicit-constructor*/ Dim(size_t x)
      : sizes_{x} {}

  AX_HOST_DEVICE AX_FORCE_INLINE const auto& operator*() const noexcept {
    return sizes_;
  }

  AX_HOST_DEVICE AX_FORCE_INLINE auto& operator*() noexcept {
    return sizes_;
  }

  AX_HOST_DEVICE bool operator==(const Dim<1>& other) const noexcept {
    return sizes_[0] == other.sizes_[0];
  }

  AX_HOST_DEVICE AX_CONSTEXPR size_t& X() noexcept {
    return sizes_[0];
  }

  AX_HOST_DEVICE AX_CONSTEXPR const size_t& X() const noexcept {
    return sizes_[0];
  }
};

template <>
struct Dim<2> {
  size_t sizes_[2];

  AX_HOST_DEVICE AX_HOST_DEVICE AX_CONSTEXPR Dim(size_t x, size_t y) : sizes_{x, y} {}

  AX_HOST_DEVICE AX_FORCE_INLINE const auto& operator*() const noexcept {
    return sizes_;
  }

  AX_HOST_DEVICE AX_FORCE_INLINE auto& operator*() noexcept {
    return sizes_;
  }

  AX_HOST_DEVICE bool operator==(const Dim<2>& other) const noexcept {
    return sizes_[0] == other.sizes_[0] && sizes_[1] == other.sizes_[1];
  }

  AX_HOST_DEVICE AX_CONSTEXPR size_t& X() noexcept {
    return sizes_[0];
  }

  AX_HOST_DEVICE AX_CONSTEXPR const size_t& X() const noexcept {
    return sizes_[0];
  }

  AX_HOST_DEVICE AX_CONSTEXPR size_t& Y() noexcept {
    return sizes_[1];
  }

  AX_HOST_DEVICE AX_CONSTEXPR const size_t& Y() const noexcept {
    return sizes_[1];
  }
};

template <>
struct Dim<3> {
  size_t sizes_[3];

  AX_HOST_DEVICE AX_CONSTEXPR /* NOLINT: google-explicit-constructor*/ Dim(size_t x)
      : sizes_{x, 0, 0} {}

  AX_HOST_DEVICE AX_CONSTEXPR Dim(size_t x, size_t y) : sizes_{x, y, 0} {}

  AX_HOST_DEVICE AX_CONSTEXPR Dim(size_t x, size_t y, size_t z) : sizes_{x, y, z} {}

  AX_HOST_DEVICE AX_FORCE_INLINE const auto& operator*() const noexcept {
    return sizes_;
  }

  AX_HOST_DEVICE AX_FORCE_INLINE auto& operator*() noexcept {
    return sizes_;
  }

  AX_HOST_DEVICE bool operator==(const Dim<3>& other) const noexcept {
    return sizes_[0] == other.sizes_[0] && sizes_[1] == other.sizes_[1]
           && sizes_[2] == other.sizes_[2];
  }

  AX_HOST_DEVICE AX_CONSTEXPR size_t& X() noexcept {
    return sizes_[0];
  }

  AX_HOST_DEVICE AX_CONSTEXPR const size_t& X() const noexcept {
    return sizes_[0];
  }

  AX_HOST_DEVICE AX_CONSTEXPR size_t& Y() noexcept {
    return sizes_[1];
  }

  AX_HOST_DEVICE AX_CONSTEXPR const size_t& Y() const noexcept {
    return sizes_[1];
  }

  AX_HOST_DEVICE AX_CONSTEXPR size_t& Z() noexcept {
    return sizes_[2];
  }

  AX_HOST_DEVICE AX_CONSTEXPR const size_t& Z() const noexcept {
    return sizes_[2];
  }
};

Dim(size_t) -> Dim<1>;
Dim(size_t, size_t) -> Dim<2>;
Dim(size_t, size_t, size_t) -> Dim<3>;

using Dim1 = Dim<1>;
using Dim2 = Dim<2>;
using Dim3 = Dim<3>;

namespace details {
template <size_t N>
AX_HOST_DEVICE AX_CONSTEXPR size_t dot_prod(const Dim<N>& a, const Dim<N>& b) {
  size_t result = 0;
  for (size_t i = 0; i < N; ++i) {
    result += a.sizes_[i] * b.sizes_[i];
  }
  return result;
}

template <size_t N>
AX_HOST_DEVICE AX_CONSTEXPR size_t linear_index(const Dim<N>& dim, const Dim<N>& index) {
  size_t result = 0;
  // In z-y-x order.
  for (size_t i = 0; i < N; ++i) {
    result = result * dim.sizes_[N - 1 - i] + index.sizes_[N - 1 - i];
  }
  return result;
}

AX_HOST_DEVICE AX_CONSTEXPR bool is_1d(const Dim<1>& /* dim */) {
  return true;
}

AX_HOST_DEVICE AX_CONSTEXPR bool is_1d(const Dim<2>& dim) {
  return dim.sizes_[1] == 0;
}

AX_HOST_DEVICE AX_CONSTEXPR bool is_1d(const Dim<3>& dim) {
  return dim.sizes_[1] == 0 && dim.sizes_[2] == 0;
}

AX_HOST_DEVICE AX_CONSTEXPR bool is_2d(const Dim<1>& /* dim */) {
  return false;
}

AX_HOST_DEVICE AX_CONSTEXPR bool is_2d(const Dim<2>& dim) {
  return dim.sizes_[1] != 0;
}

AX_HOST_DEVICE AX_CONSTEXPR bool is_2d(const Dim<3>& dim) {
  return dim.sizes_[2] == 0 && dim.sizes_[1] != 0;
}

AX_HOST_DEVICE AX_CONSTEXPR bool is_3d(const Dim<1>& /* dim */) {
  return false;
}

AX_HOST_DEVICE AX_CONSTEXPR bool is_3d(const Dim<2>& /* dim */) {
  return false;
}

AX_HOST_DEVICE AX_CONSTEXPR bool is_3d(const Dim<3>& dim) {
  return dim.sizes_[2] != 0;
}

AX_HOST_DEVICE AX_CONSTEXPR size_t prod(const Dim<1>& dim) {
  return dim.sizes_[0];
}

AX_HOST_DEVICE AX_CONSTEXPR size_t prod(const Dim<2>& dim) {
  return dim.sizes_[0] * (dim.sizes_[1] > 0 ? dim.sizes_[1] : 1);
}

AX_HOST_DEVICE AX_CONSTEXPR size_t prod(const Dim<3>& dim) {
  return dim.sizes_[0] * (dim.sizes_[1] > 0 ? dim.sizes_[1] : 1)
         * (dim.sizes_[2] > 0 ? dim.sizes_[2] : 1);
}

}  // namespace details

template <size_t N>
AX_HOST_DEVICE AX_CONSTEXPR size_t prod(const Dim<N>& dim) {
  return details::prod(dim);
}

template <size_t N>
AX_HOST_DEVICE AX_CONSTEXPR size_t dot_prod(const Dim<N>& a, const Dim<N>& b) {
  return details::dot_prod(a, b);
}

template <size_t N>
AX_HOST_DEVICE AX_CONSTEXPR size_t linear_index(const Dim<N>& dim, const Dim<N>& index) {
  return details::linear_index(dim, index);
}

template <size_t N>
AX_HOST_DEVICE AX_CONSTEXPR bool is_1d(const Dim<N>& dim) {
  return details::is_1d(dim);
}

template <size_t N>
AX_HOST_DEVICE AX_CONSTEXPR bool is_2d(const Dim<N>& dim) {
  return details::is_2d(dim);
}

template <size_t N>
AX_HOST_DEVICE AX_CONSTEXPR bool is_3d(const Dim<N>& dim) {
  return details::is_3d(dim);
}

template <typename Fn>
AX_HOST_DEVICE AX_CONSTEXPR void for_each_indexed(const Dim<1>& d, Fn&& f) {
  for (size_t i = 0; i < d.X(); ++i) {
    f(i);
  }
}

template <typename Fn>
AX_HOST_DEVICE AX_CONSTEXPR void for_each_indexed(const Dim<2>& d, Fn&& f) {
  for (size_t j = 0; j < d.Y(); ++j) {
    for (size_t i = 0; i < d.X(); ++i) {
      f(i, j);
    }
  }
}

template <typename Fn>
AX_HOST_DEVICE AX_CONSTEXPR void for_each_indexed(const Dim<3>& d, Fn&& f) {
  for (size_t k = 0; k < d.Z(); ++k) {
    for (size_t j = 0; j < d.Y(); ++j) {
      for (size_t i = 0; i < d.X(); ++i) {
        f(i, j, k);
      }
    }
  }
}

}  // namespace ax
