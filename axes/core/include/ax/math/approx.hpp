#pragma once
#include "ax/core/config.hpp"

namespace ax::math {

template <typename Scalar> struct Approx {
  AX_FORCE_INLINE Approx(Scalar value, Scalar epsilon) : value_(value), epsilon_(epsilon) {}
  const Scalar value_;
  const Scalar epsilon_;
};

template <typename Scalar> bool operator==(Scalar const& lhs, Approx<Scalar> const& rhs) {
  return std::abs(lhs - rhs.value_) < rhs.epsilon_;
}

template <typename Scalar> bool operator==(Approx<Scalar> const& lhs, Scalar const& rhs) {
  return std::abs(lhs.value_ - rhs) < lhs.epsilon_;
}

template <typename Scalar> bool operator!=(Scalar const& lhs, Approx<Scalar> const& rhs) {
  return !(lhs == rhs);
}

template <typename Scalar> bool operator!=(Approx<Scalar> const& lhs, Scalar const& rhs) {
  return !(lhs == rhs);
}

template <typename Scalar = real, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Approx<Scalar> approx(Scalar value, Scalar tol = math::epsilon<Scalar>) {
  return Approx<Scalar>(value, tol);
}
}  // namespace ax::math