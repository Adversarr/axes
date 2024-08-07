#pragma once
#include "ax/math/common.hpp"

namespace ax::math {

template <typename Scalar> class Approx final {
public:
  explicit Approx(Scalar value) : value_(value), epsilon_(math::epsilon<Scalar>) {}

  Approx(Scalar value, Scalar epsilon) : value_(value), epsilon_(epsilon) {}

  Approx& Epsilon(Scalar const& eps) {
    epsilon_ = eps;
    return *this;
  }

  Scalar value_;
  const Scalar epsilon_;
};

template <typename Scalar> Approx(Scalar) -> Approx<Scalar>;
template <typename Scalar> Approx(Scalar, Scalar) -> Approx<Scalar>;

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

template <typename Scalar = real>
Approx<Scalar> approx(Scalar value, Scalar tol = math::epsilon<Scalar>) {
  return Approx<Scalar>(value, tol);
}
}  // namespace ax::math