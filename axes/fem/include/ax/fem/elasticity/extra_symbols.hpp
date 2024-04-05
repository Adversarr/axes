#pragma once
#include "ax/math/linalg.hpp"
#include "common.hpp"

namespace ax::fem::elasticity {

template<typename Derived>
auto green_strain(math::MBcr<Derived> F) {
  return 0.5 * (F.transpose() * F - math::eye<math::rows_v<Derived>>());
}

template <typename Derived>
auto approx_green_strain(math::MBcr<Derived> F) {
  return 0.5 * (F.transpose() + F) - math::eye<math::rows_v<Derived>>();
}

namespace details {
AX_FORCE_INLINE math::matr<2, 2> partial_determinant(DeformationGradient<2> const& F) {
  // [ f11, -f10;
  //  -f01,  f00]
  math::matr<2, 2> dJdF;
  dJdF << F(1, 1), -F(0, 1),
          -F(1, 0), F(0, 0);
  return dJdF;
}

AX_FORCE_INLINE math::matr<3, 3> partial_determinant(DeformationGradient<3> const & F) {
  // [Cross[f1, f2], Cross[f2, f0], Cross[f0, f1]]
  math::matr<3, 3> dJdF;
  dJdF << math::cross(F.col(1), F.col(2)),
          math::cross(F.col(2), F.col(0)),
          math::cross(F.col(0), F.col(1));
  return dJdF;
}

AX_FORCE_INLINE void add_HJ(
  math::matr<4, 4>& H,
  const math::matr<2, 2>& /*F*/,
  real scale) {
  H(3, 0) += scale;
  H(0, 3) += scale;
  H(1, 2) -= scale;
  H(2, 1) -= scale;
}

template<typename Derived>
AX_FORCE_INLINE math::mat3r x_hat(math::MBcr<Derived>Fi) {
  math::mat3r x_hat;
  x_hat << 0, -Fi(2), Fi(1),
           Fi(2), 0, -Fi(0),
           -Fi(1), Fi(0), 0;
  return x_hat;
}

AX_FORCE_INLINE void add_HJ(
  math::matr<9, 9>& H,
  const math::matr<3, 3>& F,
  real scale) {
  math::mat3r const f0 = x_hat(F.col(0));
  math::mat3r const f1 = x_hat(F.col(1));
  math::mat3r const f2 = x_hat(F.col(2));
  H.block<3, 3>(3, 0) += f2 * scale;
  H.block<3, 3>(0, 3) -= f2 * scale;
  H.block<3, 3>(6, 0) -= f1 * scale;
  H.block<3, 3>(0, 6) += f1 * scale;
  H.block<3, 3>(6, 3) += f0 * scale;
  H.block<3, 3>(3, 6) -= f0 * scale;
}
} // namespace details
}