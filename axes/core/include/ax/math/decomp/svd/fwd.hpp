
#pragma once
#include "ax/math/common.hpp"

namespace ax::math {

template <idx dim, typename Scalar = real> struct SvdResultImpl {
  using scalar_t = Scalar;
  using vec_t = vecr<dim>;
  using mat_t = matr<dim, dim>;

  vec_t sigma_;
  mat_t U_;
  mat_t V_;

  SvdResultImpl(mat_t U, vec_t sigma, mat_t V) : sigma_(sigma), U_(U), V_(V) {}
  SvdResultImpl() = default;
};

template <idx dim, typename Scalar = real> using SvdResult = StatusOr<SvdResultImpl<dim, Scalar>>;

}  // namespace ax::math
