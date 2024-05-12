
#pragma once
#include "ax/math/common.hpp"

namespace ax::math::decomp {

template <idx dim, typename Scalar = real> struct SvdResult {
  using scalar_t = Scalar;
  using vec_t = vecr<dim>;
  using mat_t = matr<dim, dim>;

  vec_t sigma_;
  mat_t U_;
  mat_t V_;

  SvdResult(mat_t U, vec_t sigma, mat_t V) : sigma_(sigma), U_(U), V_(V) {}
  SvdResult() = default;
};

}  // namespace ax::math
