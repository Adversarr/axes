
#pragma once
#include "ax/math/common.hpp"

namespace ax::math::decomp {

template <int dim, typename Scalar = Real> struct SvdResult {
  using scalar_t = Scalar;
  using vec_t = RealVector<dim>;
  using mat_t = RealMatrix<dim, dim>;

  vec_t sigma_;
  mat_t U_;
  mat_t V_;

  SvdResult(mat_t U, vec_t sigma, mat_t V) : sigma_(sigma), U_(U), V_(V) {}
  SvdResult() = default;
};

}  // namespace ax::math
