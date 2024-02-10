#pragma once
#include "axes/math/functional.hpp"
#include "axes/geometry/common.hpp"

namespace ax::math {

/**
 * @brief Computes the integral of the multivariate polynomial over the standard simplex:
 *        Integrates: x_1^a_1 * x_2^a_2 * ... * x_n^a_n ]
 *        n: {x_1 + ... + x_n <= 1, x_i >= 0}
 * @tparam dim 
 * @param x 
 * @return real 
 */
template <idx dim> real pint_on_standard_simplex(math::vecr<dim> const& x) {
  math::vecr<dim> gamma_1_d;
  for (idx i = 0; i < dim; ++i) {
    gamma_1_d[i] = tgamma(1 + x[i]);
  }
  return math::prod(gamma_1_d) / tgamma(1 + dim + math::sum(x));
}

/**
 * @brief Computes the integral of the multivariate polynomial over the standard simplex:
 *        Integrates: x_1^a_1 * x_2^a_2 * ... * x_n^a_n ]
 *        n: {x_1 + ... + x_n <= 1, x_i >= 0}
 * @tparam dim 
 * @param a 
 * @return real 
 */
template <idx dim> real pint_on_standard_simplex_fast(math::veci<dim> const& a) {
  math::vecr<dim> gamma_1_d;
  for (idx i = 0; i < dim; ++i) {
    gamma_1_d[i] = details::factorials[1 + a[i]];
  }
  real det = details::factorials[1 + dim + math::sum(a)];
  return math::prod(gamma_1_d) / det;
}

template<idx dim>
struct IntegrateSimplex {
  IntegrateSimplex(geo::SimplexN<dim> const& spx) {
    for (idx i = 1; i <= dim; ++i) {
      jacobi_.col(i - 1) = spx[i] - spx[0];
    }
  }

  real Integrate(math::veci<dim> const& a) const {
    real det = jacobi_.determinant();
    return pint_on_standard_simplex_fast(a) * det;
  }

  math::matr<dim, dim> jacobi_;
};

}  // namespace ax::math
