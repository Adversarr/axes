#pragma once
#include "axes/utils/god.hpp"

namespace ax::pde::elasticity {

// convert Young's modulus (E) and Poisson's ratio (nu) to Lamé parameters
real compute_mu(real E, real nu);

real compute_lambda(real E, real nu);

/**
 * @brief Compute the deformation gradient F = (X_curr) * (X_rest).inv
 * 
 * @tparam dim 
 * @param position 
 * @param rest_position 
 * @return math::matr<dim, dim> 
 */
template<idx dim>
math::matr<dim, dim> compute_deformation_gradient_p1(
  math::matr<dim, dim+1> const& position,
  math::matr<dim, dim+1> const& rest_position
);

/**
 * @brief Compute the deformation gradient cache, i.e. (X_rest).inv
 * 
 * @tparam dim 
 * @param position 
 * @param rest_position 
 * @return math::matr<dim, dim> 
 */
template<idx dim>
math::matr<dim, dim> compute_deformation_gradient_rest_cache_p1(
  math::matr<dim, dim+1> const& rest_position
);

/**
 * @brief Compute the deformation gradient F = (X_curr H) * (X_rest H).inv
 * 
 * @tparam dim 
 * @param position 
 * @param rest_position 
 * @return math::matr<dim, dim> 
 */
template<idx dim>
math::matr<dim, dim> compute_deformation_gradient_q1(
  math::matr<dim, utils::god::pow(2, dim)> const& position,
  math::matr<dim, utils::god::pow(2, dim)> const& rest_position
);

/**
 * @brief Compute the deformation gradient cache, i.e. (X_rest H).inv
 * 
 * @tparam dim 
 * @param position 
 * @param rest_position 
 * @return math::matr<dim, dim> 
 */
template<idx dim>
math::matr<dim, dim> compute_deformation_gradient_rest_cache_q1(
  math::matr<dim, utils::god::pow(2, dim)> const& rest_position
);

template<typename Derived>
auto green_strain(math::MBcr<Derived> F) {
  return 0.5 * (F.transpose() * F - math::eye<math::rows_static(F)>());
}

template <typename Derived>
auto approx_green_strain(math::MBcr<Derived> F) {
  return 0.5 * (F.transpose() + F) - math::eye<math::rows_static(F)>();
}

namespace details{
AX_FORCE_INLINE math::matr<2, 2> partial_determinant(const math::matr<2, 2>& F) {
  // [ f11, -f10;
  //  -f01,  f00]
  math::matr<2, 2> dJdF;
  dJdF << F(1, 1), -F(0, 1),
          -F(1, 0), F(0, 0);
  return dJdF;
}

AX_FORCE_INLINE math::matr<3, 3> partial_determinant(const math::matr<3, 3>& F) {
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


template <idx dim, typename Derived> class ElasticityBase {
public:
  static constexpr idx dof_cnt = dim * dim;
  using stress_t = math::matr<dim, dim>;
  using hessian_t = math::matr<dof_cnt, dof_cnt>;
  /**
   * @brief Construct a new Elasticity Base object
   *
   * @param lambda Lamé parameter
   * @param mu Lamé parameter
   */
  ElasticityBase(real lambda, real mu) : lambda_(lambda), mu_(mu) {}

  real Energy() const { return static_cast<Derived const*>(this)->Energy(); }

  stress_t Stress() const { return static_cast<Derived const*>(this)->Stress(); }

  math::mat<real, dof_cnt, dof_cnt> Hessian() const {
    return static_cast<Derived const*>(this)->Hessian();
  }

protected:
  // Deformation gradient
  math::matr<dim, dim> F_;

  // SVD of F
  math::matr<dim, dim> U_;
  math::matr<dim, dim> V_;
  math::vecr<dim> sigma_;
  bool is_svd_ready_{false};

  // Lamé parameters
  real lambda_, mu_;
};

}  // namespace ax::pde::elasticity