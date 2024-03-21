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

  stress_t PK1() const { return static_cast<Derived const*>(this)->PK1(); }

  math::mat<real, dof_cnt, dof_cnt> Hessian() const {
    return static_cast<Derived const*>(this)->Hessian();
  }

  // Compute the energy, force, and hessian at the same time.
  void Compute(real* energy, stress_t* force, hessian_t* hessian) {
    return static_cast<Derived*>(this)->Compute(energy, force, hessian);
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