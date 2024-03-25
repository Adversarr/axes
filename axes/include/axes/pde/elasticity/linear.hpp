#pragma once
#include "base.hpp"
namespace ax::pde::elasticity {
/**
 * @brief Linear Elasticity Model.
 * @tparam dim 
 */
template <idx dim> struct Linear: public ElasticityBase<dim, Linear<dim>>{
  using base_t = ElasticityBase<dim, Linear<dim>>;
  using stress_t = typename base_t::stress_t;
  using hessian_t = typename base_t::hessian_t;
  using base_t::ElasticityBase;

  /**
   * @brief Compute the Linear Elasticity Potential value.
   *      Phi = mu * |e|_F + la / 2 * (trace_e)^2;
   * where e is the Approximate Green Strain (Ignore the high order terms)
   * @return real 
   */
  real Energy() const {
    const auto& F = this->F_;
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    math::matr<dim, dim> const E = approx_green_strain(F);
    real E_trace = E.trace();
    return mu * math::norm(E, math::l2) + 0.5 * lambda * E_trace * E_trace;
  }

  /**
   * @brief Compute the stress tensor.
   * 
   * @return stress_t 
   */
  stress_t Stress() const {
    const auto& F = this->F_;
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    math::matr<dim, dim> const E = approx_green_strain(F);
    return 2 * mu * E + lambda * E.trace() * math::eye<dim>();
  }

  /**
   * @brief Compute the Hessian of Pk1 Stress
   * 
   * @return hessian_t 
   */
  hessian_t Hessian() const {
    hessian_t H = math::eye<dim * dim>() * this->mu_;
    for (idx i = 0; i < dim; ++i) {
      H(i * (dim + 1), i * (dim + 1)) += this->mu_ + this->lambda_;
    }
    return H;
  }
};

}