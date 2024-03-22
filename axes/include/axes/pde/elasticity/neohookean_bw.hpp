#pragma once
#include "base.hpp"
namespace ax::pde::elasticity {
/**
 * @brief NeoHookean Elasticity Model.
 * @tparam dim
 */
template <idx dim> struct NeoHookeanBW : public ElasticityBase<dim, NeoHookeanBW<dim>> {
  using base_t = ElasticityBase<dim, NeoHookeanBW<dim>>;
  using stress_t = typename base_t::stress_t;
  using hessian_t = typename base_t::hessian_t;

  /**
   * @brief Compute the NeoHookean Elasticity Potential value.
   *      Phi = 0.5 mu (||F||_f-3) - mu Log[J] + lambda/2 (Log[J])^2
   * where J is the determinant of F
   * @return real
   */
  real Energy() const {
    const auto& F = this->F_;
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    real J = math::det(F);
    if_unlikely (J <= 0) {
      return math::inf<real>;
    }
    return 0.5 * mu * (math::norm(F, math::l2) - 3) - mu * math::log(J)
           + 0.5 * lambda * math::square(math::log(J));
  }

  /**
   * @brief Compute the stress tensor.
   * @note See "Dynamic Deformables": 4.2.2 Neo-Hookean: Not So Easy
   * Equation:
   *    P = mu (F- 1/J (dJ/dF)) + lambda Log[J]/J (dJ/dF)
   *
   * @return stress_t
   */
  stress_t Stress() const {
    const auto& F = this->F_;
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    real J = math::det(F);
    if_unlikely(J <= 0) {
      return math::constant<dim, dim, real>(math::inf<real>);
    }
    math::matr<dim, dim> dJdF = details::partial_determinant(F);
    return mu * (F - 1 / J * dJdF) + lambda * math::log(J) / J * dJdF;
  }

  /**
   * @brief Compute the Hessian of Pk1 Stress
   * @note "Dynamic Deformables": 4.2.2.3 The Full Hessian
   * μI9×9+ (mu+lambda(1-log[J]))/J^2 gJgJ^T + (λlog[J]-μ)/J Hj
   * @return hessian_t
   */
  hessian_t Hessian() const {
    const auto& F = this->F_;
    const real& mu = this->mu_;
    const real& lambda = this->lambda_;
    real J = math::det(F);
    if_unlikely(J <= 0) {
      return math::constant<dim * dim, dim * dim, real>(math::inf<real>);
    }
    // First part
    auto H = math::eye<dim * dim>() * mu;
    // Second part
    math::vecr<dim * dim> dJdF_flat = math::flatten(details::partial_determinant(F));
    real scale = (mu + lambda*(1-math::log(J))) / math::square(J);
    H += scale * dJdF_flat * dJdF_flat.transpose();
    // Third part
    details::add_HJ(H, F, (lambda * math::log(J) - mu) / J);
    return H;
  }
};

}  // namespace ax::pde::elasticity