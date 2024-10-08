#pragma once
#include "base.hpp"
#include "extra_symbols.hpp"
namespace ax::fem::elasticity {
/**
 * @brief NeoHookean Elasticity Model.
 * @tparam dim
 */
template <int dim> class NeoHookeanBW : public ElasticityBase<dim, NeoHookeanBW<dim>> {
public:
  using base_t = ElasticityBase<dim, NeoHookeanBW<dim>>;
  using stress_t = typename base_t::stress_t;
  using hessian_t = typename base_t::hessian_t;

  using ElasticityBase<dim, NeoHookeanBW<dim>>::ElasticityBase;
  AX_HOST_DEVICE NeoHookeanBW(Real lambda, Real mu): ElasticityBase<dim, NeoHookeanBW<dim>>(lambda, mu) {}
  AX_HOST_DEVICE NeoHookeanBW() : ElasticityBase<dim, NeoHookeanBW<dim>>() {}

  /**
   * @brief Compute the NeoHookean Elasticity Potential value.
   *      Phi = 0.5 mu (||F||_f-3) - mu Log[J] + lambda/2 (Log[J])^2
   * where J is the determinant of F
   * @return Real
   */
  AX_HOST_DEVICE Real EnergyImpl(DeformGrad<dim> const& F,
                  const math::decomp::SvdResult<dim, Real>&) const {
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    Real J = math::det(F);
    if_unlikely (J <= 0) {
      return math::inf<Real>;
    }
    return 0.5 * mu * (math::norm2(F) - dim) - mu * math::log(J)
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
   AX_HOST_DEVICE stress_t StressImpl(DeformGrad<dim> const& F,
                  math::decomp::SvdResult<dim, Real> const&) const {
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    Real J = math::det(F);
    if_unlikely(J <= 0) {
      return math::constant<dim, dim, Real>(math::inf<Real>);
    }
    math::RealMatrix<dim, dim> dJdF = details::partial_determinant(F);
    return mu * (F - 1 / J * dJdF) + lambda * math::log(J) / J * dJdF;
  }

  /**
   * @brief Compute the HessianImpl of Pk1 StressImpl
   * @note "Dynamic Deformables": 4.2.2.3 The Full HessianImpl
   * μI9×9+ (mu+lambda(1-log[J]))/J^2 gJgJ^T + (λlog[J]-μ)/J Hj
   * @return hessian_t
   */
  AX_HOST_DEVICE hessian_t HessianImpl(DeformGrad<dim> const& F,
                        const math::decomp::SvdResult<dim, Real>&) const {
    const Real& mu = this->mu_;
    const Real& lambda = this->lambda_;
    Real J = math::det(F);
    if_unlikely(J <= 0) {
      return math::constant<dim * dim, dim * dim, Real>(math::inf<Real>);
    }
    // First part
    math::RealMatrix<dim * dim, dim * dim> H = math::eye<dim * dim>() * mu;
    // Second part
    math::RealVector<dim * dim> dJdF_flat = math::flatten(details::partial_determinant(F));
    Real scale = (mu + lambda*(1-math::log(J))) / math::square(J);
    H += scale * dJdF_flat * dJdF_flat.transpose();
    // Third part
    details::add_HJ(H, F, (lambda * math::log(J) - mu) / J);
    return H;
  }
};

}  // namespace ax::fem::elasticity
