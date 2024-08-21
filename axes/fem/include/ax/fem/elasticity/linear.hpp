#pragma once
#include "base.hpp"
#include "extra_symbols.hpp"

namespace ax::fem::elasticity {
/**
 * @brief Linear Elasticity Model.
 * @tparam dim
 */
template <int dim> class Linear : public ElasticityBase<dim, Linear<dim>> {
  public:
  using base_t = ElasticityBase<dim, Linear<dim>>;
  using stress_t = typename base_t::stress_t;
  using hessian_t = typename base_t::hessian_t;
  using ElasticityBase<dim, Linear<dim>>::ElasticityBase;

  AX_HOST_DEVICE Linear(Real lambda, Real mu): ElasticityBase<dim, Linear<dim>>(lambda, mu) {}
  AX_HOST_DEVICE Linear() : ElasticityBase<dim, Linear<dim>>() {}

  /**
   * @brief Compute the Linear Elasticity Potential value.
   *      Phi = mu * |e|_F + la / 2 * (trace_e)^2;
   * where e is the Approximate Green Strain (Ignore the high order terms)
   * @return Real
   */
  AX_HOST_DEVICE Real EnergyImpl(DeformGrad<dim> const& F, const math::decomp::SvdResult<dim, Real>&) const {
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    math::RealMatrix<dim, dim> const E = approx_green_strain(F);
    Real E_trace = E.trace();
    return mu * math::norm2(E) + 0.5 * lambda * E_trace * E_trace;
  }

  /**
   * @brief Compute the stress tensor.
   *
   * @return stress_t
   */
  AX_HOST_DEVICE stress_t StressImpl(DeformGrad<dim> const& F,
                  math::decomp::SvdResult<dim, Real> const&) const {
    Real lambda = this->lambda_;
    Real mu = this->mu_;
    math::RealMatrix<dim, dim> const E = approx_green_strain(F);
    return 2 * mu * E + lambda * E.trace() * math::eye<dim>();
  }

  /**
   * @brief Compute the HessianImpl of Pk1 StressImpl
   *
   * @return hessian_t
   */
  AX_HOST_DEVICE hessian_t HessianImpl(DeformGrad<dim> const&, const math::decomp::SvdResult<dim, Real>&) const {
    hessian_t H = math::eye<dim * dim>() * this->mu_;
    // mu * dF.transpose().
    for (Index i = 0; i < dim; ++i) {
      for (Index j = 0; j < dim; ++j) {
        H(i * dim + j, j * dim + i) += this->mu_;
      }
    }

    // la * dF.trace() * I.
    for (Index i = 0; i < dim; ++i) {
      for (Index j = 0; j < dim; ++j) {
        H(i * dim + i, j * dim + j) += this->lambda_;
      }
    }

    return H;
  }
};

}  // namespace ax::fem::elasticity
