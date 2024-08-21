#pragma once
#include "base.hpp"
#include "extra_symbols.hpp"

namespace ax::fem::elasticity {


/**
 * @brief Stable NeoHookean Elasticity Model. (With barrier.)
 * @ref See HOBAKv1/src/Hyperelastic/Volume/SNH.cpp for details.
 *
 *  WARN: To be consistent with linear elasticity, set following.
 *    mu = 4.0/3.0 * this->mu_, lambda = this->lambda_ + 5.0/6.0;
 *
 * @tparam dim
 */
template <int dim> class StableNeoHookean
    : public ElasticityBase<dim, StableNeoHookean<dim>> {
public:
  using base_t = ElasticityBase<dim, StableNeoHookean<dim>>;
  using stress_t = typename base_t::stress_t;
  using hessian_t = typename base_t::hessian_t;

  using ElasticityBase<dim, StableNeoHookean<dim>>::ElasticityBase;
  AX_HOST_DEVICE StableNeoHookean() = default;
  AX_HOST_DEVICE StableNeoHookean(Real lambda, Real mu) : base_t(lambda, mu) {}

  AX_HOST_DEVICE constexpr Real delta() const noexcept {
    if constexpr(dim == 2) {
      Real mu = this->mu_, lambda = this->lambda_;
      const Real t = mu / lambda;
      const Real a = 2 * t + 1;   // > 0, 2t + 1.
      const Real b = 2 * t * (dim - 1) + dim; // > 0, 2t + 2.
      const Real c = -t * dim;    // < 0, -2t.
      const Real delta = (-b + std::sqrt(b * b - 4 * a * c)) / (2 * a);
      return delta;
    } else /* dim == 3 */ {
      return 1.0;
    }
  }

  // energy= 1/2 mu (|F|^2 - dim) + 1/2 lambda (det(F) - alpha)^2 + mu/2 log(Ic + delta)
  AX_HOST_DEVICE Real EnergyImpl(DeformGrad<dim> const& F,
                  const math::decomp::SvdResult<dim, Real>&) const {
    Real mu = this->mu_, lambda = this->lambda_;
    Real const Ic = F.squaredNorm();
    Real const del= delta();
    Real alpha = (1 - 1 / (dim + del)) * mu / lambda + 1;
    Real const Jminus1 = math::det(F) - alpha;
    return 0.5 * (mu * (Ic - dim) + lambda * Jminus1 * Jminus1 - mu * math::log(Ic + del));
  }

  
  AX_HOST_DEVICE stress_t StressImpl(DeformGrad<dim> const& F,
                  math::decomp::SvdResult<dim, Real> const&) const {
    Real mu = this->mu_, lambda = this->lambda_;
    Real const Ic = math::norm2(F);
    Real const del= delta();
    Real alpha = (1 - 1 / (dim + del)) * mu / lambda + 1;
    Real const Jminus1 = math::det(F) - alpha;
    math::RealMatrix<dim, dim> dJdF = details::partial_determinant(F);
    return lambda * Jminus1 * dJdF  + mu * (1.0 - 1.0 / (Ic + del)) * F;
  }


  AX_HOST_DEVICE hessian_t HessianImpl(DeformGrad<dim> const& F,
                        const math::decomp::SvdResult<dim, Real>&) const {
    Real mu = this->mu_, lambda = this->lambda_;
    Real const I2 = math::norm2(F);
    Real const del= delta();
    Real alpha = (1 - 1 / (dim + del)) * mu / lambda + 1;
    math::RealVector<dim * dim> dJdF = math::flatten(details::partial_determinant(F));
    Real I3 = math::det(F);
    Real scale = lambda * (I3 - alpha);
    math::RealVector<dim * dim> F_flat = math::flatten(F);
    hessian_t hess = mu * (1 - 1 / (I2 + del)) * math::eye<dim * dim>()
                  + mu * (2.0 / math::square(I2 + del)) * (F_flat * F_flat.transpose())
                  + lambda * dJdF * dJdF.transpose();
    details::add_HJ(hess, F, scale);
    return hess;
  }
};


}  // namespace ax::fem::elasticity
