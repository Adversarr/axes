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
template <idx dim> class StableNeoHookean
    : public ElasticityBase<dim, StableNeoHookean<dim>> {
public:
  using base_t = ElasticityBase<dim, StableNeoHookean<dim>>;
  using stress_t = typename base_t::stress_t;
  using hessian_t = typename base_t::hessian_t;

  using ElasticityBase<dim, StableNeoHookean<dim>>::ElasticityBase;
  AX_HOST_DEVICE StableNeoHookean() = default;
  AX_HOST_DEVICE StableNeoHookean(real lambda, real mu) : base_t(lambda, mu) {}

  AX_HOST_DEVICE constexpr real delta() const noexcept {
    if constexpr(dim == 2) {
      real mu = this->mu_, lambda = this->lambda_;
      const real t = mu / lambda;
      const real a = 2 * t + 1;   // > 0, 2t + 1.
      const real b = 2 * t * (dim - 1) + dim; // > 0, 2t + 2.
      const real c = -t * dim;    // < 0, -2t.
      const real delta = (-b + std::sqrt(b * b - 4 * a * c)) / (2 * a);
      return delta;
    } else /* dim == 3 */ {
      return 1.0;
    }
  }

  // energy= 1/2 mu (|F|^2 - dim) + 1/2 lambda (det(F) - alpha)^2 + mu/2 log(Ic + delta)
  AX_HOST_DEVICE real EnergyImpl(DeformationGradient<dim> const& F,
                  const math::decomp::SvdResult<dim, real>&) const {
    real mu = this->mu_, lambda = this->lambda_;
    real const Ic = F.squaredNorm();
    real const del= delta();
    real alpha = (1 - 1 / (dim + del)) * mu / lambda + 1;
    real const Jminus1 = math::det(F) - alpha;
    return 0.5 * (mu * (Ic - dim) + lambda * Jminus1 * Jminus1 - mu * math::log(Ic + del));
  }

  
  AX_HOST_DEVICE stress_t StressImpl(DeformationGradient<dim> const& F,
                  math::decomp::SvdResult<dim, real> const&) const {
    real mu = this->mu_, lambda = this->lambda_;
    real const Ic = math::norm2(F);
    real const del= delta();
    real alpha = (1 - 1 / (dim + del)) * mu / lambda + 1;
    real const Jminus1 = math::det(F) - alpha;
    math::matr<dim, dim> dJdF = details::partial_determinant(F);
    return lambda * Jminus1 * dJdF  + mu * (1.0 - 1.0 / (Ic + del)) * F;
  }


  AX_HOST_DEVICE hessian_t HessianImpl(DeformationGradient<dim> const& F,
                        const math::decomp::SvdResult<dim, real>&) const {
    real mu = this->mu_, lambda = this->lambda_;
    real const I2 = math::norm2(F);
    real const del= delta();
    real alpha = (1 - 1 / (dim + del)) * mu / lambda + 1;
    math::vecr<dim * dim> dJdF = math::flatten(details::partial_determinant(F));
    real I3 = math::det(F);
    real scale = lambda * (I3 - alpha);
    math::vecr<dim * dim> F_flat = math::flatten(F);
    hessian_t hess = mu * (1 - 1 / (I2 + del)) * math::eye<dim * dim>()
                  + mu * (2.0 / math::square(I2 + del)) * (F_flat * F_flat.transpose())
                  + lambda * dJdF * dJdF.transpose();
    details::add_HJ(hess, F, scale);
    return hess;
  }
};


}  // namespace ax::fem::elasticity
