#pragma once
#include "ax/math/decomp/svd/common.hpp"
#include "ax/math/traits.hpp"
#include "common.hpp"

namespace ax::fem::elasticity {

// convert Young's modulus (E) and Poisson's ratio (nu) to Lamé parameters
AX_FORCE_INLINE Real compute_mu(Real E, Real nu) { return E / (2 * (1 + nu)); }

AX_FORCE_INLINE Real compute_lambda(Real E, Real nu) { return E * nu / ((1 + nu) * (1 - 2 * nu)); }

AX_FORCE_INLINE math::RealVector2 compute_lame(Real E, Real nu) {
  return {compute_lambda(E, nu), compute_mu(E, nu)};
}

/**
 * @brief Base class for describe materials, use CRTP pattern.
 * @note CRTP have a better performance than virtual function. And we can use virtual function
 *       to compute the hole energy, stress and hessian on mesh.
 */
template <int dim, typename Derived> class ElasticityBase {
public:
  static constexpr Index dof_cnt = dim * dim;
  using stress_t = math::RealMatrix<dim, dim>;
  using hessian_t = math::RealMatrix<dof_cnt, dof_cnt>;
  /**
   * @brief Construct a new Elasticity Base object
   *
   * @param lambda Lamé parameter
   * @param mu Lamé parameter
   */
  AX_HOST_DEVICE ElasticityBase(Real lambda, Real mu) : lambda_(lambda), mu_(mu) {}

  AX_HOST_DEVICE ElasticityBase() = default;

  virtual ~ElasticityBase() = default;

  AX_HOST_DEVICE AX_FORCE_INLINE Real Energy(DeformGrad<dim> const& F,
                              math::decomp::SvdResult<dim, Real> const& svdr) const {
    return static_cast<Derived const*>(this)->EnergyImpl(F, svdr);
  }

  AX_HOST_DEVICE AX_FORCE_INLINE stress_t Stress(DeformGrad<dim> const& F,
                                  const math::decomp::SvdResult<dim, Real>& svdr) const {
    return static_cast<Derived const*>(this)->StressImpl(F, svdr);
  }

  AX_HOST_DEVICE AX_FORCE_INLINE math::Matrix<Real, dof_cnt, dof_cnt> Hessian(
      DeformGrad<dim> const& F, math::decomp::SvdResult<dim, Real> const& svdr) const {
    return static_cast<Derived const*>(this)->HessianImpl(F, svdr);
  }

  AX_HOST_DEVICE void SetLame(Real lambda, Real mu) {
    lambda_ = lambda;
    mu_ = mu;
  }

  AX_HOST_DEVICE void SetLame(math::RealVector2 const& lame) { SetLame(lame[0], lame[1]); }

  virtual bool EnergyRequiresSvd() const noexcept { return false; }
  virtual bool StressRequiresSvd() const noexcept { return false; }
  virtual bool HessianRequiresSvd() const noexcept { return false; }

protected:
  // Lamé parameters
  Real lambda_, mu_;
};

}  // namespace ax::fem::elasticity
