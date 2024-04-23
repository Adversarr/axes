#pragma once
#include "ax/math/decomp/svd/common.hpp"
#include "ax/math/traits.hpp"
#include "common.hpp"

namespace ax::fem::elasticity {

// convert Young's modulus (E) and Poisson's ratio (nu) to Lamé parameters
AX_FORCE_INLINE real compute_mu(real E, real nu) { return E / (2 * (1 + nu)); }

AX_FORCE_INLINE real compute_lambda(real E, real nu) { return E * nu / ((1 + nu) * (1 - 2 * nu)); }

AX_FORCE_INLINE math::vec2r compute_lame(real E, real nu) {
  return {compute_lambda(E, nu), compute_mu(E, nu)};
}

/**
 * @brief Base class for describe materials, use CRTP pattern.
 * @note CRTP have a better performance than virtual function. And we can use virtual function
 *       to compute the hole energy, stress and hessian on mesh.
 */
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
  AX_HOST_DEVICE ElasticityBase(real lambda, real mu) : lambda_(lambda), mu_(mu) {}

  AX_HOST_DEVICE ElasticityBase() = default;

  AX_HOST_DEVICE AX_FORCE_INLINE real Energy(DeformationGradient<dim> const& F,
                              math::decomp::SvdResultImpl<dim, real> const& svdr) const {
    return static_cast<Derived const*>(this)->EnergyImpl(F, svdr);
  }

  AX_HOST_DEVICE AX_FORCE_INLINE stress_t Stress(DeformationGradient<dim> const& F,
                                  const math::decomp::SvdResultImpl<dim, real>& svdr) const {
    return static_cast<Derived const*>(this)->StressImpl(F, svdr);
  }

  AX_HOST_DEVICE AX_FORCE_INLINE math::mat<real, dof_cnt, dof_cnt> Hessian(
      DeformationGradient<dim> const& F, math::decomp::SvdResultImpl<dim, real> const& svdr) const {
    return static_cast<Derived const*>(this)->HessianImpl(F, svdr);
  }

  AX_HOST_DEVICE void SetLame(real lambda, real mu) {
    lambda_ = lambda;
    mu_ = mu;
  }

  AX_HOST_DEVICE void SetLame(math::vec2r const& lame) { SetLame(lame[0], lame[1]); }

  virtual bool EnergyRequiresSvd() const noexcept { return false; }
  virtual bool StressRequiresSvd() const noexcept { return false; }
  virtual bool HessianRequiresSvd() const noexcept { return false; }

protected:
  // Lamé parameters
  real lambda_, mu_;
};

}  // namespace ax::fem::elasticity
