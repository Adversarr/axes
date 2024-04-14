#pragma once
#include "ax/math/linalg.hpp"
#include "base.hpp"

namespace ax::fem::elasticity {
/**
 * @brief ARAP Elasticity Model
 * @tparam dim
 */
template <idx dim> class IsotropicARAP final : public ElasticityBase<dim, IsotropicARAP<dim>> {
public:
  using base_t = ElasticityBase<dim, IsotropicARAP<dim>>;
  using stress_t = typename base_t::stress_t;
  using hessian_t = typename base_t::hessian_t;
  using ElasticityBase<dim, IsotropicARAP<dim>>::ElasticityBase;

  /**
   * @brief Compute the ARAP Elasticity Potential value.
   *      Phi = mu * || F - R ||_F
   * where e is the Approximate Green Strain (Ignore the high order terms)
   * @return real
   */
  real EnergyImpl(DeformationGradient<dim> const& F,
                  const math::decomp::SvdResultImpl<dim, real>* svd) const {
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    // F should be Ut Sigma V,
    math::matr<dim, dim> R = svd->U_ * svd->V_.transpose();
    return mu * math::norm2(F - R);
  }

  /**
   * @brief Compute the stress tensor.
   *
   * @return stress_t
   */
  stress_t StressImpl(DeformationGradient<dim> const& F,
                  math::decomp::SvdResultImpl<dim, real> const* svd) const {
    real lambda = this->lambda_;
    real mu = this->mu_;
    math::matr<dim, dim> R = svd->U_ * svd->V_.transpose();
    math::matr<dim, dim> S = svd->V_ * svd->sigma_.asDiagonal() * svd->V_.transpose();
    return R * (2 * mu * (S - math::eye<dim>()));
  }

  /**
   * @brief Compute the HessianImpl of Pk1 StressImpl
   *
   * @return hessian_t
   */
  hessian_t HessianImpl(DeformationGradient<dim> const&,
                        const math::decomp::SvdResultImpl<dim, real>* svd) const;

  bool EnergyRequiresSvd() const noexcept final { return true; }
  bool StressRequiresSvd() const noexcept final { return true; }
  bool HessianRequiresSvd() const noexcept final { return true; }
};

template<> typename IsotropicARAP<3>::hessian_t
IsotropicARAP<3>::HessianImpl(const DeformationGradient<3>&, const math::decomp::SvdResultImpl<3, real>* svd) const {
  real s0 = svd->sigma_[0];
  real s1 = svd->sigma_[1];
  real s2 = svd->sigma_[2];
  auto const& U = svd->U_, V = svd->V_;
  // Twists:
  math::mat3r t0, t1, t2;
  t0.setZero(); t0(1, 2) = -1; t0(2, 1) = 1;
  t1.setZero(); t1(0, 2) = 1; t1(2, 0) = -1;
  t2.setZero(); t2(0, 1) = 1; t2(1, 0) = -1;

  // Eigen Vectors.
  real inv_sqrt2 = 1.0 / math::sqrt<real>(2.0);
  t0 = inv_sqrt2 * U * t0 * V.transpose();
  t1 = inv_sqrt2 * U * t1 * V.transpose();
  t2 = inv_sqrt2 * U * t2 * V.transpose();

  math::vecr<9> q0 = math::flatten(t0),
                q1 = math::flatten(t1),
                q2 = math::flatten(t2);

  real lambda0 = this->mu_ * 4.0 / (s1 + s2),
       lambda1 = this->mu_ * 4.0 / (s0 + s2),
       lambda2 = this->mu_ * 4.0 / (s0 + s1);

  math::matr<9, 9> dpdf = 2 * this->mu_ * math::eye<9>();
  dpdf -= lambda0 * (q0 * q0.transpose());
  dpdf -= lambda1 * (q1 * q1.transpose());
  dpdf -= lambda2 * (q2 * q2.transpose());
  return dpdf;
}
}
