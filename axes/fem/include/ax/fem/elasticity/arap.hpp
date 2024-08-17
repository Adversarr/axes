#pragma once
#include "ax/math/linalg.hpp"
#include "base.hpp"

namespace ax::fem::elasticity {
/**
 * @brief ARAP Elasticity Model
 * @tparam dim
 */
template <Index dim> class IsotropicARAP final : public ElasticityBase<dim, IsotropicARAP<dim>> {
public:
  using base_t = ElasticityBase<dim, IsotropicARAP<dim>>;
  using stress_t = typename base_t::stress_t;
  using hessian_t = typename base_t::hessian_t;
  using ElasticityBase<dim, IsotropicARAP<dim>>::ElasticityBase;
  AX_HOST_DEVICE IsotropicARAP(real lambda, real mu): ElasticityBase<dim, IsotropicARAP<dim>>(lambda, mu) {}
  AX_HOST_DEVICE IsotropicARAP() : ElasticityBase<dim, IsotropicARAP<dim>>() {}

  /**
   * @brief Compute the ARAP Elasticity Potential value.
   *      Phi = mu * || F - R ||_F
   * where e is the Approximate Green Strain (Ignore the high order terms)
   * @return real
   */
  AX_HOST_DEVICE real EnergyImpl(DeformationGradient<dim> const& F,
                  const math::decomp::SvdResult<dim, real>& svd) const {
    const auto& mu = this->mu_;
    // F should be Ut Sigma V,
    math::RealMatrix<dim, dim> R = svd.U_ * svd.V_.transpose();
    return mu * math::norm2(F - R);
  }

  /**
   * @brief Compute the stress tensor.
   *
   * @return stress_t
   */
  AX_HOST_DEVICE stress_t StressImpl(DeformationGradient<dim> const& ,
                  math::decomp::SvdResult<dim, real> const& svd) const {
    real mu = this->mu_;
    math::RealMatrix<dim, dim> R = svd.U_ * svd.V_.transpose();
    math::RealMatrix<dim, dim> S = svd.V_ * svd.sigma_.asDiagonal() * svd.V_.transpose();
    return R * (2 * mu * (S - math::eye<dim>()));
  }

  /**
   * @brief Compute the HessianImpl of Pk1 StressImpl
   *
   * @return hessian_t
   */
  AX_HOST_DEVICE hessian_t HessianImpl(DeformationGradient<dim> const&,
                        const math::decomp::SvdResult<dim, real>& svd) const;

  bool EnergyRequiresSvd() const noexcept final { return true; }
  bool StressRequiresSvd() const noexcept final { return true; }
  bool HessianRequiresSvd() const noexcept final { return true; }
};

template<> typename IsotropicARAP<3>::hessian_t
AX_HOST_DEVICE AX_FORCE_INLINE IsotropicARAP<3>::HessianImpl(const DeformationGradient<3>&, const math::decomp::SvdResult<3, real>& svd) const {
  real s0 = svd.sigma_[0];
  real s1 = svd.sigma_[1];
  real s2 = svd.sigma_[2];
  auto const& U = svd.U_, V = svd.V_;
  // Twists:
  math::RealMatrix3 t0, t1, t2;
  t0.setZero(); t0(1, 2) = -1; t0(2, 1) = 1;
  t1.setZero(); t1(0, 2) = 1; t1(2, 0) = -1;
  t2.setZero(); t2(0, 1) = 1; t2(1, 0) = -1;

  // Eigen Vectors.
  real inv_sqrt2 = 1.0 / sqrt(2.0);
  t0 = inv_sqrt2 * U * t0 * V.transpose();
  t1 = inv_sqrt2 * U * t1 * V.transpose();
  t2 = inv_sqrt2 * U * t2 * V.transpose();

  math::RealVector<9> q0 = math::flatten(t0),
                q1 = math::flatten(t1),
                q2 = math::flatten(t2);

  real lambda0 = this->mu_ * 4.0 / (s1 + s2),
       lambda1 = this->mu_ * 4.0 / (s0 + s2),
       lambda2 = this->mu_ * 4.0 / (s0 + s1);

  math::RealMatrix<9, 9> dpdf = 2 * this->mu_ * math::eye<9>();
  dpdf -= lambda0 * (q0 * q0.transpose());
  dpdf -= lambda1 * (q1 * q1.transpose());
  dpdf -= lambda2 * (q2 * q2.transpose());
  return dpdf;
}


template<> typename IsotropicARAP<2>::hessian_t
AX_HOST_DEVICE AX_FORCE_INLINE IsotropicARAP<2>::HessianImpl(const DeformationGradient<2>&, const math::decomp::SvdResult<2, real>& svd) const {
  // I2 - 2 I1
  real s0 = svd.sigma_[0];
  real s1 = svd.sigma_[1];
  auto const& U = svd.U_, V = svd.V_;
  // Twists:
  math::RealMatrix2 t0;
  t0(0, 0) = t0(1, 1) = 0;
  t0(0, 1) = -1; t0(1, 0) = 1;
  const real inv_sqrt2 = 1.0 / sqrt(2.0);
  math::RealMatrix2 T = inv_sqrt2 * U * t0 * V.transpose();
  math::RealVector<4> q = math::flatten(T);

  IsotropicARAP<2>::hessian_t I1 = 2.0 / (s0 + s1) * q * q.transpose();
  math::RealMatrix<4, 4> dpdf = this->mu_ * (2 * math::eye<4>() - 2 * I1);
  return dpdf;
}

}  // namespace ax::fem::elasticity
