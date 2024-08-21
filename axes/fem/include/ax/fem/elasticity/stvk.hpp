#pragma once
#include "ax/math/functional.hpp"
#include "base.hpp"
#include "extra_symbols.hpp"

namespace ax::fem::elasticity {

/**
 * @brief St. Venant-Kirchhoff model
 * @note See Page 33 of "Dynamic Deformables", 3.6.2 The Complete St. Venant Kirchhoff
 * @tparam dim
 */
template <int dim> class StVK : public ElasticityBase<dim, StVK<dim>> {
public:
  using base_t = ElasticityBase<dim, StVK<dim>>;
  using stress_t = typename base_t::stress_t;
  using hessian_t = typename base_t::hessian_t;
  using ElasticityBase<dim, StVK<dim>>::ElasticityBase;

  AX_HOST_DEVICE StVK() = default;
  AX_HOST_DEVICE StVK(Real lambda, Real mu) : base_t(lambda, mu) {}

  AX_HOST_DEVICE Real EnergyImpl(DeformGrad<dim> const& F,
                  const math::decomp::SvdResult<dim, Real>&) const {
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    math::RealMatrix<dim, dim> const E = green_strain(F);
    return mu * math::norm2(E) + 0.5 * lambda * math::square(math::trace(E));
  }

  // PStVK(F) = μFE + λ (tr E) F.
  AX_HOST_DEVICE stress_t StressImpl(DeformGrad<dim> const& F,
                  math::decomp::SvdResult<dim, Real> const& ) const {
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    math::RealMatrix<dim, dim> const E = green_strain(F);
    return 2.0 * mu * F * E + lambda * math::trace(E) * F;
  }

  /**
   * @brief Compute the HessianImpl of Pk1 StressImpl
   * @note See Page 45. of "Dynamic Deformables", 4.2.3 St. Venant-Kirchhoff: Things Get Worse
   *
   * @return hessian_t
   */
  AX_HOST_DEVICE hessian_t HessianImpl(DeformGrad<dim> const& F,
                        const math::decomp::SvdResult<dim, Real>&) const {
    hessian_t H = math::make_zeros<hessian_t>();
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    // This is the first part, i.e. norm(E)
    math::RealMatrix<dim, dim> const FtF = F.transpose() * F;
    // HII = 4(I ⊗ FFT + FTF ⊗ I +D)
    auto I = math::eye<dim>();
    for (Index i = 0; i < dim; ++i) {
      for (Index j = 0; j < dim; ++j) {
        Real fij = F(i, j);
        auto Dij = F.col(j) * F.col(i).transpose();
        H.template block<dim, dim>(i * dim, j * dim) += (FtF + fij * I) + Dij;
      }
    }
    H -= math::eye<dim * dim>();
    H *= mu;

    // The second part comes from square(tr(E)).
    // tr(E) = tr(FtF-I) = ||F||_F - dim.
    // EnergyImpl = square(tr(E)) = square(||F||_F^2 - dim) = ||F||_F^4 - 2 * dim * ||F||_2^2 + dim^2
    // The first part is the most difficult (Equals to 4 (f f.T + 2 I)).
    // The second part is just some multiple of identity.
    // TODO: It seems to be correct?
    auto flattened = math::flatten(F);
    H += 0.5 * lambda * (flattened * flattened.transpose() + 2 * math::eye<dim * dim>());
    return H;
  }
};

}  // namespace ax::fem::elasticity
