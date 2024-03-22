#pragma once
#include "base.hpp"
namespace ax::pde::elasticity {

/**
 * @brief St. Venant-Kirchhoff model
 * @note See Page 33 of "Dynamic Deformables", 3.6.2 The Complete St. Venant Kirchhoff
 * @tparam dim 
 */
template <idx dim> struct StVK: public ElasticityBase<dim, StVK<dim>>{
  using base_t = ElasticityBase<dim, StVK<dim>>;
  using stress_t = typename base_t::stress_t;
  using hessian_t = typename base_t::hessian_t;
  
  real Energy() const {
    const auto& F = this->F_;
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    math::matr<dim, dim> const E = green_strain(F);
    return mu * math::norm(E, math::l2) + 0.5 * lambda * math::square(math::trace(E));
  }

  // PStVK(F) = μFE + λ (tr E) F.
  stress_t Stress() const {
    const auto& F = this->F_;
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    math::matr<dim, dim> const E = green_strain(F);
    return 2.0 * mu * F * E + lambda * math::trace(E) * F;
  }

  /**
   * @brief Compute the Hessian of Pk1 Stress
   * @note See Page 45. of "Dynamic Deformables", 4.2.3 St. Venant-Kirchhoff: Things Get Worse
   * 
   * @return hessian_t 
   */
  hessian_t Hessian() const {
    hessian_t H = math::make_zeros<hessian_t>();
    const auto& F = this->F_;
    const auto& lambda = this->lambda_;
    const auto& mu = this->mu_;
    math::matr<dim, dim> const FtF = F.transpose() * F;
    // HII = 4(I ⊗ FFT + FTF ⊗ I +D)
    auto I = math::eye<dim>();
    for (idx i = 0; i < dim; ++i) {
      for (idx j = 0; j < dim; ++j) {
        real fij = F(i, j);
        auto Dij = F.col(j) * F.col(i).transpose();
        H.template block<dim, dim>(i * dim, j * dim) += (FtF + fij * I) + Dij;
      }
    }
    H *= 4.0;
    return H;
  }
};

}