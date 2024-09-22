#pragma once
#include "ax/fem/mesh.hpp"
#include "ax/fem/problem.hpp"

namespace ax::fem {

/**
 * @brief Mass Term for FEM.
 *
 * Variational form of the mass term:
 *    Integrate_Omega rho * v * u dx = Integrate_Omega rho * v * f dx
 * i.e.
 *    in a(u, v) = <f, v>, a is just the inner product with mass as the kernel.
 *
 * @note Energy/Gradient/Hessian are:
 *   Energy   = 0.5 * Integrate_Omega rho * (u - rhs)^2 dx.
 *   Gradient = MassMatrix * (u - rhs).
 *   Hessian  = MassMatrix.
 */
class MassTerm : public TermBase {
public:
  MassTerm(shared_not_null<State> state, shared_not_null<Mesh> mesh);

  void SetDensity(ConstRealBufferView density);
  void SetDensity(Real uniform_density);

  void MarkDirty() override;
  void UpdateEnergy() override;
  void UpdateGradient() override;
  void UpdateHessian() override;

  void SetRhs(ConstRealBufferView rhs);

  RealBufferView GetRhs() { return rhs_->View(); }

  ConstRealBufferView GetRhs() const { return rhs_->ConstView(); }

private:
  // The RHS vector without Mass multiplied.
  BufferPtr<Real> rhs_;   ///< for dynamics, the external **ACCELERATION**
  BufferPtr<Real> diff_;  ///< stores (u - rhs)
  bool is_diff_up_to_date_{false};
};

}  // namespace ax::fem