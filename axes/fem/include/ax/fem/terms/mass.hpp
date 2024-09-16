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
 */
class MassTerm : public TermBase {
public:
  MassTerm(std::shared_ptr<State> state, std::shared_ptr<Mesh> mesh);

  void SetDensity(ConstRealBufferView density);

  void SetDensity(Real uniform_density);

  Real Energy() const override;
  void UpdateGradient() override;
  void UpdateHessian() override;
};

}  // namespace ax::fem