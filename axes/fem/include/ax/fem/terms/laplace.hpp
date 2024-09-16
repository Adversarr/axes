#pragma once
#include "ax/fem/mesh.hpp"
#include "ax/fem/problem.hpp"

namespace ax::fem {

/**
 * @brief Define the Laplace term for FEM.
 *
 * Variational form of the Laplace term:
 *   Integrate_Omega a * grad(u) * grad(v) dx = Integrate_Omega f * v dx
 * where a is the diffusivity, and f is the source term.
 *
 */
class LaplaceTerm : public TermBase {
public:
  LaplaceTerm(std::shared_ptr<State> state, std::shared_ptr<Mesh> mesh);

  // Set the diffusivity for each element.
  void SetDiffusivity(ConstRealBufferView diffusivity);
  void SetDiffusivity(Real uniform_diffusivity);

  Real Energy() const override;
  void UpdateGradient() override;
  void UpdateHessian() override;
};

}  // namespace ax::fem