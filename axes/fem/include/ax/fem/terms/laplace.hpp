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
 * For now, we just consider the isotropic diffusivity.
 *
 */
class LaplaceTerm : public TermBase {
public:
  LaplaceTerm(shared_not_null<State> state, shared_not_null<Mesh> mesh);

  // Set the diffusivity for each element.
  void SetDiffusivity(ConstRealBufferView diffusivity);
  void SetDiffusivity(Real uniform_diffusivity);

  void MarkDirty() override;
  void UpdateEnergy() override;
  void UpdateGradient() override;
  void UpdateHessian() override;

  void SetRhs(ConstRealBufferView rhs);

private:
  BufferPtr<Real> rhs_;
  BufferPtr<Real> diff_; ///< stores (u - rhs)
  bool is_diff_up_to_date_{false};
};

}  // namespace ax::fem