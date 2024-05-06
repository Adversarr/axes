#pragma once
#include "ax/fem/trimesh.hpp"
#include "ax/utils/opt.hpp"

namespace ax::fem {

/**
 * @brief The Poisson Problem is defined as follows:
 *    - ∇· (a ∇ u) = f in Ω
 *               u = g on ∂Ω_1
 *     s u + ∂u/∂n = h on ∂Ω_2
 *  If a is not set, it is assumed to be 1 uniformly.
 *  If s is not set, it is assumed to be 0 uniformly.
 * @note We only consider
 */
template <idx dim> struct PoissonProblemData {
  math::field1r f_;
  math::fieldr<dim> a_;
  math::field1r g_, h_, s_;

  // Although the TriMesh support Dirichlet BC, we still declare them here.
  std::vector<bool> dirichlet_bc_vertices_;
  std::vector<bool> neumann_bc_vertices_;
};

struct AssembledPoissonProblemData {
  math::sp_matxxr A_;
  math::vecxr b_;
};

template <idx dim> class PoissonSolver : public utils::Tunable {
public:
  PoissonSolver() = default;

  void SetMesh(SPtr<TriMesh<dim>> mesh) { mesh_ = mesh; }

  math::field1r Solve(PoissonProblemData<dim> const& data);

  AssembledPoissonProblemData Assemble(PoissonProblemData<dim> const& data);

  math::field1r Solve(AssembledPoissonProblemData const& data);

private:
  SPtr<TriMesh<dim>> mesh_;
};

}  // namespace ax::fem
