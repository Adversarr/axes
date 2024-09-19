#pragma once
#include "ax/fem/state.hpp"
#include "ax/math/sparse_matrix/block_matrix.hpp"

namespace ax::fem {

class PruneDirichletBc {
public:
  explicit PruneDirichletBc(std::shared_ptr<State> state);

  // For each Dirichlet dof, set grad to zero
  void Prune(RealBufferView grad);

  // For each (i, j) block, if i or j is a Dirichlet dof, set the entry to
  // 1. zero       if i != j
  // 2. Identity   if i == j
  void Prune(math::RealBlockMatrix& hessian);

  // For each (i, j) block, if i or j is a Dirichlet dof, set the entry to
  // 1. zero       if i != j
  // 2. Identity   if i == j
  // and the gradient will add the Dirichlet dof to the gradient properly
  void PruneWithGradient(math::RealBlockMatrix& hessian, RealBufferView grad);

  void UpdateDbcValue();

private:
  std::shared_ptr<State> state_;
  BufferPtr<Real> bc_var_; ///< stores the dbc value.
};

}  // namespace ax::fem