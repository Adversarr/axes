#pragma once
#include "ax/core/gsl.hpp"
#include "ax/fem/state.hpp"
#include "ax/math/sparse_matrix/block_matrix.hpp"

namespace ax::fem {

class PruneDirichletBc {
public:
  explicit PruneDirichletBc(shared_not_null<State> state);

  // For each Dirichlet dof, set grad to zero
  void PruneGradient(RealBufferView grad);
  void PruneVariable(RealBufferView variables);

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

  ConstRealBufferView GetDbcValue() const { return bc_var_->ConstView(); }

private:
  std::shared_ptr<State> state_;
  BufferPtr<Real> bc_var_;  ///< stores the dbc value.
};

}  // namespace ax::fem