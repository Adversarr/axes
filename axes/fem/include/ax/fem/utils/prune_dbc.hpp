#pragma once
#include "ax/fem/state.hpp"
#include "ax/math/block_matrix/block_matrix.hpp"

namespace ax::fem {

class PruneDirichletBc {
public:
  explicit PruneDirichletBc(std::shared_ptr<State> state) : state_(std::move(state)) {}

  // For each Dirichlet dof, set grad to zero
  void Prune(RealBufferView grad);

  // For each (i, j) block, if i or j is a Dirichlet dof, set the entry to
  // 1. zero       if i != j
  // 2. Identity   if i == j
  void Prune(math::RealBlockMatrix& hessian);

private:
  std::shared_ptr<State> state_;
};

}  // namespace ax::fem