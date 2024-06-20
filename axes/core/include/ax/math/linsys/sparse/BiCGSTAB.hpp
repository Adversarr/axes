#pragma once
#include "ax/math/linsys/sparse.hpp"

namespace ax::math {

class SparseSolver_BiCGSTAB : public SparseSolverBase {
public:
  void Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

  SparseSolverKind GetKind() const final { return SparseSolverKind::kBiCGSTAB; }

private:
  Eigen::BiCGSTAB<sp_matxxr> solver_;
};
}  // namespace ax::math