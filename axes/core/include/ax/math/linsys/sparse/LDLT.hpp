#pragma once
#include "ax/math/linsys/sparse.hpp"

namespace ax::math {

class SparseSolver_LDLT : public SparseSolverBase {
public:
  void Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

  SparseSolverKind Kind() const final { return SparseSolverKind::kLDLT; }

  Eigen::SimplicialLDLT<sp_matxxr> solver_;
};
}  // namespace ax::math
