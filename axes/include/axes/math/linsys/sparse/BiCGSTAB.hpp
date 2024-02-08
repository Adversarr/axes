#pragma once
#include "axes/math/linsys/sparse.hpp"

namespace ax::math {

class SparseSolver_BiCGSTAB : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::BiCGSTAB<sp_matxxr> solver_;
};
}  // namespace ax::math