#pragma once
#include "axes/math/linsys/sparse.hpp"

namespace ax::math {

class SparseSolver_LeastSquaresConjugateGradient : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::LeastSquaresConjugateGradient<sp_matxxr> solver_;
};

}  // namespace ax::math