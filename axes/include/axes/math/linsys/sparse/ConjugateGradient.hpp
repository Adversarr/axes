#pragma once
#include "axes/math/linsys/sparse.hpp"

namespace ax::math {

class SparseSolver_ConjugateGradient : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::ConjugateGradient<sp_matxxr, Eigen::Lower | Eigen::Upper,
                           Eigen::DiagonalPreconditioner<real>>
      solver_;
};
}  // namespace ax::math