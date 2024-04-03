#include "ax/math/linsys/sparse/LU.hpp"
namespace ax::math {

Status SparseSolver_LU::Analyse(LinsysProblem_Sparse const &problem) {
  if (!problem.A_.isCompressed()) {
    return utils::InvalidArgumentError("SparseSolver_LU: matrix is not compressed");
  }
  solver_.compute(problem.A_);
  if (solver_.info() != Eigen::Success) {
    return utils::FailedPreconditionError("SparseSolver_LU: analyzePattern failed");
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_LU::Solve(vecxr const &b, vecxr const &) {
  vecxr x = solver_.solve(b);
  if (solver_.info() != Eigen::Success) {
    return utils::InvalidArgumentError("SparseSolver_LU: solve failed");
  }
  return x;
}
}
