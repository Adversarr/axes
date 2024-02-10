#include "axes/math/linsys/sparse/QR.hpp"
namespace ax::math {

Status SparseSolver_QR::Analyse(LinsysProblem_Sparse const &problem) {
  if (!problem.A_.isCompressed()) {
    return utils::InvalidArgumentError("SparseSolver_CG: matrix is not compressed");
  }
  solver_.compute(problem.A_);
  if (solver_.info() != Eigen::Success) {
    return utils::FailedPreconditionError("SparseSolver_CG: factorization failed");
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_QR::Solve(vecxr const &b, vecxr const &) {
  vecxr x = solver_.solve(b);
  if (solver_.info() != Eigen::Success) {
    return utils::InvalidArgumentError("SparseSolver_CG: solve failed");
  }
  return x;
}

}  // namespace ax::math
