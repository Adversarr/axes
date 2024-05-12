#include "ax/math/linsys/sparse/QR.hpp"
namespace ax::math {

void SparseSolver_QR::Analyse(LinsysProblem_Sparse const &problem) {
  // if (!problem.A_.isCompressed()) {
  //   return utils::InvalidArgumentError("SparseSolver_QR: matrix is not compressed");
  // }
  AX_THROW_IF_FALSE(problem.A_.isCompressed(), "SparseSolver_QR: matrix is not compressed");
  solver_.compute(problem.A_);
  // if (solver_.info() != Eigen::Success) {
  //   return utils::FailedPreconditionError("SparseSolver_QR: factorization failed");
  // }

  // AX_RETURN_OK();
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_QR: factorization failed");
}

LinsysSolveResult SparseSolver_QR::Solve(vecxr const &b, vecxr const &) {
  vecxr x = solver_.solve(b);
  // if (solver_.info() != Eigen::Success) {
  //   return utils::InvalidArgumentError("SparseSolver_CG: solve failed");
  // }
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_QR: solve failed");
  return x;
}

}  // namespace ax::math
