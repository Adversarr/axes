#include "ax/math/linsys/sparse/QR.hpp"
namespace ax::math {

void SparseSolver_QR::AnalyzePattern() {
  AX_THROW_IF_FALSE(cached_problem_->A_.isCompressed(), "SparseSolver_QR: matrix is not compressed");
  solver_.analyzePattern(cached_problem_->A_);
}

void SparseSolver_QR::Factorize() {
  AX_THROW_IF_FALSE(cached_problem_->A_.isCompressed(), "SparseSolver_QR: matrix is not compressed");
  solver_.factorize(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_QR: factorization failed");
}

LinsysSolveResult SparseSolver_QR::Solve(RealMatrixX const &b, RealMatrixX const &) {
  RealMatrixX x = solver_.solve(b);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_QR: solve failed");
  return {x};
}

}  // namespace ax::math
