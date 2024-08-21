#include "ax/math/linsys/sparse/LDLT.hpp"
namespace ax::math {

void SparseSolver_LDLT::AnalyzePattern() {
  solver_.analyzePattern(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LDLT: AnalyzePattern failed");
}

void SparseSolver_LDLT::Factorize() {
  solver_.factorize(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LDLT: Factorization failed");
}

LinsysSolveResult SparseSolver_LDLT::Solve(RealMatrixX const &b, RealMatrixX const &) {
  RealMatrixX x = solver_.solve(b);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LDLT: solve failed");
  return {x};
}

}
