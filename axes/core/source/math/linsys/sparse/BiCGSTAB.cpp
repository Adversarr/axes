#include "ax/math/linsys/sparse/BiCGSTAB.hpp"
namespace ax::math {

void SparseSolver_BiCGSTAB::AnalyzePattern() {
  solver_.analyzePattern(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success,
                    "SparseSolver_BiCGSTAB: AnalyzePattern failed");
}

void SparseSolver_BiCGSTAB::Factorize() {
  solver_.factorize(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success,
                    "SparseSolver_BiCGSTAB: Compute failed");
}

LinsysSolveResult SparseSolver_BiCGSTAB::Solve(RealMatrixX const &b, RealMatrixX const &x0) {
  RealVectorX x;
  if (x0.size() > 0) {
    AX_THROW_IF_NE(x0.size(), b.size(), "SparseSolver_BiCGSTAB: x0 size mismatch");
    x = solver_.solveWithGuess(b, x0);
  } else {
    x = solver_.solve(b);
  }

  LinsysSolveResult impl(x, solver_.info() == Eigen::Success);
  impl.num_iter_ = solver_.iterations();
  impl.l2_err_ = solver_.error();
  return impl;
}

}  // namespace ax::math
