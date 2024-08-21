#include "ax/math/linsys/sparse/LLT.hpp"
namespace ax::math {

void SparseSolver_LLT::AnalyzePattern() {
  solver_.analyzePattern(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LLT: AnalyzePattern failed");
}

void SparseSolver_LLT::Factorize() {
  solver_.factorize(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LLT: Factorization failed");
}

LinsysSolveResult SparseSolver_LLT::Solve(RealMatrixX const &b, RealMatrixX const &) {
  RealMatrixX x = solver_.solve(b);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LLT: solve failed");
  return {x};
}

}
