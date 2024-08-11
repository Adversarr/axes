#include "ax/math/linsys/sparse/LU.hpp"
namespace ax::math {

void SparseSolver_LU::AnalyzePattern() {
  if (!cached_problem_->A_.isCompressed()) {
    cached_problem_->A_.makeCompressed();
  }
  solver_.analyzePattern(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LU: factorize failed {}", 
      to_string(solver_.info()));
}

void SparseSolver_LU::Factorize() {
  if (!cached_problem_->A_.isCompressed()) {
    cached_problem_->A_.makeCompressed();
  }
  solver_.factorize(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LU: factorize failed {}", 
      to_string(solver_.info()));
}

LinsysSolveResult SparseSolver_LU::Solve(matxxr const &b, matxxr const &) {
  matxxr x = solver_.solve(b);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LU: solve failed");
  return x;
}
}  // namespace ax::math
