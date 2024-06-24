#include "ax/math/linsys/sparse/LU.hpp"
namespace ax::math {

void SparseSolver_LU::AnalyzePattern() {
  AX_THROW_IF_FALSE(cached_problem_->A_.isCompressed(),
                    "SparseSolver_LU: matrix is not compressed");
  solver_.analyzePattern(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LU: analyzePattern failed");
}

void SparseSolver_LU::Factorize() {
  AX_THROW_IF_FALSE(cached_problem_->A_.isCompressed(),
                    "SparseSolver_LU: matrix is not compressed");
  solver_.factorize(cached_problem_->A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LU: factorize failed");
}

LinsysSolveResult SparseSolver_LU::Solve(vecxr const &b, vecxr const &) {
  vecxr x = solver_.solve(b);
  // if (solver_.info() != Eigen::Success) {
  //   return utils::InvalidArgumentError("SparseSolver_LU: solve failed");
  // }
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LU: solve failed");
  return x;
}
}  // namespace ax::math
