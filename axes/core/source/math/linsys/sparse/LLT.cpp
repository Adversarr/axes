#include "ax/math/linsys/sparse/LLT.hpp"
namespace ax::math {

void SparseSolver_LLT::Analyse(LinsysProblem_Sparse const &problem) {
  solver_.compute(problem.A_);
  // if (solver_.info() != Eigen::Success) {
  //   return utils::FailedPreconditionError("SparseSolver_LLT: factorization failed");
  // }
  // AX_RETURN_OK();
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LLT: factorization failed");
}

LinsysSolveResult SparseSolver_LLT::Solve(vecxr const &b, vecxr const &) {
  vecxr x = solver_.solve(b);
  // if (solver_.info() != Eigen::Success) {
  //   return utils::InvalidArgumentError("SparseSolver_LLT: solve failed");
  // }
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LLT: solve failed");
  return x;
}

}