#include "ax/math/linsys/sparse/LDLT.hpp"
namespace ax::math {

void SparseSolver_LDLT::Analyse(LinsysProblem_Sparse const &problem) {
  solver_.compute(problem.A_);
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LDLT: factorization failed");
}

LinsysSolveResult SparseSolver_LDLT::Solve(vecxr const &b, vecxr const &) {
  vecxr x = solver_.solve(b);
  // if (solver_.info() != Eigen::Success) {
  //   return utils::InvalidArgumentError("SparseSolver_LDLT: solve failed");
  // }
  AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_LDLT: solve failed");
  return x;
}

}