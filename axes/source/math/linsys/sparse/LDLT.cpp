#include "axes/math/linsys/sparse/LDLT.hpp"
namespace ax::math {

Status SparseSolver_LDLT::Analyse(LinsysProblem_Sparse const &problem) {
  solver_.compute(problem.A_);

  if (solver_.info() != Eigen::Success) {
    return utils::FailedPreconditionError("SparseSolver_LDLT: factorization failed");
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_LDLT::Solve(vecxr const &b, vecxr const &x0) {
  vecxr x = solver_.solve(b);
  if (solver_.info() != Eigen::Success) {
    return utils::InvalidArgumentError("SparseSolver_LDLT: solve failed");
  }
  return x;
}

}